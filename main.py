
import time
from threading import Thread, Lock
from pyModbusTCP.client import ModbusClient
from time import sleep
import json

# https://pymodbustcp.readthedocs.io/en/stable/examples/client_thread.html

SERVER_HOST = '192.168.59.35'
SERVER_PORT = 502


'''
@ TODO: 🔲 ✅

🔲 Timeouts is very long also doesnt crop up when setting up the device when unplugged

🔲 Error 0x0021 2 Variable holds the error code of the last error. 
 must be read or set to 0 to clear.
 — 0

🔲 in case calculating is troublesome  Microstep resolution 0x0048
🔲 error handling, logging
🔲 make class importable as we need to run this with an UI
🔲 read and write process may collide so there needs to be a semaphore of sorts

Stall mode 1 0x007A 1

'''


def convert_value_to_register(self, value, value_range, register_count):
    clipped_value = max(min(value, value_range[1]), value_range[0])
    if clipped_value != value:
        print(f"Value: {value} was out of range {value_range}. Clipped to {clipped_value}")
        value = clipped_value
    abs_range = sum(abs(x) for x in value_range)
    # checking if it's a single register write, if so we can already skip all conversion
    if abs_range < self.max_register_range:
        return [value]
    else:
        high_register = (value >> 16) & 0xFFFF
        low_register = value & 0xFFFF
        return [low_register, high_register]


def set_slew_revs_minute(self, revs):
    value = round((revs/60) * self.steps_per_rev)
    self.writeActions["slew"].set_value(value)


class WriteCommand:
    def __init__(self, register, value_range, register_count):
        self.register = register
        self.value_range = value_range
        self.register_count = register_count

    def set_value(self, value):
        register_value = convert_value_to_register(value, self.value_range, self.register_count)
        res = modbus_client.write_multiple_registers(self.register, register_value)
        if not (modbus_client.last_error and modbus_client.last_except):
            return True

        # does this reset automatically with the next command? no manual function for that listed afaik
        print(modbus_client.last_error_as_txt)
        print(modbus_client.last_except_as_full_txt)

        return False

class ReadCommand:
    def __init__(self, register, register_count=1):
        self.register = register
        self.register_count = register_count

    def get_regs(self, check_for_errors=True):
        regs_l = modbus_client.read_holding_registers(self.register, self.register_count)
        if check_for_errors:
            if regs_l is None:
                print("communication error, no value returned")
        return regs_l


class ModBuscontroller():

    register_size = 16
    max_register_range = 1 << register_size    # amount of value, the max value is one less since 0 is also a number

    # default of 256, see command 0x0048
    steps_per_rev = 51200

    def __init__(self):
        self.modbus_client = ModbusClient(host=SERVER_HOST, port=SERVER_PORT, auto_open=True, timeout=5)
        modbus_lock = Lock()
        # start polling thread
        self.polling_thread = Thread(target=self.polling_thread)
        # set daemon: polling thread will exit if main thread exit
        self.polling_thread.daemon = True
        self.polling_thread.start()

        self.writeActions = {
            "slew": WriteCommand(0x0078, (-5000000, +5000000), 2),
            "holdCurrent": WriteCommand(0x0029, (0, 100), 1),   # default 5
            "runCurrent": WriteCommand(0x0067, (0, 100), 1),    # default 25
            "setTorque": WriteCommand(0x00A6, (0, 100), 1),     # default 25
            "setMaxVelocity": WriteCommand(0x008B, (+1, 2560000), 2),
            "error": WriteCommand(0x0021, (0, 0), 1),
            "driveEnable": WriteCommand(0x001C, (0, 1), 1),
            # default is 256 - 51200 steps / rev
            "microStep": WriteCommand(0x0048, (1, 256), 1),
            "encodeEnable": WriteCommand(0x001E, (0, 1), 1)
        }

        self.readAction = {
            "stalled": ReadCommand(0x007B),
            "moving": ReadCommand(0x004A),
            "outputFault": ReadCommand(0x004E),
            "error": ReadCommand(0x0021)
        }


    '''
    # 1 << 19 = 524288
    # [65535, (1 << 4)] seems to be the fastest speed it will actually run
    
    write_multiple_registers(0x0078
    [65535, 0] CW rot
    [0, 65535] CCW rot
    entering any negative number is out of range
    [x, 65535] CCW rot the larger x the slower 
    [65535, 65535] fails silently or pretty much doesnt move
    [1, 0] is extremely slow probably the LSB here
    [0, 1] is faster
    
    msb should be in the second register?
    
    
    [30534, ~300] goes crazy and stops moviing
    [65535, 1] CW rot
    
    [65535, 5000000-65535] fails?
    
    '''

    def get_cfg():
        try:
            with open('config.json', 'r') as config_file:
                return json.load(config_file)
        except FileNotFoundError:
            print("missing config file")
        except json.decoder.JSONDecodeError as err:
            print(f"Config error:\n{err} \ncannot open config")
        exit()


    def run_preset(cfg):
        # wild guess we are working with non programmers or matlab "people" (such an evil word)
        start_index = cfg.get("startAt", 1) - 1
        # print(step)
        intervals = cfg.get("timeRevIntervals")
        # print(intervals)

        for interval in intervals[start_index:]:
            set_slew_revs_minute(interval[1])
            sleep(interval[0])

    def polling_thread():
        """Modbus polling thread."""

        while True:
            print(f"stalled: {readAction['stalled'].get_regs()}")
            print(f"moving: {readAction['moving'].get_regs()}")
            print(f"outputFault: {readAction['outputFault'].get_regs()}")
            print(f"error: {readAction['error'].get_regs()}")
            time.sleep(1.5)


def main():

    res = readAction["error"].get_regs(False)
    if res is None or not writeActions["driveEnable"].set_value(1):
        exit("No device found, check connections")
    print("initialisation successful")

    sleep(1)
    # run_preset(get_cfg())

    try:
        writeActions["runCurrent"].set_value(10)
        writeActions["encodeEnable"].set_value(1)
        set_slew_revs_minute(20)
        sleep(10)
    except KeyboardInterrupt:
        writeActions["slew"].set_value(0)

    writeActions["slew"].set_value(0)


    '''
    print("testing")

    print(modbus_client.write_multiple_registers(0x0078, [0, 0]))
    sleep(1)
    writeActions["runCurrent"].set_value(100)
    writeActions["setTorque"].set_value(100)
    
    
    # writeActions["error"].set_value(0)
    print("success??")
    sleep(3)
    print(modbus_client.last_error)
    print(modbus_client.last_except)
    print(modbus_client.last_error_as_txt)
    print(modbus_client.last_except_as_full_txt)
    print(modbus_client.write_multiple_registers(0x0078, [0, 0]))
    exit()

    
    
    '''





main()
