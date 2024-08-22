
import time
from threading import Thread, Lock
from pyModbusTCP.client import ModbusClient
from time import sleep
import json


SERVER_HOST = '192.168.59.35'
SERVER_PORT = 502

'''
https://pymodbustcp.readthedocs.io/en/stable/examples/client_thread.html
https://novantaims.com/downloads/manuals/modbus_tcp.pdf
@ TODO: 🔲 ✅

🔲 Timeouts is very long also doesnt crop up when setting up the device when unplugged
🔲 Error 0x0021 2 Variable holds the error code of the last error. 
 must be read or set to 0 to clear.
 — 0

🔲 in case calculating is troublesome  Microstep resolution 0x0048
🔲 error handling, logging
🔲 reset error flag after printing, make a function to print and reset errors, this can be later hooked up to logging
✅ make class importable as we need to run this with an UI
🔲 read and write process may collide so there needs to be a semaphore of sorts
🔲 register_count is not properly handled
🔲 self.last_slew = value for running the writecommand
🔲 volumenstrom berechnen


'''


class ModbusController:

    register_size = 16
    max_register_range = 1 << register_size    # amount of value, the max value is one less since 0 is also a number
    # default of 256, see command 0x0048
    steps_per_rev = 51200

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
            high_register = (value >> self.register_size) & 0xFFFF
            low_register = value & 0xFFFF
            return [low_register, high_register]

    class ReadCommand:
        def __init__(self, modbus, register, register_count=1):
            self.register = register
            self.register_count = register_count
            self.modbus = modbus

        def get_regs(self):
            self.modbus.bus_semaphore.acquire()
            regs_l = self.modbus.client.read_holding_registers(self.register, self.register_count)
            self.modbus.bus_semaphore.release()

            if regs_l is None:
                print("communication error, no value returned\n")

            if len(regs_l) > 2:
                print("unexpected return length")
                return False
            if len(regs_l) == 2:
                low_register, high_register = regs_l
                combined_value = (high_register << 16) | low_register
                if combined_value & (1 << 31):
                    combined_value -= (1 << 32)
                regs_l = [combined_value]

            return regs_l[0]

    class WriteCommand:
        def __init__(self, modbus, register, value_range, register_count):
            self.modbus = modbus
            self.register = register
            self.value_range = value_range
            self.register_count = register_count

        def set_value(self, value):
            register_value = self.modbus.convert_value_to_register(value, self.value_range, self.register_count)
            self.modbus.bus_semaphore.acquire()
            res = self.modbus.client.write_multiple_registers(self.register, register_value)
            self.modbus.bus_semaphore.release()
            if res or not (self.modbus.client.last_error and self.modbus.client.last_except):
                return True

            # does this reset automatically with the next command? no manual function for that listed afaik
            print(self.modbus.client.last_error_as_txt)
            print(self.modbus.client.last_except_as_full_txt)

            return False

    def __init__(self, run_preset=False):
        self.client = ModbusClient(host=SERVER_HOST, port=SERVER_PORT, auto_open=True, timeout=5)
        self.stall_occured = False
        self.last_slew = 0
        self.bus_semaphore = Lock()
        self.total_steps = 0
        self.step_overflow = 0

        self.writeActions = {
            "slew": self.WriteCommand(self, 0x0078, (-5000000, +5000000), 2),
            "holdCurrent": self.WriteCommand(self, 0x0029, (0, 100), 1),   # default 5
            "runCurrent": self.WriteCommand(self, 0x0067, (0, 100), 1),    # default 25
            "setTorque": self.WriteCommand(self, 0x00A6, (0, 100), 1),     # default 25
            "setMaxVelocity": self.WriteCommand(self, 0x008B, (+1, 2560000), 2),
            "error": self.WriteCommand(self, 0x0021, (0, 0), 1),
            "driveEnable": self.WriteCommand(self, 0x001C, (0, 1), 1),
            # default is 256 - 51200 steps / rev
            "microStep": self.WriteCommand(self, 0x0048, (1, 256), 1),
            "encodeEnable": self.WriteCommand(self, 0x001E, (0, 1), 1),
            "position": self.WriteCommand(self, 0x0057, (-2147483648, 2147483647), 2)
        }

        self.readAction = {
            "stalled": self.ReadCommand(self, 0x007B),
            "moving": self.ReadCommand(self, 0x004A),
            "outputFault": self.ReadCommand(self, 0x004E),
            "error": self.ReadCommand(self, 0x0021),
            # If hybrid circuitry is in make-up mode, 0x0085-86 will not return an accurate value.
            # When the hybrid product is in torque control mode 0x0085-86 will return a zero (0).
            "velocity": self.ReadCommand(self, 0x0085, 2),
            "position": self.ReadCommand(self, 0x0057, 2)
        }
        self.writeActions["encodeEnable"].set_value(1)
        self.writeActions['error'].set_value(0)
        self.writeActions['position'].set_value(0)

        self.polling_thread = Thread(target=self.polling_thread)
        # set daemon: polling thread will exit if main thread exit
        self.polling_thread.daemon = True
        self.polling_thread.start()

        if run_preset:
            self.__run_preset(self.__get_cfg())

    def set_slew_revs_minute(self, revs):
        value = round((revs / 60) * self.steps_per_rev)
        self.writeActions["slew"].set_value(value)
        self.last_slew = value

    @staticmethod
    def __get_cfg():
        try:
            with open('config.json', 'r') as config_file:
                return json.load(config_file)
        except FileNotFoundError:
            print("missing config file")
        except json.decoder.JSONDecodeError as err:
            print(f"Config error:\n{err} \ncannot open config")
        exit()

    def __run_preset(self, cfg):
        # wild guess we are working with non programmers or matlab "people" (such an evil word)
        start_index = cfg.get("startAt", 1) - 1
        intervals = cfg.get("timeRevIntervals")

        for interval in intervals[start_index:]:
            try:
                self.set_slew_revs_minute(interval[1])
                sleep(interval[0])
            except KeyboardInterrupt:
                return

    def polling_thread(self):

        while True:
            # stalled flag doesnt change
            stalled = False
            # stalled = self.readAction['stalled'].get_regs()
            # print(f"stalled: {stalled}")

            moving = self.readAction['moving'].get_regs()
            print(f"moving: {moving}")
            if stalled or (not moving and self.last_slew):
                self.stall_occured = True
                '''
                right here an error is thrown but without consequences as much as i can tell, delays don't fix this
                modbus exception
                Unrecoverable error occurred while slave was attempting to perform requested action.
                '''
                self.writeActions["slew"].set_value(self.last_slew)
            position = self.readAction['position'].get_regs()
            print(f"position: {position}")
            # an overflow hitting 32 uint limit is 41943,04 revolutions with the default resolution
            self.total_steps = self.step_overflow + position
            if abs(position) > 1 << 30:
                self.step_overflow += position
                self.writeActions['position'].set_value(0)
            print(f"total steps: {self.total_steps}")
            print(f"veclocity: {self.readAction['velocity'].get_regs()}")

            err = self.readAction['error'].get_regs()
            if err:
                print(f"error: {err}")
                self.writeActions['error'].set_value(0)
                output_fault = self.readAction['outputFault'].get_regs()
                if output_fault:
                    print(f"outputFault: {output_fault}")

            time.sleep(1.5)


def main():
    run_preset = False
    modbus_controller = ModbusController(run_preset)
    modbus_controller.readAction["error"].get_regs()

    if not run_preset:
        try:
            modbus_controller.writeActions["runCurrent"].set_value(10)
            modbus_controller.set_slew_revs_minute(20)
            sleep(20)
        except KeyboardInterrupt:
            modbus_controller.writeActions["slew"].set_value(0)

    modbus_controller.writeActions["slew"].set_value(0)


if __name__ == "__main__":
    main()
