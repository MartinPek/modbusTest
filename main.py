
import time
from threading import Thread, Lock
from pyModbusTCP.client import ModbusClient
from time import sleep

# https://pymodbustcp.readthedocs.io/en/stable/examples/client_thread.html

SERVER_HOST = '192.168.59.35'
SERVER_PORT = 502

# set global
regs = []

# init a thread lock
regs_lock = Lock()

'''
@ TODO: 🔲 ✅

🔲 Timeouts is very long also doesnt crop up when setting up the device when unplugged

🔲 Error 0x0021 2 Variable holds the error code of the last error. 
 must be read or set to 0 to clear.
 — 0

🔲 in case calculating is troublesome  Microstep resolution 0x0048

'''

#   Holding current 0x0029 1 Sets the motor holding current in percent (%) 0 to 100
# Run current 0x0067 1 Sets the motor run current in percent (%). 1 to 100 25

#   Slew axis 0x0078 -
# 0x0079
#  4 Slews the axis at velocity in steps/second
# in the specified ± direction, Slew velocity is
# independent of 0x008B (maximum velocity).
#  ±5000000

# Stall flag 0x007B 1 indicates a motor stall (1) or not stalled (0)
# (Closed loop only).
#  0/1 0

#  Set torque 0x00A6 1 Sets the motor torque in percent for torque mode
# operation.
#  1 – 100 25

#  Slew axis 0x0078 - 0x0079
#  4 Slews the axis at velocity in steps/second
# in the specified ± direction, Slew velocity is
# independent of 0x008B (maximum velocity).
#  ±5000000

register_size = 16
max_register_range = 1 << register_size    # amount of value, the max value is one less since 0 is also a number
c = ModbusClient(host=SERVER_HOST, port=SERVER_PORT, auto_open=True)


def convert_value_to_register(value, value_range, register_count):
    clipped_value = max(min(value, value_range[1]), value_range[0])
    if clipped_value != value:
        print(f"Value: {value} was out of range {value_range}. Clipped to {clipped_value}")
        value = clipped_value
    abs_range = sum(abs(x) for x in value_range)
    # checking if it's a single register write, if so we can already skip all conversion
    if abs_range < max_register_range:
        return [value]
    else:
        high_register = (value >> 16) & 0xFFFF
        low_register = value & 0xFFFF
        return [low_register, high_register]


class WriteCommand:
    def __init__(self, register, value_range, register_count):
        self.register = register
        self.value_range = value_range
        self.register_count = register_count

    def set_value(self, value):
        register_value = convert_value_to_register(value, self.value_range, self.register_count)
        res = c.write_multiple_registers(self.register, register_value)
        if not (c.last_error and c.last_except):
            return True

        # does this reset automatically with the next command? no manual function for that listed afaik
        print(c.last_error_as_txt)
        print(c.last_except_as_full_txt)

        return False


class ReadCommand:
    def __init__(self, register, register_count=1):
        self.register = register
        self.register_count = register_count

    def get_regs(self):
        regs_l = c.read_holding_registers(self.register, self.register_count )


writeActions = {
    "slew": WriteCommand(0x0078, (-5000000, +5000000), 2),
    "holdCurrent": WriteCommand(0x0029, (0, 100), 1),   # default 5
    "runCurrent": WriteCommand(0x0067, (0, 100), 1),    # default 25
    "setTorque": WriteCommand(0x00A6, (0, 100), 1),     # default 25
    "setMaxVelocity": WriteCommand(0x008B, (+1, 2560000), 2),
    "error": WriteCommand(0x0021, (0,0) ,1)
}

readAction = {
    "stalled": ReadCommand(0x007B),
    "moving": ReadCommand(0x004A),
    "outputFault": ReadCommand(0x004E),
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

print("testing")

print(c.write_multiple_registers(0x0078,  [0, 0]))
sleep(1)
writeActions["runCurrent"].set_value(100)
writeActions["setTorque"].set_value(100)
writeActions["slew"].set_value(-500000)

# writeActions["error"].set_value(0)
print("success??")
sleep(3)
print(c.last_error)
print(c.last_except)
print(c.last_error_as_txt)
print(c.last_except_as_full_txt)
print(c.write_multiple_registers(0x0078,  [0, 0]))
exit()


def setHoldCurrent(percent):
    print()
    # c.write_single_register(0x0029, percent)


def polling_thread():
    """Modbus polling thread."""
    global regs, regs_lock
    c = ModbusClient(host=SERVER_HOST, port=SERVER_PORT, auto_open=True)
    # polling loop
    print(c.write_multiple_registers(0x0078, [1, 0]))

    return False

    while True:
        # do modbus reading on socket
        print(c.write_multiple_registers(0x0078, [1, 0]))
        reg_list = c.read_holding_registers(5, 5)
        # if read is ok, store result in regs (with thread lock)
        if reg_list:
            with regs_lock:
                regs = list(reg_list)
        # 1s before next polling
        time.sleep(1)


# start polling thread
tp = Thread(target=polling_thread)
# set daemon: polling thread will exit if main thread exit
tp.daemon = True
tp.start()

# display loop (in main thread)
while True:
    # print regs list (with thread lock synchronization)
    with regs_lock:
        print(regs)
    # 1s before next print
    time.sleep(1)