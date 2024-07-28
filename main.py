
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
print(max_register_range)


def convertValueToRegister(value, value_range, register_count):

    abs_range = sum(abs(x) for x in value_range)
    # checking if its a single register write, if so we can already skip all conversion
    if abs_range < max_register_range:
        return [value]
    else:

        high_register = (value >> 16) & 0xFFFF
        low_register = value & 0xFFFF

        for register in range(register_count, 0 , -1):
            print(register)
            for i in range(register_size, 0, -1):
                print(i)
        return [low_register, high_register]


res = convertValueToRegister(500000, (-5000000, +5000000), 2)

c = ModbusClient(host=SERVER_HOST, port=SERVER_PORT, auto_open=True)

resid = 5000000 - 65535
# (1 << 16)-1

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

val = (1 << 16)

print(f"value is {res}")

print(c.write_multiple_registers(0x0078,  [0, 0]))
sleep(1)
print("testing")
print(c.write_multiple_registers(0x0078,  res))
print("success??")
sleep(3)
print(c.last_error_as_txt)
print(c.last_except_as_full_txt)
print(c.write_multiple_registers(0x0078,  [0, 0]))
exit()



def setHoldCurrent(percent):
    print()
    #c.write_single_register(0x0029, percent)


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