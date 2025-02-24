import rustypot
import time

io = rustypot.feetech("/dev/ttyACM0", 1000000)

io.write_goal_position([30, 31, 32, 33], [0, 0, 0, 0])
time.sleep(1)
# while True:
#     print(io.read_present_position([30, 31, 32, 33]))
    # time.sleep(0.01)
