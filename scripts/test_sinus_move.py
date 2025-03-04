import rustypot
import time
import numpy as np

io = rustypot.feetech("/dev/ttyACM0", 1000000)

ids = [24]
io.set_kps(ids, [16])
io.set_kds(ids, [0])

io.write_goal_position(ids, [0])

control_freq = 50  # Hz
A = 0.2
F = 0.5
while True:
    target = 1 * np.sin(2 * np.pi * F * time.time())

    io.write_goal_position(ids, [target])
    present_pos = io.read_present_position(ids)[0]
    present_velocity = io.read_present_velocity(ids)[0]

    print("present pos : ", present_pos)
    print("present velocity : ", present_velocity)
    print("==")

    time.sleep(1 / control_freq)
