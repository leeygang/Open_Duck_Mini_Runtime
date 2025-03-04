import rustypot
import time
import numpy as np

io = rustypot.feetech("/dev/ttyACM0", 1000000)

ids = [1]
io.set_kps(ids, [16])
io.set_kds(ids, [0])
io.set_mode(ids, 0)

io.write_goal_position(ids, [0])
io.enable_torque(ids)

raw_velocities = []

control_freq = 50  # Hz
A = 1.0
F = 1.0
try:
    while True:
        target = A * np.sin(2 * np.pi * F * time.time())

        io.write_goal_position(ids, [target])
        present_pos = io.read_present_position(ids)[0]
        present_velocity = io.read_present_velocity(ids)[0]
        raw_velocities.append(present_velocity)

        print("present pos : ", present_pos)
        print("present velocity : ", present_velocity)
        print("==")

        time.sleep(1 / control_freq)
except KeyboardInterrupt:
    pass

import matplotlib.pyplot as plt

plt.plot(raw_velocities)
plt.show()