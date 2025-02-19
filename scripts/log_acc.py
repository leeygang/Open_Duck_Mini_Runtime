from mini_bdx_runtime.feetech_pwm_control import FeetechPWMControl
import numpy as np
import time

control = FeetechPWMControl(
    ids=[1],
    init_pos_rad=[0],
    usb_port="/dev/ttyACM0"
)
control.kps = [32]

control.goal_positions = [0]
time.sleep(1)
control.goal_positions = [150]
positions = []
speeds = []
s = time.time()
dt = 0.01
while True:
    positions.append(control.present_positions[0])
    speed_deg_s = control.present_speeds[0]
    speed_rad_s = np.deg2rad(speed_deg_s)
    speeds.append(speed_rad_s)
    time.sleep(0.01)
    if time.time() - s > 0.25:
        break


ts = np.arange(0, len(positions)*dt, dt)
import matplotlib.pyplot as plt

plt.plot(ts, speeds)
# plt.legend(["speed (rad/s)"])
plt.xlabel("time (s)")
plt.ylabel("speed (rad/s)")
# ylim 6
plt.ylim(-0.1, 6)
# show grid
plt.grid()
plt.show()


