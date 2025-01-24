from mini_bdx_runtime.imu import Imu
from mini_bdx_runtime.hwi_feetech_pwm_control import HWI
import time
import pickle

control_freq = 50

imu = Imu(control_freq)

hwi = HWI("/dev/ttyACM0")
hwi.turn_on()
kps = [0, 0, 0, 0, 0, 32, 32, 32, 32, 0, 0, 0, 0, 0]
hwi.set_kps(kps)

init_pos = hwi.init_pos
for i in range(10):
    zero_imu = imu.get_data(euler=True)
    time.sleep(0.1)

print("zero imu", zero_imu)
saved_euler = []

try:
    while True:
        s = time.time()

        euler = imu.get_data(euler=True)
        saved_euler.append(euler)

        pos = init_pos.copy()

        euler_diff = zero_imu - euler
        pos["head_roll"] = init_pos["head_roll"] - euler_diff[0]
        pos["head_pitch"] = init_pos["head_pitch"] - euler_diff[1]
        pos["head_yaw"] = init_pos["head_yaw"] + euler_diff[2]

        hwi.set_position_all(pos)

        took = time.time() - s
        time.sleep(max(1 / control_freq - took, 0))


except KeyboardInterrupt:
    hwi.freeze()
    time.sleep(1)
    pickle.dump(saved_euler, open("saved_euler.pkl", "wb"))
    time.sleep(1)
