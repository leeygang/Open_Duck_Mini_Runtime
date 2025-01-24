from mini_bdx_runtime.imu import Imu
from mini_bdx_runtime.hwi_feetech_pwm_control import HWI
import time

control_freq = 50

imu = Imu(control_freq)

hwi = HWI("/dev/ttyACM0")
hwi.turn_on()
hwi.set_kps([4 * 14])

init_pos = hwi.init_pos
for i in range(100):
    zero_imu = imu.get_data(euler=True)

print(zero_imu)

try:
    while True:
        s = time.time()

        euler = imu.get_data(euler=True)
        print("euler : ", euler)
        # print("yaw : ", imu.get_data(euler=True)[2])

        euler_diff = zero_imu[2] - euler[2]
        head_yaw_target = init_pos["head_yaw"] + euler_diff
        print("euler diff : ", euler_diff)
        print(head_yaw_target)
        pos = init_pos.copy()
        # pos["head_yaw"] = head_yaw_target

        hwi.set_position_all(pos)

        took = time.time() - s
        time.sleep(max(1 / control_freq - took, 0))


except KeyboardInterrupt:
    hwi.freeze()
    time.sleep(1)
