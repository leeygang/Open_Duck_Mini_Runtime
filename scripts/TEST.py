# from mini_bdx_runtime.hwi_feetech_pypot import HWI
from mini_bdx_runtime.hwi_feetech_pwm_control import HWI
import numpy as np
import time

hwi = HWI()
hwi.turn_on()
while True:
    time.sleep(0.01)
# exit()

# zero_pos = hwi.zero_pos.copy()
# id = 4
# name = list(hwi.joints.keys())[id]
# while True:
#     # target = 0.4
#     target = np.around(0.2*np.sin(2*np.pi*0.1*time.time()), 3)
#     zero_pos[name] = target
#     hwi.set_position_all(zero_pos)
#     present_positions = hwi.get_present_positions()
#     present_velocities = hwi.get_present_velocities()
#     # print(f"target : {target}, pos : {present_positions[id]}, diff : {target - present_positions[id]}")
#     print(f"vel : {present_velocities[id]}")
#     # # hwi.set_position("left_hip_yaw", 0.2)
