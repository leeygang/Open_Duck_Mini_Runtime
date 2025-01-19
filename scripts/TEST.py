from mini_bdx_runtime.hwi_feetech_pypot import HWI
import numpy as np

hwi = HWI()
hwi.turn_off()


# zero_pos = hwi.zero_pos.copy()
# zero_pos["right_ankle"] = 0.2
# hwi.set_position_all(zero_pos)
# # hwi.set_position("left_hip_yaw", 0.2)
