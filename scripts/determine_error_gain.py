from pypot.feetech import FeetechSTS3215IO
import numpy as np
import time


def convert_raw_load(raw_load):
    pwm_magnitude = raw_load & 0x3FF  # Extract lower 10 bits (0-1000)
    direction = (raw_load >> 10) & 1  # Extract 11th bit (sign)

    percentage_load = (pwm_magnitude / 1000) * 100  # Convert to percentage
    return percentage_load * 0.01 if direction == 1 else -percentage_load * 0.01


io = FeetechSTS3215IO("/dev/ttyACM0")
io.set_mode({1: 0})
ids = [1]

KP = 32
io.set_P_coefficient({1: KP})
io.set_D_coefficient({1: 0})
target_error = np.deg2rad(2)
present_position = np.deg2rad(io.get_present_position(ids)[0])

target_position = present_position + target_error

io.set_goal_position({1: np.rad2deg(target_position)})


time.sleep(1)
while True:
    present_position = np.deg2rad(io.get_present_position(ids)[0])

    error = target_position - present_position

    load = convert_raw_load(io.get_present_load(ids)[0])
    volts = io.get_present_voltage([1])[0] * 0.1
    # load = error_gain * KP * error
    error_gain = load / (KP * error)
    print("volts : ", volts)
    print("error (rad): ", error, "error (deg) ", np.rad2deg(error))
    print("load : ", load)
    print("error_gain : ", error_gain)
    print("==")
    time.sleep(0.05)



# 0
# 0
# 0.193
# 0.193
# 0.193
# 0.252
# 0.252
# 0.252
# 0.252
# 0.0311
# 0.311
# 0.311
# 0.430
# 0.725
# 1.44
# 2.79
# 4.09
# 5.45
# 6.81
# 7.18