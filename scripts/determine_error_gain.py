from pypot.feetech import FeetechSTS3215IO
import numpy as np
import time
# import pickle


io = FeetechSTS3215IO("/dev/ttyACM0")
io.set_mode({1: 0})
ids = [1]

KP = 32
target_error = np.deg2rad(1)
present_position = np.deg2rad(io.get_present_position(ids)[0])


target_position = present_position + target_error
io.set_P_coefficient({1: KP})
# print(np.deg2rad(io.get_present_position(ids)[0]))
io.set_goal_position({1: np.rad2deg(target_position)})
def convert_raw_load(raw_load):
    sign = (raw_load >> 10) & 1
    magnitude = raw_load & 0b1111111111
    if sign:
        return magnitude * 0.1 # to get percents
    else:
        return -magnitude * 0.1

time.sleep(1)
while True:
    # io.get_present_position(ids)[0]
    present_position = np.deg2rad(io.get_present_position(ids)[0])

    error = target_position - present_position
    raw_load = io.get_present_load(ids)[0]
    # print(raw_load)
    load = convert_raw_load(raw_load) * 0.01
    # print(load)
    # load = error_gain * KP * error
    error_gain = load / (KP * error)
    print("error (rad): ", error, "error (deg) ", np.rad2deg(error))
    print("load : ", load)
    print("error_gain : ", error_gain)
    print("==")
    time.sleep(0.01)