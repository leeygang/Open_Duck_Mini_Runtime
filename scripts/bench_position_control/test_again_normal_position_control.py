from pypot.feetech import FeetechSTS3215IO
import numpy as np
import time
import pickle


io = FeetechSTS3215IO("/dev/ttyACM0")
io.set_mode({1: 0})
ids = [1]


def convert_raw_load(raw_load):
    sign = (raw_load >> 10) & 1
    magnitude = raw_load & 0b1111111111
    if sign:
        return magnitude * 0.1 # to get percents
    else:
        return -magnitude * 0.1


# io.set_acceleration({1: 100})
# io.set_minimum_startup_force({1: 0})

io.set_D_coefficient({1: 0})

io.set_goal_position({1: 0})
time.sleep(1)
io.set_goal_position({1: 120})
positions = []
speeds = []
loads = []
start = time.time()
while True:
    if time.time() - start > 1:
        break
    # target = 30 * np.sin(2*np.pi*1*time.time())
    # io.set_goal_position({1:target})
    positions.append(io.get_present_position(ids))
    speeds.append(io.get_present_speed(ids))
    raw_load = io.get_present_load(ids)[0]
    loads.append(convert_raw_load(raw_load))
    # loads.append(io.get_present_load(ids))
    # io.get_present_load(ids)
    # print(io.get_present_position(ids))
    time.sleep(1 / 100)

data = {"positions": positions, "speeds": speeds, "loads": loads}

with open("normal_position_control.pkl", "wb") as f:
    pickle.dump(data, f)
