from pypot.feetech import FeetechSTS3215IO
import pickle
import numpy as np
import time

io = FeetechSTS3215IO("/dev/ttyACM0")

acceleration = 50
kp = 50
kd = 0

io.set_lock({1: 1})
io.set_maximum_acceleration({1: acceleration})
io.set_P_coefficient({1: kp})
io.set_D_coefficient({1: kd})

time.sleep(0.1)
goal_position = 90

io.set_goal_position({1: 0})
time.sleep(3)


times = []
positions = []
goal_positions = []
speeds = []
loads = []
currents = []


def convert_load(raw_load):
    if raw_load > 1023:
        raw_load -= 1024
    return raw_load * 0.001


io.set_goal_position({1: goal_position})
s = time.time()
while True:
    present_position = np.deg2rad(io.get_present_position([1])[0])
    present_speed = np.deg2rad(io.get_present_speed([1])[0])
    present_load = convert_load(io.get_present_load([1])[0])
    present_current = io.get_present_current([1])[0]

    times.append(time.time())
    positions.append(present_position)
    goal_positions.append(np.deg2rad(goal_position))
    speeds.append(present_speed)
    loads.append(present_load)
    currents.append(present_current)

    if time.time() - s > 3:
        break

    time.sleep(0.01)

data = {
    "acceleration": acceleration,
    "kp": kp,
    "kd": kd,
    "times": times,
    "positions": positions,
    "goal_positions": goal_positions,
    "speeds": speeds,
    "loads": loads,
    "currents": currents,
}


pickle.dump(data, open(f"data_acceleration_{acceleration}_kp_{kp}_kd_{kd}.pkl", "wb"))
