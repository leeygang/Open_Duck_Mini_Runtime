from mini_bdx_runtime.feetech_pwm_control import FeetechPWMControl
import time
import pickle

control = FeetechPWMControl([1], [0], usb_port="/dev/ttyACM1")

positions = []
speeds = []
loads = []

def convert_raw_load(raw_load):
    sign = (raw_load >> 10) & 1
    magnitude = raw_load & 0b1111111111
    if sign:
        return magnitude * 0.1 # to get percents
    else:
        return -magnitude * 0.1


control.goal_positions = [0]
time.sleep(1)
control.goal_positions = [120]
start = time.time()
while True:
    if time.time() - start > 1:
        break

    positions.append(control.present_positions[0])
    speeds.append(control.present_speeds[0])
    raw_load = control.io.get_present_load([1])[0]
    loads.append(convert_raw_load(raw_load))

    time.sleep(1/100)




data = {"positions": positions, "speeds": speeds, "loads": loads}

with open("pwm_control.pkl", "wb") as f:
    pickle.dump(data, f)