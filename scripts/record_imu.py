import time
from scipy.spatial.transform import Rotation as R
import pickle
from mini_bdx_runtime.imu import Imu
import argparse

parser = argparse.ArgumentParser()
parser.add_argument("--pitch_bias", type=float, default=0)
args = parser.parse_args()

imu = Imu(sampling_freq=100, user_pitch_bias=args.pitch_bias)
data = []
data_euler = []
print("Starting to record. Press Ctrl+C to stop.")
try:
    while True:
        try:
            raw_orientation = imu.get_data(euler=False)
            data.append(raw_orientation)
            euler = R.from_quat(raw_orientation).as_euler("xyz")
            data_euler.append(euler)
        except Exception as e:
            print(e)
            continue

        time.sleep(1 / 30)
except KeyboardInterrupt:
    print("Saving data...")
    with open("imu_data.pkl", "wb") as f:
        pickle.dump(data, f)
    with open("imu_data_euler.pkl", "wb") as f:
        pickle.dump(data_euler, f)
    print("Data saved.")
    exit(0)
