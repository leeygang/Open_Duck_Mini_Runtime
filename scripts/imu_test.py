from mini_bdx_runtime.imu import Imu
import time

imu = Imu(sampling_freq=30)
while True:
    orientation_quat = imu.get_data()

    print(orientation_quat)

    time.sleep(1 / 30)
