
from adafruit_extended_bus import ExtendedI2C as I2C
import adafruit_bno055
# import board
# import busio
import numpy as np

from queue import Queue
from threading import Thread
import time
from scipy.spatial.transform import Rotation as R



# TODO filter spikes
class Imu:
    def __init__(self, sampling_freq, user_pitch_bias=0, calibration=False, raw=False):
        self.sampling_freq = sampling_freq
        self.user_pitch_bias = user_pitch_bias
        self.nominal_pitch_bias = 20
        self.raw = raw

        # i2c = busio.I2C(board.SCL, board.SDA)
        i2c = I2C(1)  # Device is /dev/i2c-1
        self.imu = adafruit_bno055.BNO055_I2C(i2c)

        self.imu.mode = adafruit_bno055.IMUPLUS_MODE
        # self.imu.mode = adafruit_bno055.GYRONLY_MODE
        # self.imu.mode = adafruit_bno055.NDOF_MODE

        # self.compute_nominal_pitch_bias() # TODO

        self.pitch_bias = self.nominal_pitch_bias + self.user_pitch_bias

        if calibration:
            print("[IMU]: press ctrl+c when you are satisfied with the calibration")
            print("[IMU]: Calibrating...")
            try:
                while True:
                    print(
                        "[IMU]: ",
                        self.imu.calibration_status,
                        "calibrated : ",
                        self.imu.calibrated,
                    )
                    time.sleep(0.1)
            except KeyboardInterrupt:
                print("[IMU]: Calibration done.")
                exit()

        # # calibrate imu
        # calibrated = self.imu.calibration_status[1] == 3
        # print("Calibrating Gyro...")
        # while not calibrated:
        #     calibrated = self.imu.calibration_status[1] == 3
        #     time.sleep(0.01)

        # self.zero_euler = None
        # while self.zero_euler is None:
        #     try:
        #         zero_quat = np.array(self.imu.quaternion)
        #         self.zero_euler = R.from_quat(zero_quat).as_euler("xyz")
        #     except Exception as e:
        #         print(e)
        #         continue
        self.last_imu_data = [0, 0, 0, 0]
        self.imu_queue = Queue(maxsize=1)
        Thread(target=self.imu_worker, daemon=True).start()

    def convert_axes(self, euler):
        euler = [np.pi - euler[1], euler[2], -euler[0]]
        return euler

    def compute_nominal_pitch_bias(self):
        # the duck should not move during this process
        num_samples = 10
        pitch_samples = []
        # while np.std(pitch_samples) > 0.1 and len(pitch_samples) < num_samples:
        while True:
            try:
                raw_orientation = np.array(self.imu.quaternion)  # quat
                euler = R.from_quat(raw_orientation).as_euler("xyz")
                euler = self.convert_axes(euler)
                print("euler : ", np.rad2deg(euler[1]))
                pitch_samples.append(np.rad2deg(euler[1]))
                pitch_samples = pitch_samples[-num_samples:]
                print("std : ", np.std(pitch_samples))
                print("mean : ", np.mean(pitch_samples))
                print("==")
            except Exception as e:
                print("[IMU]:", e)
                continue
            time.sleep(1/self.sampling_freq)

        self.nominal_pitch_bias = np.mean(pitch_samples)
        print("[IMU]: Nominal pitch bias:", self.nominal_pitch_bias)

    def imu_worker(self):
        while True:
            s = time.time()
            try:
                # raw_orientation = np.array(self.imu.quaternion)  # quat
                # euler = R.from_quat(raw_orientation).as_euler("xyz")
                euler = np.array(self.imu.euler)
            except Exception as e:
                print("[IMU]:", e)
                continue

            # if self.raw:
            #     self.imu_queue.put(raw_orientation)
            # else:
            # Converting to correct axes
            euler = self.convert_axes(euler)
            # quat = R.from_euler("xyz", euler).as_quat()
            # euler = R.from_quat(quat).as_euler("xyz")
            euler[1] += np.deg2rad(self.pitch_bias)
            # ignoring yaw
            euler[2] = 0 

            final_orientation_quat = R.from_euler("xyz", euler).as_quat()

            self.imu_queue.put(final_orientation_quat)
            took = time.time() - s
            time.sleep(max(0, 1 / self.sampling_freq - took))

    def get_data(self, euler=False):
        try:
            self.last_imu_data = self.imu_queue.get(False)  # non blocking
        except Exception:
            pass

        if not euler:
            return self.last_imu_data
        else:
            return R.from_quat(self.last_imu_data).as_euler("xyz")
