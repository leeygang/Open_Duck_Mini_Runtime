import pickle

imu_data = pickle.load(open("imu_data.pkl", "rb"))

gyros = imu_data["gyros"]
acceleros = imu_data["accels"]

import matplotlib.pyplot as plt

plt.plot(gyros)
plt.show()

plt.plot(acceleros)
plt.show()