import pickle

euler = pickle.load(open("saved_euler.pkl", "rb"))  # list of euler angles over time

import matplotlib.pyplot as plt
import numpy as np

euler = np.array(euler)

plt.plot(euler[:, 0], label="roll")
plt.plot(euler[:, 1], label="pitch")
plt.plot(euler[:, 2], label="yaw")
plt.legend()
plt.show()