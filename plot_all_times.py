import pickle
import numpy as np

all_times = pickle.load(open("all_times_ischedule.pkl", "rb"))

diff = np.diff(all_times)

print("mean", np.mean(diff))
print("std", np.std(diff))
print("max", np.max(diff))
print("min", np.min(diff))


import matplotlib.pyplot as plt

plt.plot(diff)
plt.show()
