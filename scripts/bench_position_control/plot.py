import pickle
import argparse

parser = argparse.ArgumentParser()
parser.add_argument('-f', type=str)
args = parser.parse_args()


data = pickle.load(open(args.f, 'rb'))

# data = {
#     'positions': positions,
#     'speeds': speeds,
#     'loads': loads
# }

import matplotlib.pyplot as plt

# 3 figures in the same window

plt.figure(1)
plt.subplot(311)
plt.plot(data['positions'])
plt.title('positions')
plt.xlim(-5, 100)
plt.ylim(-10, 130)
plt.subplot(312)
plt.plot(data['speeds'])
plt.title('speeds')
plt.xlim(-5, 100)
plt.ylim(-100, 400)
plt.subplot(313)
plt.plot(data['loads'])
plt.title('loads')
plt.xlim(-5, 100)
plt.ylim(-100, 150)

plt.show()