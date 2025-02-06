from pypot.feetech import FeetechSTS3215IO
import time
import numpy as np

io = FeetechSTS3215IO(
    "/dev/ttyACM0",
    baudrate=1000000,
    # use_sync_read=False,
    use_sync_read=True,
)

ids = [1]
# ids = [10, 11, 12, 13, 14, 20, 21, 22, 23, 24, 30, 31, 32, 33]

# io.enable_torque(ids)
# io.set_mode({id: 0 for id in ids})
times = []
stt = time.time()
while True:
    s = time.time()
    io.get_present_position(ids)
    # io.set_goal_position({id: 0 for id in ids})
    took = time.time() - s
    times.append(took)
    print(took)

    time.sleep(1 / 100)
    if time.time() - stt > 5:
        break

print("avg :", np.mean(times))
