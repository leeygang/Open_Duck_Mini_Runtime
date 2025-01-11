from pypot.feetech import FeetechSTS3215IO
import time
import sys


io = FeetechSTS3215IO(
    "/dev/ttyACM0",
    baudrate=1000000,
    use_sync_read=True,
)


joints = {
    "right_hip_yaw": 10,
    "right_hip_roll": 11,
    "right_hip_pitch": 12,
    "right_knee": 13,
    "right_ankle": 14,
    "left_hip_yaw": 20,
    "left_hip_roll": 21,
    "left_hip_pitch": 22,
    "left_knee": 23,
    "left_ankle": 24,
    "neck_pitch": 30,
    "head_pitch": 31,
    "head_yaw": 32,
    "head_roll": 33,
}

id = int(sys.argv[1])
print("id : ",  id)

# for name, id in joints.items():
#     print(name)
#     print("present position before: ", io.get_present_position([id])[0])
#     io.set_offset({id:0})
#     time.sleep(1)
#     print("present position after: ", io.get_present_position([id])[0])
#     print("==")
# exit()

# print("offset : ", io.get_offset([id]))
try:
    while True:
        print("present position : ", io.get_present_position([id]))
        time.sleep(0.1)
except KeyboardInterrupt:
    pass

exit()
io.disable_torque([id])
input("press enter to go to zero")
io.enable_torque([id])
time.sleep(1)
io.set_goal_position({id:0})
time.sleep(1)


# io.set_lock({id: 0})
# time.sleep(1)

# io.set_offset({id:0})
# time.sleep(1)
