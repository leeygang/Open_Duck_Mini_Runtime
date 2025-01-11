from pypot.feetech import FeetechSTS3215IO
import time


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


io = FeetechSTS3215IO(
    "/dev/ttyACM0",
    baudrate=1000000,
    use_sync_read=True,
)

for joint_name, joint_id in joints.items():
    print(f"Setting up {joint_name}")
    io.set_lock({joint_id: 1})

    io.enable_torque([joint_id])
    io.set_offset({joint_id: 0})

    print("Going to zero. Press enter to continue.")
    input()

    io.set_goal_position({joint_id: 0})
    time.sleep(1)
    zero_pos = io.get_present_position([joint_id])[0]
    io.disable_torque([joint_id])

    print("Move the motor to the real zero. Press enter to continue.")
    input()

    new_zero_pos = io.get_present_position([joint_id])[0]
    offset = zero_pos - new_zero_pos
    print("Offset: ", offset)
    io.set_offset({joint_id: offset})
    time.sleep(1)

    io.set_lock({joint_id: 0})
