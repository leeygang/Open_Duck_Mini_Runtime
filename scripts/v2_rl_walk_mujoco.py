import time
import pickle

# import pygame
import numpy as np

# from mini_bdx_runtime.hwi_feetech_pwm_control import HWI
# from mini_bdx_runtime.rustypot_control_hwi import HWI
from mini_bdx_runtime.rustypot_position_hwi import HWI
from mini_bdx_runtime.onnx_infer import OnnxInfer
from mini_bdx_runtime.rl_utils import (
    make_action_dict,
)
# from mini_bdx_runtime.imu import Imu
from mini_bdx_runtime.raw_imu import Imu
from mini_bdx_runtime.poly_reference_motion import PolyReferenceMotion
from mini_bdx_runtime.xbox_controller import XBoxController
from mini_bdx_runtime.feet_contacts import FeetContacts
from mini_bdx_runtime.eyes import Eyes

joints_order = [
    "left_hip_yaw",
    "left_hip_roll",
    "left_hip_pitch",
    "left_knee",
    "left_ankle",
    "neck_pitch",
    "head_pitch",
    "head_yaw",
    "head_roll",
    "left_antenna",
    "right_antenna",
    "right_hip_yaw",
    "right_hip_roll",
    "right_hip_pitch",
    "right_knee",
    "right_ankle",
]


class RLWalk:
    def __init__(
        self,
        onnx_model_path: str,
        serial_port: str = "/dev/ttyACM0",
        control_freq: float = 50,
        pid=[32, 0, 0],
        action_scale=0.25,
        commands=False,
        pitch_bias=0,
        replay_obs=None,
        standing=False,
    ):
        self.commands = commands
        self.pitch_bias = pitch_bias

        self.onnx_model_path = onnx_model_path
        self.policy = OnnxInfer(self.onnx_model_path, awd=True)

        self.num_dofs = 14

        # Control
        self.control_freq = control_freq
        self.pid = pid

        # self.saved_obs = []

        self.replay_obs = replay_obs
        if self.replay_obs is not None:
            self.replay_obs = pickle.load(open(self.replay_obs, "rb"))

        self.standing = standing

        self.hwi = HWI(serial_port)
        self.start()

        self.imu = Imu(
            sampling_freq=int(self.control_freq / 2), user_pitch_bias=self.pitch_bias
        )

        self.eyes = Eyes()

        self.feet_contacts = FeetContacts()

        # Scales
        self.action_scale = action_scale

        self.last_action = np.zeros(self.num_dofs)
        self.last_last_action = np.zeros(self.num_dofs)
        self.last_last_last_action = np.zeros(self.num_dofs)

        self.init_pos = [
            0.002,
            0.053,
            -0.63,
            1.368,
            -0.784,
            0,
            0,
            0,
            0,
            -0.003,
            -0.065,
            0.635,
            1.379,
            -0.796,
        ]

        self.last_commands = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

        self.paused = False

        self.command_freq = 10  # hz
        if self.commands:
            self.xbox_controller = XBoxController(self.command_freq, self.standing)

        if not self.standing:
            self.PRM = PolyReferenceMotion("./polynomial_coefficients.pkl")
            self.imitation_i = 0

    def add_fake_head(self, pos):
        # add just the antennas now
        assert len(pos) == self.num_dofs
        pos_with_head = np.insert(pos, 9, [0, 0])
        return np.array(pos_with_head)

    def get_obs(self):

        # imu_mat = self.imu.get_data(mat=True)
        imu_data = self.imu.get_data()
        # if imu_mat is None:
        #     print("IMU ERROR")
        #     return None

        dof_pos = self.hwi.get_present_positions(
            ignore=[
                # "neck_pitch",
                # "head_pitch",
                # "head_yaw",
                # "head_roll",
                "left_antenna",
                "right_antenna",
            ]
        )  # rad

        dof_vel = self.hwi.get_present_velocities(
            ignore=[
                # "neck_pitch",
                # "head_pitch",
                # "head_yaw",
                # "head_roll",
                "left_antenna",
                "right_antenna",
            ]
        )  # rad/s

        if len(dof_pos) != self.num_dofs:
            print(f"ERROR len(dof_pos) != {self.num_dofs}")
            return None

        if len(dof_vel) != self.num_dofs:
            print(f"ERROR len(dof_vel) != {self.num_dofs}")
            return None

        # projected_gravity = quat_rotate_inverse(orientation_quat, [0, 0, -1])
        # projected_gravity = np.array(imu_mat).reshape((3, 3)).T @ np.array([0, 0, -1])

        cmds = self.last_commands

        feet_contacts = self.feet_contacts.get()

        if not self.standing:
            ref = self.PRM.get_reference_motion(*cmds[:3], self.imitation_i)

        obs = np.concatenate(
            [
                imu_data["gyro"],
                imu_data["accelero"],
                # projected_gravity,
                cmds,
                dof_pos - self.init_pos,
                dof_vel * 0.05,
                self.last_action,
                self.last_last_action,
                self.last_last_last_action,
                feet_contacts,
                ref if not self.standing else np.array([]),
            ]
        )

        return obs

    def start(self):
        kps = [self.pid[0]] * 14
        kds = [self.pid[2]] * 14

        self.hwi.set_kps(kps)
        self.hwi.set_kds(kds)
        self.hwi.turn_on()

        time.sleep(2)

    def run(self):
        i = 0
        try:
            print("Starting")
            while True:
                t = time.time()

                if self.commands:
                    self.last_commands, A_pressed = (
                        self.xbox_controller.get_last_command()
                    )

                if A_pressed and not self.paused:
                    self.paused = True
                    print("PAUSE")
                elif A_pressed and self.paused:
                    self.paused = False
                    print("UNPAUSE")

                if self.paused:
                    time.sleep(0.1)
                    continue

                obs = self.get_obs()
                if obs is None:
                    continue

                if not self.standing:
                    self.imitation_i += 1
                    self.imitation_i = self.imitation_i % 450

                # self.saved_obs.append(obs)

                if self.replay_obs is not None:
                    if i < len(self.replay_obs):
                        obs = self.replay_obs[i]
                    else:
                        print("BREAKING ")
                        break

                # obs = np.clip(obs, -100, 100)

                action = self.policy.infer(obs)

                # action = np.clip(action, -1, 1)

                self.last_last_last_action = self.last_last_action.copy()
                self.last_last_action = self.last_action.copy()
                # self.last_action_action = action.copy() # WTF MAN
                self.last_action = action.copy()

                # action = np.zeros(10)

                robot_action = self.init_pos + action * self.action_scale

                robot_action = self.add_fake_head(robot_action)

                action_dict = make_action_dict(
                    robot_action, joints_order
                )  # Removes antennas

                self.hwi.set_position_all(action_dict)

                i += 1

                took = time.time() - t
                # print("Full loop took", took, "fps : ", np.around(1 / took, 2))
                if (1 / self.control_freq - took) < 0:
                    print(
                        "Policy control budget exceeded by",
                        np.around(took - 1 / self.control_freq, 3),
                    )
                time.sleep(max(0, 1 / self.control_freq - took))

        except KeyboardInterrupt:
            # self.hwi.freeze()
            pass

        # pickle.dump(self.saved_obs, open("robot_saved_obs.pkl", "wb"))
        # print("FREEZING")
        # self.hwi.freeze()
        print("TURNING OFF")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx_model_path", type=str, required=True)
    parser.add_argument("-a", "--action_scale", type=float, default=0.25)
    parser.add_argument("-p", type=int, default=32)
    parser.add_argument("-i", type=int, default=0)
    parser.add_argument("-d", type=int, default=0)
    parser.add_argument("-c", "--control_freq", type=int, default=50)
    parser.add_argument("--pitch_bias", type=float, default=0, help="deg")
    parser.add_argument(
        "--commands",
        action="store_true",
        default=False,
        help="external commands, keyboard or gamepad. Launch control_server.py on host computer",
    )
    parser.add_argument("--replay_obs", type=str, required=False, default=None)
    parser.add_argument("--standing", action="store_true", default=False)
    args = parser.parse_args()
    pid = [args.p, args.i, args.d]

    print("Done parsing args")
    rl_walk = RLWalk(
        args.onnx_model_path,
        action_scale=args.action_scale,
        pid=pid,
        control_freq=args.control_freq,
        commands=args.commands,
        pitch_bias=args.pitch_bias,
        replay_obs=args.replay_obs,
        standing=args.standing,
    )
    print("Done instantiating RLWalk")
    # rl_walk.start()
    rl_walk.run()
