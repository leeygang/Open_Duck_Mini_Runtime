import time
import pickle
from queue import Queue
from threading import Thread
import pygame
import numpy as np
import RPi.GPIO as GPIO

# from mini_bdx_runtime.hwi_feetech_pwm_control import HWI
# from mini_bdx_runtime.rustypot_control_hwi import HWI
from mini_bdx_runtime.rustypot_position_hwi import HWI
from mini_bdx_runtime.onnx_infer import OnnxInfer
from mini_bdx_runtime.rl_utils import (
    make_action_dict,
)
from mini_bdx_runtime.imu import Imu
from mini_bdx_runtime.poly_reference_motion import PolyReferenceMotion


# Commands
X_RANGE = [-0.1, 0.15]
Y_RANGE = [-0.2, 0.2]
YAW_RANGE = [-0.5, 0.5]

LEFT_FOOT_PIN = 22
RIGHT_FOOT_PIN = 27
GPIO.setwarnings(False)  # Ignore warning for now
GPIO.setmode(GPIO.BCM)  # Use physical pin numbering
GPIO.setup(LEFT_FOOT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)
GPIO.setup(RIGHT_FOOT_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)


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
    ):
        self.commands = commands
        self.pitch_bias = pitch_bias

        self.onnx_model_path = onnx_model_path
        self.policy = OnnxInfer(self.onnx_model_path, awd=True)

        self.num_dofs = 10

        # Control
        self.control_freq = control_freq
        self.pid = pid

        # self.saved_obs = []

        self.replay_obs = replay_obs
        if self.replay_obs is not None:
            self.replay_obs = pickle.load(open(self.replay_obs, "rb"))

        self.hwi = HWI(serial_port)
        self.start()

        self.imu = Imu(sampling_freq=int(self.control_freq/2), user_pitch_bias=self.pitch_bias)

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
            -0.003,
            -0.065,
            0.635,
            1.379,
            -0.796,
        ]

        self.last_commands = [0.0, 0, 0]

        self.command_freq = 10  # hz
        if self.commands:
            pygame.init()
            self._p1 = pygame.joystick.Joystick(0)
            self._p1.init()
            print(f"Loaded joystick with {self._p1.get_numaxes()} axes.")
            self.cmd_queue = Queue(maxsize=1)
            Thread(target=self.commands_worker, daemon=True).start()

        self.last_command_time = time.time()

        self.PRM = PolyReferenceMotion("./polynomial_coefficients.pkl")
        self.imitation_i = 0

    def add_fake_head(self, pos):
        assert len(pos) == self.num_dofs
        pos_with_head = np.insert(pos, 5, [0, 0, 0, 0, 0, 0])
        return np.array(pos_with_head)

    def commands_worker(self):
        while True:
            self.cmd_queue.put(self.get_commands())
            time.sleep(1 / self.command_freq)

    def get_commands(self):
        last_commands = self.last_commands
        for event in pygame.event.get():
            lin_vel_y = -1 * self._p1.get_axis(0)
            lin_vel_x = -1 * self._p1.get_axis(1)
            ang_vel = -1 * self._p1.get_axis(2)
            if lin_vel_x >= 0:
                lin_vel_x *= np.abs(X_RANGE[1])
            else:
                lin_vel_x *= np.abs(X_RANGE[0])

            if lin_vel_y >= 0:
                lin_vel_y *= np.abs(Y_RANGE[1])
            else:
                lin_vel_y *= np.abs(Y_RANGE[0])

            if ang_vel >= 0:
                ang_vel *= np.abs(YAW_RANGE[1])
            else:
                ang_vel *= np.abs(YAW_RANGE[0])

            last_commands[0] = lin_vel_x
            last_commands[1] = lin_vel_y
            last_commands[2] = ang_vel

        pygame.event.pump()  # process event queue

        return np.around(last_commands, 3)

    def get_last_command(self):
        try:
            self.last_commands = self.cmd_queue.get(False)  # non blocking
        except Exception:
            pass

        return self.last_commands

    def get_feet_contacts(self):
        left = False
        right = False
        if GPIO.input(LEFT_FOOT_PIN) == GPIO.LOW:
            left = True
        if GPIO.input(RIGHT_FOOT_PIN) == GPIO.LOW:
            right = True
        return np.array([left, right])

    def get_obs(self):

        imu_mat = self.imu.get_data(mat=True)
        if imu_mat is None:
            print("IMU ERROR")
            return None

        if self.commands:
            self.last_commands = self.get_last_command()

        dof_pos = self.hwi.get_present_positions(
            ignore=[
                "neck_pitch",
                "head_pitch",
                "head_yaw",
                "head_roll",
                "left_antenna",
                "right_antenna",
            ]
        )  # rad

        dof_vel = self.hwi.get_present_velocities(
            ignore=[
                "neck_pitch",
                "head_pitch",
                "head_yaw",
                "head_roll",
                "left_antenna",
                "right_antenna",
            ]
        )  # rad/s

        if len(dof_pos) != self.num_dofs:
            print(f"ERROR len(dof_pos) != {self.num_dofs}")
            return None

        if len(dof_vel) != 10:
            print(f"ERROR len(dof_vel) != {self.num_dofs}")
            return None

        # projected_gravity = quat_rotate_inverse(orientation_quat, [0, 0, -1])
        projected_gravity = np.array(imu_mat).reshape((3, 3)).T @ np.array([0, 0, -1])

        cmds = self.last_commands

        feet_contacts = self.get_feet_contacts()

        ref = self.PRM.get_reference_motion(*cmds, self.imitation_i)

        obs = np.concatenate(
            [
                projected_gravity,
                cmds,
                dof_pos - self.init_pos,
                dof_vel * 0.05,
                self.last_action,
                self.last_last_action,
                self.last_last_last_action,
                feet_contacts,
                ref,
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
                
                obs = self.get_obs()
                if obs is None:
                    continue

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
    )
    print("Done instantiating RLWalk")
    # rl_walk.start()
    rl_walk.run()
