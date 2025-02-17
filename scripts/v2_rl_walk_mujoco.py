import time
import pickle
from queue import Queue
from threading import Thread
import pygame
import numpy as np
import RPi.GPIO as GPIO

from mini_bdx_runtime.hwi_feetech_pwm_control import HWI
from mini_bdx_runtime.onnx_infer import OnnxInfer
from mini_bdx_runtime.rl_utils import (
    make_action_dict,
)
from mini_bdx_runtime.imu import Imu


# Commands
X_RANGE = [-0.1, 0.1]
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
        action_scale=0.5,
        commands=False,
        pitch_bias=0,
        history_len=0,
    ):
        self.commands = commands
        self.pitch_bias = pitch_bias

        self.onnx_model_path = onnx_model_path
        self.policy = OnnxInfer(self.onnx_model_path, awd=True)

        self.num_dofs = 10

        # Control
        self.control_freq = control_freq
        self.pid = pid

        self.saved_obs = []

        self.current_phase = np.array([0, np.pi])
        self.gait_freq = 1.5
        self.phase_dt = 2 * np.pi * (1 / self.control_freq) * self.gait_freq

        self.hwi = HWI(serial_port)
        self.start()

        self.imu = Imu(sampling_freq=self.control_freq, user_pitch_bias=self.pitch_bias)

        # Scales
        self.action_scale = action_scale

        self.prev_action = np.zeros(self.num_dofs)

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

        self.history_len = history_len
        self.qpos_error_history = np.zeros(self.history_len * self.num_dofs)
        self.qvel_history = np.zeros(self.history_len * self.num_dofs)
        self.gravity_history = np.zeros(self.history_len * 3)

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

    def get_phase(self):
        phase_tp1 = self.current_phase + self.phase_dt
        self.current_phase = np.fmod(phase_tp1 + np.pi, 2 * np.pi) - np.pi
        cos = np.cos(self.current_phase - np.pi/2)
        # sin = np.sin(self.current_phase + np.pi)
        return cos
        # return np.concatenate([cos, sin])

    def get_obs(self):
        imu_mat = self.imu.get_data(mat=True)
        if imu_mat is None:
            print("IMU ERROR")
            return None

        if self.commands:
            self.last_commands = self.get_last_command()
            # print(self.last_commands)

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

        phase = self.get_phase()

        cmds = self.last_commands

        feet_contacts = self.get_feet_contacts()

        if self.history_len > 0:
            self.qvel_history = np.roll(self.qvel_history, self.num_dofs)
            self.qvel_history[: self.num_dofs] = dof_vel

            last_motor_target = self.init_pos + self.prev_action * self.action_scale
            qpos_error = dof_pos - last_motor_target
            self.qpos_error_history = np.roll(self.qpos_error_history, self.num_dofs)
            self.qpos_error_history[: self.num_dofs] = qpos_error

            self.gravity_history = np.roll(self.gravity_history, 3)
            self.gravity_history[:3] = projected_gravity

        obs = np.concatenate(
            [
                projected_gravity,
                cmds,
                dof_pos - self.init_pos,
                dof_vel,
                self.prev_action,
                phase,
                feet_contacts,
                self.qpos_error_history,  # is [] if history_len == 0
                self.qvel_history,  # is [] if history_len == 0
                self.gravity_history,  # is [] if history_len == 0
            ]
        )

        return obs

    def start(self):
        self.hwi.turn_on()
        kps = [self.pid[0]] * 14

        self.hwi.set_kps(kps)

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

                self.saved_obs.append(obs)

                obs = np.clip(obs, -100, 100)

                action = self.policy.infer(obs)

                action = np.clip(action, -1, 1)

                self.prev_action = action.copy()

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
            self.hwi.freeze()
            pass

        pickle.dump(self.saved_obs, open("robot_saved_obs.pkl", "wb"))
        print("FREEZING")
        self.hwi.freeze()
        print("TURNING OFF")


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx_model_path", type=str, required=True)
    parser.add_argument("-a", "--action_scale", type=float, default=0.5)
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
    )
    print("Done instantiating RLWalk")
    # rl_walk.start()
    rl_walk.run()
