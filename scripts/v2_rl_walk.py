import time
import pickle
from queue import Queue
from threading import Thread
import pygame
import numpy as np
import RPi.GPIO as GPIO

# from mini_bdx_runtime.hwi_feetech_pypot import HWI
from mini_bdx_runtime.hwi_feetech_pwm_control import HWI
from mini_bdx_runtime.onnx_infer import OnnxInfer
from mini_bdx_runtime.rl_utils import (
    LowPassActionFilter,
    make_action_dict,
    quat_rotate_inverse,
)
from mini_bdx_runtime.imu import Imu


# Commands
X_RANGE = [-0.2, 0.3]
Y_RANGE = [-0.2, 0.2]
YAW_RANGE = [-0.2, 0.2]

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
        cutoff_frequency=None,
        commands=False,
        pitch_bias=0,
        replay_obs=None,
        replay_actions=None,
        zero_head=False,
        stand=False,
        rma=False,
        adaptation_module_path=None,
    ):
        self.commands = commands
        self.pitch_bias = pitch_bias
        self.zero_head = zero_head
        self.stand = stand
        self.rma = rma
        self.adaptation_module_path = adaptation_module_path

        self.num_obs = 56

        self.onnx_model_path = onnx_model_path
        self.policy = OnnxInfer(self.onnx_model_path, awd=True)

        if self.rma:
            self.adaptation_module = OnnxInfer(
                self.adaptation_module_path, "rma_history", awd=True
            )
            self.rma_obs_history_size = 50
            self.rma_obs_history = np.zeros((self.rma_obs_history_size, self.num_obs)).tolist()
            self.rma_decimation = 5  # 10 Hz if control_freq = 50 Hz

        self.replay_obs = replay_obs
        if self.replay_obs is not None:
            self.replay_obs = pickle.load(open(self.replay_obs, "rb"))

        self.replay_actions = replay_actions
        if self.replay_actions is not None:
            self.replay_actions = pickle.load(open(self.replay_actions, "rb"))
            self.replay_obs = None

        # Control
        self.control_freq = control_freq
        self.pid = pid

        self.hwi = HWI(serial_port)
        self.start()

        self.imu = Imu(sampling_freq=self.control_freq, user_pitch_bias=self.pitch_bias)

        # Scales
        self.linearVelocityScale = 1.0
        self.angularVelocityScale = 1.0
        self.dof_pos_scale = 1.0
        self.dof_vel_scale = 1.0
        self.action_scale = action_scale

        self.prev_action = np.zeros(16)

        self.init_pos = self.add_fake_antennas(list(self.hwi.init_pos.values()))

        if not self.stand:
            self.last_commands = [0.1, 0, 0]
        else:
            self.last_commands = [0, 0, 0, 0, 0, 0]

        self.command_freq = 10  # hz
        if self.commands:
            pygame.init()
            self._p1 = pygame.joystick.Joystick(0)
            self._p1.init()
            print(f"Loaded joystick with {self._p1.get_numaxes()} axes.")
            self.cmd_queue = Queue(maxsize=1)
            Thread(target=self.commands_worker, daemon=True).start()

        self.last_command_time = time.time()

        if cutoff_frequency is not None:
            self.action_filter = LowPassActionFilter(
                self.control_freq, cutoff_frequency
            )
        else:
            self.action_filter = None

    def add_fake_antennas(self, pos):
        # takes in position without antennas, adds position 0 for both antennas at the right place
        assert len(pos) == 14
        pos_with_antennas = np.insert(pos, 9, [0, 0])
        return np.array(pos_with_antennas)

    def set_zero_head(self, pos):
        pos[5] = np.deg2rad(10)
        pos[6] = np.deg2rad(-10)
        # pos[5] = 0
        # pos[6] = 0
        pos[7] = 0
        pos[8] = 0
        return pos

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
        orientation_quat = self.imu.get_data()
        if orientation_quat is None:
            print("IMU ERROR")
            return None

        if self.commands:
            self.last_commands = self.get_last_command()
            # print(self.last_commands)

        dof_pos = self.hwi.get_present_positions()  # rad
        dof_vel = self.hwi.get_present_velocities()  # rad/s

        if len(dof_pos) != 14:
            print("ERROR len(dof_pos) != 14")
            return None

        if len(dof_vel) != 14:
            print("ERROR len(dof_vel) != 14")
            return None

        dof_pos = self.add_fake_antennas(dof_pos)
        dof_vel = self.add_fake_antennas(dof_vel)

        dof_pos_scaled = list(dof_pos * self.dof_pos_scale)
        dof_vel_scaled = list(dof_vel * self.dof_vel_scale)

        projected_gravity = quat_rotate_inverse(orientation_quat, [0, 0, -1])

        feet_contacts = self.get_feet_contacts()

        cmds = self.last_commands
        # cmds = list(
        #     np.array(self.last_commands).copy()
        #     * np.array(
        #         [
        #             self.linearVelocityScale,
        #             self.linearVelocityScale,
        #             self.angularVelocityScale,
        #         ]
        #     )
        # )

        obs = np.concatenate(
            [
                projected_gravity,
                dof_pos_scaled,
                dof_vel_scaled,
                feet_contacts,
                self.prev_action,
                cmds,
            ]
        )

        return obs

    def start(self):
        self.hwi.turn_on()
        kps = [self.pid[0]] * 14
        # kps[5] = 16
        # kps[6] = 16
        # kps[7] = 16
        # kps[8] = 16

        self.hwi.set_kps(kps)

        time.sleep(2)

    def run(self):
        robot_computed_obs = []
        # voltages = []
        i = 0
        start = time.time()
        latent = None
        try:
            print("Starting")
            while True:
                t = time.time()

                obs = self.get_obs()
                if obs is None:
                    continue
                robot_computed_obs.append(obs)
                if self.replay_obs is not None:
                    if i < len(self.replay_obs):
                        obs = self.replay_obs[i]
                    else:
                        break

                if self.rma:
                    # self.rma_obs_history.append(obs)
                    # self.rma_obs_history = self.rma_obs_history[-self.rma_obs_history_size:]
                    self.rma_obs_history = np.roll(self.rma_obs_history, 1, axis=0)
                    self.rma_obs_history[0] = obs

                    if i % self.rma_decimation == 0 or latent is None:
                        latent = self.adaptation_module.infer(np.array(self.rma_obs_history).flatten())
                    obs = np.concatenate([obs, latent])

                obs = np.clip(obs, -100, 100)

                if self.replay_actions is None:
                    action = self.policy.infer(obs)
                else:
                    if i < len(self.replay_actions):
                        action = self.replay_actions[i][-(16 + 3) : -3]
                    else:
                        break

                action = np.clip(action, -5, 5)

                self.prev_action = action.copy()

                # action = np.zeros(16)
                robot_action = action * self.action_scale + self.init_pos

                if self.action_filter is not None:
                    self.action_filter.push(robot_action)
                    filtered_action = self.action_filter.get_filtered_action()
                    if time.time() - start > 2:
                        robot_action = filtered_action

                if self.zero_head:
                    robot_action = self.set_zero_head(robot_action)
                action_dict = make_action_dict(
                    robot_action, joints_order
                )  # Removes antennas
                self.hwi.set_position_all(action_dict)

                # voltages.append(self.hwi.get_present_voltages())

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
            # self.hwi.turn_off()
            pass

        pickle.dump(robot_computed_obs, open("robot_computed_obs.pkl", "wb"))
        # pickle.dump(voltages, open("voltages.pkl", "wb"))
        time.sleep(1)

        self.hwi.freeze()


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("--onnx_model_path", type=str, required=True)
    parser.add_argument("-a", "--action_scale", type=float, default=0.25)
    parser.add_argument("-p", type=int, default=32)
    parser.add_argument("-i", type=int, default=0)
    parser.add_argument("-d", type=int, default=0)
    parser.add_argument("-c", "--control_freq", type=int, default=50)
    parser.add_argument("--cutoff_frequency", type=int, default=None)
    parser.add_argument("--pitch_bias", type=float, default=0, help="deg")
    parser.add_argument(
        "--commands",
        action="store_true",
        default=False,
        help="external commands, keyboard or gamepad. Launch control_server.py on host computer",
    )

    parser.add_argument(
        "--zero_head",
        action="store_true",
        default=False,
        help="force all head dofs to zero",
    )
    parser.add_argument(
        "--stand",
        action="store_true",
        default=False,
        help="stand",
    )
    parser.add_argument("--replay_obs", type=str, required=False, default=None)
    parser.add_argument("--replay_actions", type=str, required=False, default=None)
    parser.add_argument("--rma", action="store_true", default=False)
    parser.add_argument("--adaptation_module_path", type=str, required=False)
    args = parser.parse_args()
    pid = [args.p, args.i, args.d]

    print("Done parsing args")
    rl_walk = RLWalk(
        args.onnx_model_path,
        action_scale=args.action_scale,
        pid=pid,
        control_freq=args.control_freq,
        cutoff_frequency=args.cutoff_frequency,
        commands=args.commands,
        pitch_bias=args.pitch_bias,
        replay_obs=args.replay_obs,
        replay_actions=args.replay_actions,
        zero_head=args.zero_head,
        stand=args.stand,
        rma=args.rma,
        adaptation_module_path=args.adaptation_module_path,
    )
    print("Done instantiating RLWalk")
    # rl_walk.start()
    rl_walk.run()
