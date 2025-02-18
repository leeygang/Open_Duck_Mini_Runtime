
from glob import glob
import os
import numpy as np
import json


class ReferenceMotion:
    def __init__(self, directory):
        self.directory = directory
        # load all json files except metadata.json
        self.json_files = glob(self.directory + "/*.json")
        self.data = {}
        self.period = None
        self.fps = None
        self.frame_offsets = None
        self.dx_range = [0, 0]
        self.dy_range = [0, 0]
        self.dtheta_range = [0, 0]
        self.dxs = []
        self.dys = []
        self.dthetas = []
        self.data_array = []
        self.process()

        self.slices = {}
        self.slices["root_pos"] = slice(
            self.frame_offsets["root_pos"], self.frame_offsets["root_quat"]
        )
        self.slices["root_quat"] = slice(
            self.frame_offsets["root_quat"], self.frame_offsets["joints_pos"]
        )
        self.slices["linear_vel"] = slice(
            self.frame_offsets["world_linear_vel"],
            self.frame_offsets["world_angular_vel"],
        )
        self.slices["angular_vel"] = slice(
            self.frame_offsets["world_angular_vel"], self.frame_offsets["joints_vel"]
        )
        self.slices["joints_pos"] = slice(
            self.frame_offsets["joints_pos"], self.frame_offsets["left_toe_pos"]
        )
        self.slices["joint_vels"] = slice(
            self.frame_offsets["joints_vel"], self.frame_offsets["left_toe_vel"]
        )
        self.slices["left_toe_pos"] = slice(
            self.frame_offsets["left_toe_pos"], self.frame_offsets["right_toe_pos"]
        )
        self.slices["right_toe_pos"] = slice(
            self.frame_offsets["right_toe_pos"], self.frame_offsets["world_linear_vel"]
        )

    def process(self):
        for file in self.json_files:

            if self.period is None:
                tmp_file = json.load(open(file))
                self.period = tmp_file["Placo"]["period"]
                self.fps = tmp_file["FPS"]
                self.frame_offsets = tmp_file["Frame_offset"][0]
                num_frames = len(tmp_file["Frames"])

            name = os.path.basename(file).strip(".json")
            split = name.split("_")
            dx = float(split[0])
            dy = float(split[1])
            dtheta = float(split[2])

            if dx not in self.dxs:
                self.dxs.append(dx)

            if dy not in self.dys:
                self.dys.append(dy)

            if dtheta not in self.dthetas:
                self.dthetas.append(dtheta)

            self.dx_range = [min(dx, self.dx_range[0]), max(dx, self.dx_range[1])]
            self.dy_range = [min(dy, self.dy_range[0]), max(dy, self.dy_range[1])]
            self.dtheta_range = [
                min(dtheta, self.dtheta_range[0]),
                max(dtheta, self.dtheta_range[1]),
            ]

            if dx not in self.data:
                self.data[dx]: dict = {}

            if dy not in self.data[dx]:
                self.data[dx][dy]: dict = {}

            if dtheta not in self.data[dx][dy]:
                self.data[dx][dy][dtheta] = json.load(open(file))["Frames"]

        self.dxs = sorted(self.dxs)
        self.dys = sorted(self.dys)
        self.dthetas = sorted(self.dthetas)

        # print("dx range: ", self.dx_range)
        # print("dy range: ", self.dy_range)
        # print("dtheta range: ", self.dtheta_range)

        nb_dx = len(self.dxs)
        nb_dy = len(self.dys)
        nb_dtheta = len(self.dthetas)

        # print("nb dx", nb_dx)
        # print("nb dy", nb_dy)
        # print("nb dtheta", nb_dtheta)

        self.data_array = nb_dx * [None]

        for x, dx in enumerate(self.dxs):
            self.data_array[x] = nb_dy * [None]
            for y, dy in enumerate(self.dys):
                self.data_array[x][y] = nb_dtheta * [None]
                for t, dtheta in enumerate(self.dthetas):
                    self.data_array[x][y][t] = self.data[dx][dy][dtheta]


    def vel_to_index(self, dx, dy, dtheta):
        # First, clamp/cap to the ranges
        dx = min(max(dx, self.dx_range[0]), self.dx_range[1])
        dy = min(max(dy, self.dy_range[0]), self.dy_range[1])
        dtheta = min(max(dtheta, self.dtheta_range[0]), self.dtheta_range[1])

        # Find the closest values in self.dxs, self.dys, self.dthetas
        ix = np.argmin(np.abs(np.array(self.dxs) - dx))
        iy = np.argmin(np.abs(np.array(self.dys) - dy))
        itheta = np.argmin(np.abs(np.array(self.dthetas) - dtheta))

        return ix, iy, itheta

    def get_closest_reference_motion(self, dx, dy, dtheta, i):
        ix, iy, itheta = self.vel_to_index(dx, dy, dtheta)
        return self.data_array[ix][iy][itheta][i]
