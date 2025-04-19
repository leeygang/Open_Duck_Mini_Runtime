import json
from typing import Optional


class DuckConfig:
    def __init__(self, config_json_path: Optional[str] = None):
        self.json_config = (
            json.load(open(config_json_path, "r")) if config_json_path else {}
        )

        if config_json_path is None:
            print("Warning : didn't provide a config json path, using default values")

        self.start_paused = self.json_config.get("start_paused", False)
        self.imu_upside_down = self.json_config.get("imu_upside_down", False)
        self.phase_frequency_factor_offset = self.json_config.get(
            "phase_frequency_factor_offset", 0.0
        )

        expression_features = self.json_config.get("expression_features", {})

        self.eyes = expression_features.get("eyes", False)
        self.projector = expression_features.get("projector", False)
        self.antennas = expression_features.get("antennas", False)
        self.speaker = expression_features.get("speaker", False)
        self.microphone = expression_features.get("microphone", False)
        self.camera = expression_features.get("camera", False)

        # default joints offsets are 0.0
        self.joints_offset = self.json_config.get(
            "joints_offsets",
            {
                "left_hip_yaw": 0.0,
                "left_hip_roll": 0.0,
                "left_hip_pitch": 0.0,
                "left_knee": 0.0,
                "left_ankle": 0.0,
                "neck_pitch": 0.0,
                "head_pitch": 0.0,
                "head_yaw": 0.0,
                "head_roll": 0.00,
                "right_hip_yaw": 0.0,
                "right_hip_roll": 0.0,
                "right_hip_pitch": 0.0,
                "right_knee": 0.0,
                "right_ankle": 0.0,
            },
        )
