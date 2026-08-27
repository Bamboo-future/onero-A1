#!/usr/bin/env python

# Copyright 2025 The HuggingFace Inc. team. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Configuration for OneRobotics A1 robot."""

from dataclasses import dataclass, field
from typing import Literal

from lerobot.cameras import CameraConfig
from lerobot.robots.config import RobotConfig

# Default joint names for 7-DOF A1 arm
DEFAULT_JOINT_NAMES = tuple(f"joint{i}" for i in range(1, 8))

# Default position limits (radians) - conservative limits
DEFAULT_POSITION_LIMITS = (
    (-3.14, 3.14),  # joint1
    (-1.57, 1.57),  # joint2
    (-3.14, 3.14),  # joint3
    (-1.57, 1.57),  # joint4
    (-3.14, 3.14),  # joint5
    (-1.57, 1.57),  # joint6
    (-3.14, 3.14),  # joint7
)

# Default camera configuration
DEFAULT_CAMERA_SHAPE = (480, 640, 3)


@dataclass
class OneroA1SingleArmConfig(RobotConfig):
    """Configuration for a single OneRobotics A1 arm."""

    # Serial port configuration
    port: str = "/dev/ttyACM0"
    baud_rate: int = 921600

    # Robot model identification
    robot_model: str = "a1_l"  # "a1_l" for left, "a1_r" for right
    version: str = "A1"
    mount_orientation: Literal["vertical", "horizontal"] = "vertical"

    # Control parameters
    dof: int = 7
    disable_torque_on_disconnect: bool = True
    max_velocity_radps: float = 1.0

    # Joint configuration
    joint_names: tuple[str, ...] = DEFAULT_JOINT_NAMES
    position_limits: tuple[tuple[float, float], ...] = DEFAULT_POSITION_LIMITS

    # Safety limits
    max_delta_rad: float = 0.20  # Maximum change per command
    enable_safety: bool = True

    # Calibration
    calibration_dir: str | None = None

    # Control mode: "movej" uses the SDK's trajectory mode (default),
    # "mit" enables the MIT position-tracking loop (higher stiffness).
    control_mode: Literal["movej", "mit"] = "movej"

    # Cameras attached to this arm, e.g.
    # {"top": OpenCVCameraConfig(index_or_path="/dev/video0", width=640, height=480, fps=15)}
    cameras: dict[str, CameraConfig] = field(default_factory=dict)


@dataclass
class OneroA1Config(RobotConfig):
    """Configuration for bimanual OneRobotics A1 setup (master + slave)."""

    # Master and slave arm configurations
    master_arm_config: OneroA1SingleArmConfig = field(default_factory=lambda: OneroA1SingleArmConfig(
        port="/dev/ttyACM0",
        robot_model="a1_l",
    ))
    slave_arm_config: OneroA1SingleArmConfig = field(default_factory=lambda: OneroA1SingleArmConfig(
        port="/dev/ttyACM1",
        robot_model="a1_r",
    ))

    # Master-slave control mode
    control_mode: Literal["joint", "cartesian"] = "joint"
    mirror_mode: bool = True  # Mirror for left-right configuration

    # Calibration
    calibration_dir: str | None = None

    # Cameras attached to the bimanual setup (e.g. a top-down camera).
    # Individual arm cameras are configured via master_arm_config/slave_arm_config.
    cameras: dict[str, CameraConfig] = field(default_factory=dict)


# 兼容 lerobot 源码内置的 onero_a1 模块：若同名配置已被注册（例如
# lerobot.robots.onero_a1 已导入），则跳过，避免 draccus 重复注册报错。
for _name, _cls in (
    ("onero_a1_single_arm", OneroA1SingleArmConfig),
    ("onero_a1", OneroA1Config),
):
    try:
        RobotConfig.register_subclass(_name)(_cls)
    except ValueError:
        pass
