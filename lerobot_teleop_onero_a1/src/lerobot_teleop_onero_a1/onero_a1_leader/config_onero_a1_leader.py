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

"""Configuration for Onero A1 Leader teleoperator."""

from dataclasses import dataclass

from lerobot.teleoperators.config import TeleoperatorConfig


@dataclass
class OneroA1LeaderConfig(TeleoperatorConfig):
    """Configuration for Onero A1 Leader teleoperator."""

    # Serial port configuration
    port: str = "/dev/ttyACM0"
    baud_rate: int = 921600

    # Robot model identification
    robot_model: str = "a1_l"  # "a1_l" for left, "a1_r" for right
    version: str = "A1"
    mount_orientation: str = "vertical"

    # Control parameters
    dof: int = 7
    mirror_mode: bool = True  # Mirror for slave arm (left-right symmetric)

    # Zero-force control (gravity compensation) parameters
    zero_force_enabled: bool = True  # Enable zero-force control for easy dragging
    zero_force_kp: float = 5.0  # Proportional gain for MIT control (reserved)
    zero_force_kd: float = 0.5  # Derivative gain for MIT control (reserved)


# 兼容 lerobot 源码内置的 onero_a1_leader 模块：若同名配置已被注册则跳过。
try:
    TeleoperatorConfig.register_subclass("onero_a1_leader")(OneroA1LeaderConfig)
except ValueError:
    pass
