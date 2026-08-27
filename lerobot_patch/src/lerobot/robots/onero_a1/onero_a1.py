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

"""OneRobotics A1 robot implementation for LeRobot."""

import logging
import threading
import time
from functools import cached_property
from typing import Any
from concurrent.futures import ThreadPoolExecutor, TimeoutError as FuturesTimeoutError

import numpy as np

from lerobot.cameras import make_cameras_from_configs
from lerobot.lerobot_types import RobotAction, RobotObservation
from lerobot.utils.decorators import check_if_not_connected
from lerobot.utils.constants import ROBOTS

from lerobot.robots.robot import Robot
from lerobot.robots.onero_a1.config_onero_a1 import OneroA1Config, OneroA1SingleArmConfig

logger = logging.getLogger(__name__)

MOVE_OK = 0

# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# 阶段一：紧急止血 - 串口调用超时保护
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

# 全局线程池用于超时包装（避免每次创建新线程）
_timeout_executor_robot = ThreadPoolExecutor(max_workers=4, thread_name_prefix="onero_robot_timeout_")
_SERIAL_CALL_TIMEOUT = 0.05  # 50ms超时


def _run_with_timeout_robot(func, *args, timeout=_SERIAL_CALL_TIMEOUT, **kwargs):
    """
    在指定超时内执行函数，超时则返回 None 并记录警告。

    用于包装所有可能阻塞的串口调用（get_arm_state_from_motor, movej 等）。
    """
    try:
        future = _timeout_executor_robot.submit(func, *args, **kwargs)
        return future.result(timeout=timeout)
    except FuturesTimeoutError:
        logger.warning(f"Serial call {func.__name__}() timed out after {timeout}s, returning None")
        return None
    except Exception as e:
        logger.warning(f"Serial call {func.__name__}() raised exception: {e}")
        return None


class OneroA1SingleArm(Robot):
    """
    Single OneRobotics A1 arm controlled via the oneroarm Python API.

    This class wraps the OneroArm SDK to provide LeRobot-compatible interface.
    """

    config_class = OneroA1SingleArmConfig
    name = "onero_a1_single_arm"

    def __init__(self, config: OneroA1SingleArmConfig):
        super().__init__(config)
        self.config = config
        self._arm = None
        self._connected = False
        self._enabled = False
        self.cameras = make_cameras_from_configs(config.cameras)

        # MIT控制相关变量
        self._mit_mode_enabled = False
        self._mit_thread = None
        self._mit_stop_event = threading.Event()
        self._mit_target_q = None
        self._mit_lock = threading.Lock()
        self._serial_lock = threading.Lock()  # 串口访问锁（SDK 要求同一实例必须串行化）
        self._last_state_positions = None  # 上一帧位置兜底
        self._mit_time_step = 0.01  # 100Hz

        # MIT控制增益（可调）
        self._mit_kp = [150.0, 150.0, 150.0, 150.0, 30.0, 30.0, 30.0]
        self._mit_kd = [4.0, 4.0, 4.0, 4.0, 1.0, 1.0, 1.0]

    def _serial_call(self, func, *args, **kwargs):
        """串行化访问串口：SDK 要求同一 OneroArm 实例的方法必须由调用方串行化。"""
        with self._serial_lock:
            return _run_with_timeout_robot(func, *args, timeout=_SERIAL_CALL_TIMEOUT, **kwargs)

    @property
    def is_connected(self) -> bool:
        return self._connected and self._arm is not None and all(
            cam.is_connected for cam in self.cameras.values()
        )

    @property
    def is_calibrated(self) -> bool:
        # A1 calibration is handled by the SDK
        return True

    def calibrate(self) -> None:
        # No explicit calibration needed for A1
        pass

    def configure(self) -> None:
        # Apply any runtime configuration
        pass

    @property
    def _cameras_ft(self) -> dict[str, tuple]:
        """Feature shapes for RGB cameras attached to this arm."""
        return {
            cam_name: (cam.height, cam.width, 3) for cam_name, cam in self.cameras.items()
        }

    @property
    def observation_features(self) -> dict[str, type | tuple]:
        """Define observation features - joint positions + camera images."""
        features = {}
        for joint_name in self.config.joint_names:
            features[f"{joint_name}.pos"] = float
        features.update(self._cameras_ft)
        return features

    @property
    def action_features(self) -> dict[str, type]:
        """Define action features - target joint positions (cameras are not actions)."""
        return {f"{joint_name}.pos": float for joint_name in self.config.joint_names}

    def connect(self, calibrate: bool = True) -> None:
        """Connect to the A1 arm."""
        if self.is_connected:
            logger.warning(f"{self} is already connected")
            return

        logger.info(f"Connecting to {self} on {self.config.port}...")

        try:
            # Import oneroarm - should be installed from ArmApi-main
            import oneroarm

            # Build OneroConfig
            cfg = oneroarm.OneroConfig()
            cfg.device = self.config.port
            cfg.robot_model = self.config.robot_model
            cfg.version = self.config.version
            cfg.mount_orientation = self.config.mount_orientation
            cfg.dof = self.config.dof
            cfg.baud_rate = self.config.baud_rate

            # Create arm instance
            self._arm = oneroarm.OneroArm(cfg)
            self._connected = True

            # Enable motors
            self._enabled = self._arm.enable_motors()
            if not self._enabled:
                raise RuntimeError(f"Failed to enable motors on {self.config.port}")

            logger.info(f"Successfully connected and enabled {self}")

            if calibrate and not self.is_calibrated:
                self.calibrate()

            # Connect cameras attached to this arm
            for cam in self.cameras.values():
                cam.connect()

            # Enable MIT position tracking when requested (follower mode)
            if self.config.control_mode == "mit":
                self.enable_mit_mode(True)
                logger.info(f"MIT control mode enabled for {self}")

        except ImportError as e:
            raise RuntimeError(
                "oneroarm package not found. Install it from /home/exp/Downloads/ArmApi-main/python"
            ) from e
        except Exception as e:
            self._connected = False
            self._arm = None
            raise RuntimeError(f"Failed to connect to A1 arm on {self.config.port}: {e}") from e

    def disconnect(self) -> None:
        """Disconnect from the A1 arm."""
        # 停止MIT控制线程
        self._stop_mit_control()

        if self._arm and self._enabled:
            try:
                if self.config.disable_torque_on_disconnect:
                    self._arm.disable_motors()
                logger.info(f"Disconnected {self}")
            except Exception as e:
                logger.warning(f"Error during disconnect: {e}")
        for cam in self.cameras.values():
            try:
                cam.disconnect()
            except Exception as e:
                logger.warning(f"Failed to disconnect camera: {e}")

        self._connected = False
        self._enabled = False
        self._arm = None

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # MIT力位混合控制
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    def _start_mit_control(self) -> None:
        """启动MIT控制线程（用于从臂位置跟踪）"""
        if self._mit_mode_enabled:
            return

        self._mit_mode_enabled = True
        self._mit_stop_event.clear()
        self._mit_thread = threading.Thread(target=self._mit_control_loop, daemon=True)
        self._mit_thread.start()
        logger.info(f"MIT control started on {self}")

    def _stop_mit_control(self) -> None:
        """停止MIT控制线程"""
        if not self._mit_mode_enabled:
            return

        self._mit_mode_enabled = False
        self._mit_stop_event.set()
        if self._mit_thread:
            self._mit_thread.join(timeout=2.0)
            self._mit_thread = None
        logger.info(f"MIT control stopped on {self}")

    def _mit_control_loop(self):
        """MIT控制循环 - 从臂位置跟踪"""
        logger.info("MIT control loop started")

        if self._arm is None:
            logger.warning("Arm not connected, MIT loop exiting")
            return

        # 获取初始位置
        try:
            initial_state = self._serial_call(self._arm.get_arm_state_from_motor)
            with self._mit_lock:
                if self._mit_target_q is None and initial_state is not None:
                    self._mit_target_q = [float(initial_state.positions[i]) for i in range(self.config.dof)]
        except Exception as e:
            logger.warning(f"Failed to get initial position: {e}")
            with self._mit_lock:
                self._mit_target_q = [0.0] * self.config.dof

        dq = [0.0] * self.config.dof  # 目标速度为0

        next_t = time.monotonic()

        while not self._mit_stop_event.is_set() and self._arm:
            try:
                # 获取当前状态（与 get_observation/send_action 串行化访问串口）
                with self._serial_lock:
                    state = _run_with_timeout_robot(
                        self._arm.get_arm_state_from_motor,
                        timeout=_SERIAL_CALL_TIMEOUT
                    )
                    if state is None:
                        time.sleep(0.01)
                        continue

                    # 获取目标位置（线程安全）
                    with self._mit_lock:
                        if self._mit_target_q is not None:
                            target_q = list(self._mit_target_q)
                        else:
                            target_q = [float(state.positions[i]) for i in range(self.config.dof)]

                    # 计算重力补偿
                    tau_g = self._arm.compute_gravity_torque(state.positions)

                    # 发送MIT控制命令
                    self._arm.control_mit(self._mit_kp, self._mit_kd, target_q, dq, tau_g)

                # 100Hz循环
                next_t += self._mit_time_step
                sleep_time = next_t - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_t = time.monotonic()
                    time.sleep(self._mit_time_step)

            except Exception as e:
                if self._mit_mode_enabled:
                    logger.warning(f"MIT control error: {e}")
                time.sleep(0.1)

        logger.debug("MIT control loop ended")

    def set_mit_target(self, target_positions: list[float]) -> None:
        """设置MIT控制目标位置（用于从臂跟踪）"""
        with self._mit_lock:
            self._mit_target_q = list(target_positions)

    def enable_mit_mode(self, enabled: bool = True) -> None:
        """启用或禁用MIT模式"""
        if enabled and not self._mit_mode_enabled:
            self._start_mit_control()
        elif not enabled and self._mit_mode_enabled:
            self._stop_mit_control()

    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
    # 传统接口
    # ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        """
        Get current joint positions.

        🔴 阶段一：添加50ms超时保护
        """
        if self._arm is None:
            raise ConnectionError("Arm not connected")

        state = self._serial_call(self._arm.get_arm_state_from_motor)
        if state is None:
            logger.warning("Failed to get arm state (timeout or error)")
            # 用上一帧位置兜底（无历史则全零）；必须继续读相机，
            # 否则数据集帧缺少图像键，录制时 build_dataset_frame 会 KeyError
            positions = (
                self._last_state_positions
                if self._last_state_positions is not None
                else [0.0] * self.config.dof
            )
        else:
            positions = [float(state.positions[i]) for i in range(self.config.dof)]
            self._last_state_positions = positions

        obs = {}
        for i, joint_name in enumerate(self.config.joint_names):
            obs[f"{joint_name}.pos"] = positions[i]

        # Capture images from cameras
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.read_latest()

        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        """
        Send joint position commands.

        🔴 阶段一：添加50ms超时保护到所有串口调用
        """
        if self._arm is None:
            raise ConnectionError("Arm not connected")

        # Extract joint positions from action
        # Read serial port state ONCE before loop to avoid 7x timeout avalanche
        safety_state = None
        if self.config.enable_safety:
            safety_state = self._serial_call(self._arm.get_arm_state_from_motor)

        target_positions = []
        for joint_name in self.config.joint_names:
            key = f"{joint_name}.pos"
            if key not in action:
                raise ValueError(f"Action missing key: {key}")
            value = float(action[key])

            # Apply safety limits using pre-read state (no serial access in loop)
            if self.config.enable_safety and safety_state is not None:
                current = safety_state.positions[self.config.joint_names.index(joint_name)]
                delta = value - current
                if abs(delta) > self.config.max_delta_rad:
                    logger.warning(
                        f"Clamping delta for {joint_name}: {delta:.3f} > {self.config.max_delta_rad}"
                    )
                    value = current + np.sign(delta) * self.config.max_delta_rad

                # Check position limits
                limits = self.config.position_limits[self.config.joint_names.index(joint_name)]
                value = np.clip(value, limits[0], limits[1])

            target_positions.append(value)

        # 根据模式选择控制方式
        if self._mit_mode_enabled:
            # MIT模式：更新目标位置，由后台线程处理
            self.set_mit_target(target_positions)
        else:
            # 传统movej模式
            ret = self._arm.movej(
                target_positions,
                speed_scale=self.config.max_velocity_radps,
                trajectory_connect=0,  # Disable trajectory waiting to prevent blocking
            )

            if ret != MOVE_OK:
                logger.warning(f"Move command failed with code: {ret}")

        # Return the actual sent action
        sent_action = {}
        for i, joint_name in enumerate(self.config.joint_names):
            sent_action[f"{joint_name}.pos"] = target_positions[i]

        return sent_action

    def __del__(self):
        """Cleanup on deletion."""
        try:
            if self.is_connected:
                self.disconnect()
        except Exception:
            pass


class OneroA1(Robot):
    """
    Bimanual OneRobotics A1 setup with master-slave control.

    This class manages two A1 arms where the master arm controls the slave arm
    in either joint or Cartesian space with optional mirroring.
    """

    config_class = OneroA1Config
    name = "onero_a1"

    def __init__(self, config: OneroA1Config):
        super().__init__(config)
        self.config = config

        # Create master and slave arm instances
        self.master_arm = OneroA1SingleArm(config.master_arm_config)
        self.slave_arm = OneroA1SingleArm(config.slave_arm_config)
        self.cameras = make_cameras_from_configs(config.cameras)

    @property
    def is_connected(self) -> bool:
        return (
            self.master_arm.is_connected
            and self.slave_arm.is_connected
            and all(cam.is_connected for cam in self.cameras.values())
        )

    @property
    def is_calibrated(self) -> bool:
        return self.master_arm.is_calibrated and self.slave_arm.is_calibrated

    def calibrate(self) -> None:
        self.master_arm.calibrate()
        self.slave_arm.calibrate()

    def configure(self) -> None:
        self.master_arm.configure()
        self.slave_arm.configure()

    @cached_property
    def observation_features(self) -> dict[str, type | tuple]:
        """Combine observations from both arms with prefixes + camera images."""
        features = {}
        # Master arm observations
        for joint_name in self.config.master_arm_config.joint_names:
            features[f"master_{joint_name}.pos"] = float
        # Slave arm observations
        for joint_name in self.config.slave_arm_config.joint_names:
            features[f"slave_{joint_name}.pos"] = float
        # Cameras configured on individual arms (keys prefixed by arm)
        for arm_prefix, arm in (("master", self.master_arm), ("slave", self.slave_arm)):
            for cam_name, cam in arm.cameras.items():
                features[f"{arm_prefix}_{cam_name}"] = (cam.height, cam.width, 3)
        # Cameras configured at the bimanual level
        for cam_name, cam in self.cameras.items():
            features[cam_name] = (cam.height, cam.width, 3)
        return features

    @cached_property
    def action_features(self) -> dict[str, type]:
        """Actions target both arms independently (cameras are not actions)."""
        features = {}
        for joint_name in self.config.master_arm_config.joint_names:
            features[f"master_{joint_name}.pos"] = float
        for joint_name in self.config.slave_arm_config.joint_names:
            features[f"slave_{joint_name}.pos"] = float
        return features

    def connect(self, calibrate: bool = True) -> None:
        """Connect to both arms."""
        logger.info("Connecting to master arm...")
        self.master_arm.connect(calibrate=calibrate)
        logger.info("Connecting to slave arm...")
        self.slave_arm.connect(calibrate=calibrate)
        logger.info("Both arms connected.")

        # Connect cameras attached to the bimanual setup
        for cam in self.cameras.values():
            cam.connect()

    def disconnect(self) -> None:
        """Disconnect both arms and cameras."""
        self.master_arm.disconnect()
        self.slave_arm.disconnect()
        for cam in self.cameras.values():
            try:
                cam.disconnect()
            except Exception as e:
                logger.warning(f"Failed to disconnect camera: {e}")

    @check_if_not_connected
    def get_observation(self) -> RobotObservation:
        """Get observations from both arms."""
        obs = {}

        # Master arm observations
        master_obs = self.master_arm.get_observation()
        for key, value in master_obs.items():
            obs[f"master_{key}"] = value

        # Slave arm observations
        slave_obs = self.slave_arm.get_observation()
        for key, value in slave_obs.items():
            obs[f"slave_{key}"] = value

        # Capture images from bimanual-level cameras
        for cam_key, cam in self.cameras.items():
            obs[cam_key] = cam.read_latest()

        return obs

    @check_if_not_connected
    def send_action(self, action: RobotAction) -> RobotAction:
        """
        Send actions to both arms.

        For master-slave mode, you can either:
        1. Control both arms independently by providing both master_ and slave_ keys
        2. Use master_slave_follow() method for automatic following
        """
        sent_action = {}

        # Extract master action
        master_action = {}
        for key in self.config.master_arm_config.joint_names:
            full_key = f"master_{key}.pos"
            if full_key in action:
                master_action[f"{key}.pos"] = action[full_key]

        # Extract slave action
        slave_action = {}
        for key in self.config.slave_arm_config.joint_names:
            full_key = f"slave_{key}.pos"
            if full_key in action:
                slave_action[f"{key}.pos"] = action[full_key]

        # Send to master
        if master_action:
            master_sent = self.master_arm.send_action(master_action)
            for key, value in master_sent.items():
                sent_action[f"master_{key}"] = value

        # Send to slave
        if slave_action:
            slave_sent = self.slave_arm.send_action(slave_action)
            for key, value in slave_sent.items():
                sent_action[f"slave_{key}"] = value

        return sent_action

    def master_slave_follow(self, mirror: bool | None = None) -> RobotAction:
        """
        Get the action that makes slave follow master.

        Args:
            mirror: If None, uses config.mirror_mode. Otherwise overrides.

        Returns:
            Action dict with slave target positions
        """
        if mirror is None:
            mirror = self.config.mirror_mode

        # Get master current position
        master_obs = self.master_arm.get_observation()
        master_positions = []
        for i, key in enumerate(self.config.master_arm_config.joint_names):
            master_positions.append(master_obs[f"master_{key}.pos"])

        # Apply mirroring if enabled (for left-right symmetric setup)
        if mirror:
            slave_positions = self._mirror_positions(master_positions)
        else:
            slave_positions = master_positions

        # Build slave action
        slave_action = {}
        for i, key in enumerate(self.config.slave_arm_config.joint_names):
            slave_action[f"slave_{key}.pos"] = slave_positions[i]

        # Get the actual sent action by calling send_action
        sent_action = {}
        if slave_action:
            # Extract slave action without prefix for send_action
            raw_slave_action = {k.replace("slave_", "").replace(".pos", ".pos"): v
                              for k, v in slave_action.items()}
            slave_sent = self.slave_arm.send_action(raw_slave_action)
            for key, value in slave_sent.items():
                sent_action[f"slave_{key}"] = value

        return sent_action

    def _mirror_positions(self, positions: list[float]) -> list[float]:
        """
        Mirror joint positions for left-right symmetric configuration.

        For a typical 7-DOF arm, joints 1, 3, 5 need to be mirrored (negated).
        """
        mirrored = []
        for i, pos in enumerate(positions):
            if i in [1, 3, 5]:  # J2, J4, J6 for 1-indexed
                mirrored.append(-pos)
            else:
                mirrored.append(pos)
        return mirrored

    def __del__(self):
        """Cleanup on deletion."""
        try:
            if self.is_connected:
                self.disconnect()
        except Exception:
            pass
