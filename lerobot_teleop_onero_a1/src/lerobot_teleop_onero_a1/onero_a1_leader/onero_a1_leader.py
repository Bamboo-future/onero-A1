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

"""Onero A1 Leader teleoperator implementation for LeRobot."""

import logging
import threading
import time

from lerobot.lerobot_types import RobotAction
from lerobot.utils.decorators import check_if_not_connected
from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, TELEOPERATORS

from lerobot.teleoperators.teleoperator import Teleoperator
from lerobot_teleop_onero_a1.onero_a1_leader.config_onero_a1_leader import OneroA1LeaderConfig

logger = logging.getLogger(__name__)


class OneroA1Leader(Teleoperator):
    """
    Onero A1 arm as a teleoperation leader device.

    This teleoperator reads joint positions from an Onero A1 arm
    and outputs them as actions for controlling follower robots.

    Zero-force control (gravity compensation) is implemented using OneroArm.control_mit()
    with compute_gravity_torque() for easy manual dragging.
    """

    config_class = OneroA1LeaderConfig
    name = "onero_a1_leader"

    def __init__(self, config: OneroA1LeaderConfig):
        super().__init__(config)
        self.config = config
        self._arm = None  # OneroArm instance
        self._connected = False
        self._enabled = False
        self._joint_names = [f"joint{i}" for i in range(1, config.dof + 1)]

        # Zero-force control using MIT mode
        self._zero_force_enabled = False
        self._zero_force_thread = None
        self._zero_force_stop_event = threading.Event()
        self._time_step = 0.01  # 100 Hz control loop

        # Lock for protecting serial port access
        self._serial_lock = threading.Lock()

    @property
    def is_connected(self) -> bool:
        return self._connected and self._arm is not None

    @property
    def is_calibrated(self) -> bool:
        return True

    def calibrate(self) -> None:
        pass

    def configure(self) -> None:
        """Configure the leader arm for zero-force control mode."""
        logger.info(f"configure() called, zero_force_enabled={self.config.zero_force_enabled}")

        if not self.config.zero_force_enabled:
            logger.info(f"Zero-force control disabled for {self}")
            return

        logger.info(f"Zero-force control will use OneroArm.control_mit()")

    def _start_zero_force_control(self) -> None:
        """Start the zero-force control thread using MIT mode."""
        logger.info(f"_start_zero_force_control() called, _zero_force_enabled={self._zero_force_enabled}")

        if self._zero_force_enabled:
            logger.info("Already enabled, returning")
            return

        logger.info("Setting _zero_force_enabled to True")
        self._zero_force_enabled = True

        logger.info("Clearing stop event")
        self._zero_force_stop_event.clear()

        logger.info("Creating thread")
        self._zero_force_thread = threading.Thread(target=self._zero_force_loop, daemon=True)

        logger.info("Starting thread")
        self._zero_force_thread.start()

        logger.info(f"Zero-force control started on {self}")

    def _stop_zero_force_control(self) -> None:
        """Stop the zero-force control thread."""
        if not self._zero_force_enabled:
            return

        self._zero_force_enabled = False
        self._zero_force_stop_event.set()
        if self._zero_force_thread:
            self._zero_force_thread.join(timeout=2.0)
            self._zero_force_thread = None
        logger.info(f"Zero-force control stopped on {self}")

    def _zero_force_loop(self) -> None:
        """
        Background thread that maintains zero-force control using MIT mode.

        This calls control_mit() at ~100Hz with gravity compensation
        to keep the leader arm in zero-force mode for easy dragging.
        """
        logger.info("_zero_force_loop: ENTERED (using MIT mode)")

        if self._arm is None:
            logger.warning("Arm not connected, zero-force loop exiting")
            return

        # Get initial position for smooth startup
        try:
            initial_state = self._arm.get_arm_state_from_motor()
            target_q = [float(initial_state.positions[i]) for i in range(self.config.dof)]
        except Exception as e:
            logger.warning(f"Failed to get initial position: {e}")
            target_q = [0.0] * self.config.dof

        # Build gain arrays (once)
        kp = [self.config.zero_force_kp] * self.config.dof
        kd = [self.config.zero_force_kd] * self.config.dof
        dq = [0.0] * self.config.dof  # Target velocity is always zero

        next_t = time.monotonic()

        while not self._zero_force_stop_event.is_set() and self._arm:
            try:
                # Read current position
                with self._serial_lock:
                    state = self._arm.get_arm_state_from_motor()

                if state is None:
                    logger.warning("Failed to get arm state in zero-force loop")
                    time.sleep(0.1)
                    continue

                # Update target position (follow actual position with zero stiffness)
                # For pure zero-force, kp should be low and we compensate with gravity torque
                current_q = [float(state.positions[i]) for i in range(self.config.dof)]

                # Compute gravity compensation torque
                with self._serial_lock:
                    tau = self._arm.compute_gravity_torque(state.positions)

                # Send MIT control command with gravity compensation
                # q = current position, dq = 0, tau = negative gravity torque
                with self._serial_lock:
                    self._arm.control_mit(kp, kd, state.positions, dq, tau)

                # Sleep until next tick (100 Hz)
                next_t += self._time_step
                sleep_time = next_t - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    # Fell behind, reset
                    next_t = time.monotonic()
                    time.sleep(self._time_step)

            except Exception as e:
                if self._zero_force_enabled:
                    logger.warning(f"Zero-force control error: {e}")
                time.sleep(0.1)

        logger.debug("Zero-force control loop ended")

    @property
    def action_features(self) -> dict[str, type]:
        """Define action features - joint positions."""
        features = {}
        for joint_name in self._joint_names:
            features[f"{joint_name}.pos"] = float
        return features

    @property
    def feedback_features(self) -> dict[str, type]:
        """Define feedback features (same as action for position control)."""
        return self.action_features

    def connect(self, calibrate: bool = True) -> None:
        """Connect to the A1 leader arm using OneroArm."""
        if self.is_connected:
            logger.warning(f"{self} is already connected")
            return

        logger.info(f"Connecting to {self} on {self.config.port}...")

        try:
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

            if not self._arm.valid():
                raise RuntimeError(f"Failed to create OneroArm on {self.config.port}")

            logger.info(f"OneroArm created")

            # Enable motors - NOTE: This might hang if hardware is not responding
            logger.info(f"Enabling motors on {self.config.port}...")
            enabled = self._arm.enable_motors()
            if not enabled:
                raise RuntimeError(f"Failed to enable motors on {self.config.port}")

            logger.info(f"Motors enabled")

            self._connected = True
            self._enabled = True
            logger.info(f"Successfully connected and enabled {self}")

            if calibrate and not self.is_calibrated:
                logger.info("Calibrating...")
                self.calibrate()

            # Configure zero-force control
            self.configure()

            # Start zero-force control in background thread
            if self.config.zero_force_enabled:
                self._start_zero_force_control()

        except ImportError as e:
            raise RuntimeError("oneroarm package not found") from e
        except Exception as e:
            self._connected = False
            self._enabled = False
            self._arm = None
            raise RuntimeError(f"Failed to connect to A1 arm on {self.config.port}: {e}") from e

    def disconnect(self) -> None:
        """Disconnect from the A1 leader arm."""
        # Stop zero-force control first
        self._stop_zero_force_control()

        if self._arm and self._enabled:
            try:
                self._arm.disable_motors()
                logger.info(f"Disconnected {self}")
            except Exception as e:
                logger.warning(f"Error during disconnect: {e}")
        self._connected = False
        self._enabled = False
        self._arm = None

    @check_if_not_connected
    def get_action(self) -> RobotAction:
        """
        Get action from the leader arm.

        Reads current joint positions and returns them as action.
        For bimanual control with mirror mode, this can be used to drive a follower arm.
        """
        if self._arm is None:
            raise ConnectionError("Leader arm not connected")

        with self._serial_lock:
            positions = self._arm.get_joint_positions_from_motors()

        if positions is None or len(positions) == 0:
            logger.warning("Failed to get joint positions, returning zeros")
            return {f"{name}.pos": 0.0 for name in self._joint_names}

        action = {}
        for i, joint_name in enumerate(self._joint_names):
            pos = float(positions[i])
            # Apply mirroring if enabled (for left-right symmetric setup)
            if self.config.mirror_mode and i in [1, 3, 5]:  # J2, J4, J6
                pos = -pos
            action[f"{joint_name}.pos"] = pos

        return action

    @check_if_not_connected
    def send_feedback(self, feedback: dict[str, float]) -> None:
        """
        Send feedback to the leader arm (for handover during DAgger).
        """
        if self._arm is None:
            raise ConnectionError("Leader arm not connected")

        target_positions = []
        for i, joint_name in enumerate(self._joint_names):
            key = f"{joint_name}.pos"
            if key in feedback:
                value = float(feedback[key])
                if self.config.mirror_mode and i in [1, 3, 5]:
                    value = -value
                target_positions.append(value)
            else:
                # Read current position
                with self._serial_lock:
                    state = self._arm.get_arm_state_from_motor()
                if state and i < len(state.positions):
                    target_positions.append(float(state.positions[i]))
                else:
                    target_positions.append(0.0)

        if len(target_positions) == self.config.dof:
            with self._serial_lock:
                self._arm.movej(target_positions, speed_scale=1.0, trajectory_connect=1)

    def __del__(self):
        """Cleanup on deletion."""
        try:
            self._stop_zero_force_control()
            if self.is_connected:
                self.disconnect()
        except Exception:
            pass
