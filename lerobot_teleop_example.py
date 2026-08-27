#!/usr/bin/env python3
"""
LeRobot 遥操作示例 - 使用主臂控制从臂

这是标准的 LeRobot 遥操作方式：
- Teleoperator (主臂) → 读取输入 → 生成动作
- Robot (从臂) → 接收动作 → 执行运动

使用方法：
    python lerobot_teleop_example.py
"""

import signal
import sys
import time

from lerobot.teleoperators.onero_a1_leader import OneroA1Leader, OneroA1LeaderConfig
from lerobot_robot_onero_a1 import OneroA1SingleArm, OneroA1SingleArmConfig


class LeRobotTeleopControl:
    """LeRobot 标准遥操作控制"""

    def __init__(self):
        # 创建 Teleoperator（主臂/输入设备）
        self.teleop_config = OneroA1LeaderConfig(
            port="/dev/ttyACM0",  # 主臂（序列号 207936AC4343）
            robot_model="a1_r",
            mirror_mode=True,  # 镜像模式，适合左右对称
        )

        # 创建 Robot（从臂）
        self.robot_config = OneroA1SingleArmConfig(
            port="/dev/ttyACM1",  # 从臂（序列号 209636A54343）
            robot_model="a1_l",
        )

        self.teleop = None
        self.robot = None
        self.running = False

    def connect(self):
        """连接 Teleoperator 和 Robot"""
        print("=" * 60)
        print("     LeRobot 遥操作系统")
        print("=" * 60)
        print()

        # 创建 Teleoperator
        print(f"[1/2] 创建 Teleoperator (主臂)")
        print(f"      端口: {self.teleop_config.port}")
        print(f"      型号: {self.teleop_config.robot_model}")
        print(f"      镜像: {self.teleop_config.mirror_mode}")

        self.teleop = OneroA1Leader(self.teleop_config)

        # 创建 Robot
        print(f"\n[2/2] 创建 Robot (从臂)")
        print(f"      端口: {self.robot_config.port}")
        print(f"      型号: {self.robot_config.robot_model}")

        self.robot = OneroA1SingleArm(self.robot_config)

        # 连接
        print("\n[连接中...]")
        self.teleop.connect()
        self.robot.connect()

        if not self.teleop.is_connected:
            print("[错误] Teleoperator 连接失败")
            return False

        if not self.robot.is_connected:
            print("[错误] Robot 连接失败")
            return False

        print("\n✓ Teleoperator 已连接")
        print("✓ Robot 已连接")
        print()

        return True

    def disconnect(self):
        """断开连接"""
        if self.teleop:
            self.teleop.disconnect()
        if self.robot:
            self.robot.disconnect()
        print("\n✓ 已断开所有连接")

    def control_loop(self):
        """LeRobot 标准遥操作循环"""
        print("[控制循环已启动]")
        print("  Teleoperator (主臂) → 读取位置 → 生成动作")
        print("  Robot (从臂) → 接收动作 → 执行运动")
        print("  提示: 移动主臂，从臂将跟随")
        print("        按 Ctrl+C 停止\n")

        try:
            while self.running:
                # 1. Teleoperator 获取动作
                action = self.teleop.get_action()

                # 2. Robot 执行动作
                self.robot.send_action(action)

                # 控制频率 ~50Hz
                time.sleep(0.02)

        except KeyboardInterrupt:
            print("\n[停止] 用户中断")

    def start(self):
        """启动遥操作控制"""
        self.running = True

        def signal_handler(sig, frame):
            print("\n[停止] 接收到停止信号")
            self.running = False

        signal.signal(signal.SIGINT, signal_handler)
        self.control_loop()

    def run(self):
        """运行遥操作系统"""
        try:
            if not self.connect():
                return 1

            self.start()

        except Exception as e:
            print(f"[错误] {e}")
            import traceback
            traceback.print_exc()
            return 1

        finally:
            self.disconnect()

        return 0


def main():
    """主函数"""
    controller = LeRobotTeleopControl()
    sys.exit(controller.run())


if __name__ == "__main__":
    main()
