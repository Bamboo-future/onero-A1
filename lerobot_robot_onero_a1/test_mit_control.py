#!/usr/bin/env python
"""
测试脚本：主从臂全MIT控制

功能：
1. 主臂：零力MIT控制（可拖动）
2. 从臂：高刚度MIT位置跟踪（跟随主臂）

测试步骤：
1. 连接两个A1臂
2. 启用MIT模式
3. 拖动主臂，观察从臂是否平滑跟随
"""

import logging
import time
from lerobot_robot_onero_a1 import OneroA1, OneroA1Config, OneroA1SingleArmConfig

# 设置日志
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def test_mit_control():
    """测试主从臂MIT控制"""
    # 配置
    config = OneroA1Config(
        master_arm_config=OneroA1SingleArmConfig(
            port="/dev/ttyACM0",
            robot_model="a1_r",
            enable_safety=True,
            max_delta_rad=0.5,  # 每步最大关节变化
        ),
        slave_arm_config=OneroA1SingleArmConfig(
            port="/dev/ttyACM1",
            robot_model="a1_l",
            enable_safety=True,
            max_delta_rad=0.5,
        ),
        mirror_mode=True,  # 左右镜像模式
    )

    robot = None
    try:
        # 创建机器人实例
        robot = OneroA1(config)

        # 连接并启用MIT模式
        logger.info("=== 连接双臂并启用MIT模式 ===")
        robot.connect(calibrate=True, enable_mit_mode=True)

        logger.info("=== MIT模式已启用 ===")
        logger.info("主臂：零力控制（可轻松拖动）")
        logger.info("从臂：位置跟踪模式（跟随主臂）")
        logger.info("\n现在可以拖动主臂，从臂将平滑跟随...")
        logger.info("按Ctrl+C退出\n")

        # 主循环：持续更新从臂目标
        loop_hz = 100  # 100Hz更新频率
        loop_time = 1.0 / loop_hz
        iteration = 0

        while True:
            start_time = time.monotonic()

            # 让从臂跟随主臂
            robot.master_slave_follow()

            iteration += 1
            if iteration % 100 == 0:  # 每秒打印一次状态
                logger.info(f"运行中... 迭代次数: {iteration}")

            # 保持循环频率
            elapsed = time.monotonic() - start_time
            sleep_time = loop_time - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    except KeyboardInterrupt:
        logger.info("\n用户中断，正在断开连接...")
    except Exception as e:
        logger.error(f"发生错误: {e}", exc_info=True)
    finally:
        if robot:
            robot.disconnect()
            logger.info("已断开连接")

def test_single_arm_mit():
    """测试单臂MIT控制（用于调试）"""
    from lerobot_robot_onero_a1 import OneroA1SingleArm, OneroA1SingleArmConfig

    # 测试单臂MIT控制
    config = OneroA1SingleArmConfig(
        port="/dev/ttyACM0",
        robot_model="a1_l",
    )

    arm = None
    try:
        from lerobot_robot_onero_a1.onero_a1.onero_a1 import OneroA1SingleArm

        arm = OneroA1SingleArm(config)

        logger.info("=== 连接单臂 ===")
        arm.connect(calibrate=True)

        logger.info("=== 启用MIT位置跟踪 ===")
        arm.enable_mit_mode(True)

        # 设置目标位置（测试用）
        test_positions = [0.0, -0.5, 0.0, 1.5, 0.0, 0.5, 0.0]
        arm.set_mit_target(test_positions)

        logger.info("目标位置已设置，手臂应该移动到目标位置")
        logger.info("按Ctrl+C退出")

        time.sleep(10)  # 保持10秒

    except Exception as e:
        logger.error(f"错误: {e}", exc_info=True)
    finally:
        if arm:
            arm.disconnect()

if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "single":
        logger.info("=== 单臂MIT测试模式 ===")
        test_single_arm_mit()
    else:
        logger.info("=== 主从臂全MIT测试 ===")
        test_mit_control()
