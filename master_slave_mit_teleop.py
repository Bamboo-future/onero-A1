#!/usr/bin/env python3
"""
主从臂全MIT控制：主臂零力控制 + 从臂MIT高刚度跟踪

功能：
- 主臂 (ACM0): MIT零力控制，可轻松拖动
- 从臂 (ACM1): MIT高刚度位置跟踪，实时跟随主臂

对应硬件：
- ACM0 = Leader (主臂，零力控制)
- ACM1 = Follower (从臂，跟随主臂)
"""

import sys
import time
import logging
import threading

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

def main():
    try:
        from lerobot.robots.onero_a1.onero_a1 import OneroA1SingleArm
        from lerobot.robots.onero_a1.config_onero_a1 import OneroA1SingleArmConfig

        logger.info("=" * 60)
        logger.info("主从臂全MIT控制")
        logger.info("=" * 60)
        logger.info("主臂 (ACM0): 零力控制 - 可轻松拖动")
        logger.info("从臂 (ACM1): MIT位置跟踪 - 实时跟随")
        logger.info("=" * 60)

        # 主臂配置 - 零力控制
        master_cfg = OneroA1SingleArmConfig(
            port="/dev/ttyACM0",
            robot_model="a1_r",
            enable_safety=False,  # 零力模式不需要安全限制
        )

        # 从臂配置 - 高刚度跟踪
        slave_cfg = OneroA1SingleArmConfig(
            port="/dev/ttyACM1",
            robot_model="a1_l",
            enable_safety=True,
            max_delta_rad=0.5,
        )

        master_arm = None
        slave_arm = None

        try:
            # 连接主臂
            logger.info("\n[1/4] 连接主臂...")
            master_arm = OneroA1SingleArm(master_cfg)
            master_arm.connect(calibrate=True)

            # 设置主臂为零力模式增益
            master_arm._mit_kp = [0.0] * 7  # 零刚度
            master_arm._mit_kd = [0.0] * 7  # 零阻尼

            # 连接从臂
            logger.info("[2/4] 连接从臂...")
            slave_arm = OneroA1SingleArm(slave_cfg)
            slave_arm.connect(calibrate=True)

            # 设置从臂为高刚度跟踪增益
            slave_arm._mit_kp = [150.0, 150.0, 150.0, 150.0, 30.0, 30.0, 30.0]
            slave_arm._mit_kd = [4.0, 4.0, 4.0, 4.0, 1.0, 1.0, 1.0]

            # 启用MIT控制
            logger.info("[3/4] 启用MIT控制...")
            master_arm.enable_mit_mode(True)
            slave_arm.enable_mit_mode(True)

            logger.info("✓ MIT控制已启用")
            logger.info("  - 主臂: 零力模式 (可拖动)")
            logger.info("  - 从臂: 位置跟踪模式")

            # 主控制循环
            logger.info("[4/4] 开始主从控制循环 (100Hz)")
            logger.info("\n现在可以拖动主臂，从臂将实时跟随...")
            logger.info("按 Ctrl+C 退出\n")

            loop_hz = 100
            loop_time = 1.0 / loop_hz
            iteration = 0

            while True:
                start_time = time.monotonic()

                # 读取主臂位置
                master_obs = master_arm.get_observation()
                master_pos = [master_obs[f"joint{i}.pos"] for i in range(1, 8)]

                # 镜像处理（左右臂对称）
                # 关节 2,4,6 (索引1,3,5) 需要镜像
                slave_pos = list(master_pos)
                slave_pos[1] = -slave_pos[1]  # joint2
                slave_pos[3] = -slave_pos[3]  # joint4
                slave_pos[5] = -slave_pos[5]  # joint6

                # 更新从臂目标
                slave_arm.set_mit_target(slave_pos)

                # 状态打印
                iteration += 1
                if iteration % 100 == 0:  # 每秒打印一次
                    logger.info(f"运行中... 迭代: {iteration}, 主臂位置: {[f'{p:.2f}' for p in master_pos[:3]]}...")

                # 保持循环频率
                elapsed = time.monotonic() - start_time
                sleep_time = loop_time - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            logger.info("\n用户中断，正在退出...")
        except Exception as e:
            logger.error(f"发生错误: {e}", exc_info=True)
        finally:
            logger.info("断开连接...")
            if master_arm:
                master_arm.disconnect()
            if slave_arm:
                slave_arm.disconnect()
            logger.info("✓ 已断开")

    except ImportError as e:
        logger.error(f"导入错误: {e}")
        logger.error("请确保在 onero_lerobot 环境中运行")
        sys.exit(1)

if __name__ == "__main__":
    main()
