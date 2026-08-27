#!/usr/bin/env python
"""
快速MIT测试脚本 - 最小化版本

用于验证MIT控制是否正常工作
"""

import time
import sys

# 添加路径
sys.path.insert(0, '/home/exp/lerobot_robot_onero_a1/src')

try:
    from lerobot_robot_onero_a1.onero_a1.onero_a1 import OneroA1SingleArm
    from lerobot_robot_onero_a1.onero_a1.config_onero_a1 import OneroA1SingleArmConfig
except ImportError as e:
    print(f"导入错误: {e}")
    print("请确保已安装: cd /home/exp/lerobot_robot_onero_a1 && pip install -e .")
    sys.exit(1)

def test_mit_basics():
    """测试基本MIT功能"""
    print("=== MIT控制快速测试 ===\n")

    # 配置
    cfg = OneroA1SingleArmConfig(
        port="/dev/ttyACM0",
        robot_model="a1_l",
        enable_safety=False,  # 测试时禁用安全限制
    )

    arm = None
    try:
        # 1. 创建手臂实例
        print("1. 创建手臂实例...")
        arm = OneroA1SingleArm(cfg)

        # 2. 连接
        print("2. 连接手臂...")
        arm.connect(calibrate=True)
        print("   ✓ 连接成功")

        # 3. 获取当前位置
        print("3. 读取当前位置...")
        obs = arm.get_observation()
        current_pos = [obs[f"joint{i}.pos"] for i in range(1, 8)]
        print(f"   当前位置: {[f'{p:.2f}' for p in current_pos]}")

        # 4. 启用MIT模式
        print("4. 启用MIT控制...")
        arm.enable_mit_mode(True)
        print("   ✓ MIT模式已启动")

        # 5. 设置目标位置
        print("5. 设置测试目标位置...")
        test_pos = [0.0] * 7
        arm.set_mit_target(test_pos)
        print(f"   目标位置: {[f'{p:.2f}' for p in test_pos]}")

        # 6. 等待执行
        print("6. 等待5秒观察手臂运动...")
        time.sleep(5)

        # 7. 检查到达位置
        print("7. 检查当前位置...")
        obs = arm.get_observation()
        final_pos = [obs[f"joint{i}.pos"] for i in range(1, 8)]
        print(f"   最终位置: {[f'{p:.2f}' for p in final_pos]}")

        # 8. 计算误差
        errors = [abs(test_pos[i] - final_pos[i]) for i in range(7)]
        max_error = max(errors)
        print(f"   最大误差: {max_error:.3f} rad")

        if max_error < 0.1:
            print("\n✓ MIT控制工作正常！误差在可接受范围内")
        else:
            print("\n⚠ 误差较大，可能需要调整MIT增益参数")

        print("\n按Ctrl+C退出测试...")

        # 保持运行一段时间
        while True:
            time.sleep(1)

    except KeyboardInterrupt:
        print("\n\n用户中断")
    except Exception as e:
        print(f"\n✗ 错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if arm:
            print("\n断开连接...")
            arm.disconnect()
            print("✓ 已断开")

if __name__ == "__main__":
    test_mit_basics()
