#!/usr/bin/env python3
"""禁用主臂和从臂电机"""

import oneroarm

print("禁用主臂和从臂电机...")

# 主臂
cfg_m = oneroarm.OneroConfig()
cfg_m.device = "/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_207936AC4343-if00"
cfg_m.robot_model = "a1_r"
cfg_m.version = "A1"
cfg_m.mount_orientation = "vertical"

try:
    print("\n[主臂] 连接中...")
    master = oneroarm.OneroArm(cfg_m)
    master.disable_motors()
    print("✓ 主臂电机已禁用")
except Exception as e:
    print(f"✗ 主臂: {e}")

# 从臂
cfg_s = oneroarm.OneroConfig()
cfg_s.device = "/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209636A54343-if00"
cfg_s.robot_model = "a1_l"
cfg_s.version = "A1"
cfg_s.mount_orientation = "vertical"

try:
    print("\n[从臂] 连接中...")
    slave = oneroarm.OneroArm(cfg_s)
    slave.disable_motors()
    print("✓ 从臂电机已禁用")
except Exception as e:
    print(f"✗ 从臂: {e}")

print("\n完成！")
