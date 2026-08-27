#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
OneRobotics A1 双臂 + 摄像头 模仿学习数据录制脚本
====================================================

基于 LeRobot 官方 lerobot-record，封装了本机 A1 双臂与摄像头的固定参数：

  - 主臂 (Leader)  : /dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_207936AC4343-if00, a1_r, 零力控制（重力补偿，可轻松拖动）
  - 从臂 (Follower): /dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209636A54343-if00, a1_l, MIT 高刚度跟踪（默认）或 movej
  - 相机 top       : /dev/v4l/by-path/pci-0000:04:00.4-usb-0:1:1.0-video-index0, 640x480@15fps MJPG (V4L2)
  - 相机 wrist     : /dev/v4l/by-path/pci-0000:01:00.0-usb-0:2:1.0-video-index0, 640x480@15fps MJPG (V4L2)

录制结果是一个标准 LeRobot 数据集（含 observation.images.top/wrist、
observation.state 与 action），可直接用于 lerobot-train 模仿学习。

环境要求（务必使用 onero_lerobot 环境）:
    micromamba activate onero_lerobot

用法示例:
    python record_onero_camera.py \
        --repo_id your_name/onero_pick \
        --task "把红色积木放进左边的盒子"

查看全部参数:
    python record_onero_camera.py --help

只打印命令不执行:
    python record_onero_camera.py --repo_id x/onero --task "test" --dry-run
"""

import argparse
import os
import json
import shlex
import subprocess
import sys

DEFAULT_MASTER_PORT = "/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_207936AC4343-if00"  # 主臂（序列号 207936AC4343）
DEFAULT_SLAVE_PORT = "/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209636A54343-if00"  # 从臂（序列号 209636A54343）
DEFAULT_CAMERA_TOP = "/dev/v4l/by-path/pci-0000:04:00.4-usb-0:1:1.0-video-index0"
DEFAULT_CAMERA_WRIST = "/dev/v4l/by-path/pci-0000:01:00.0-usb-0:2:1.0-video-index0"
MEASURED_SUSTAINABLE_FPS = 15  # 双摄同时 MJPG 640x480 实测稳定帧率


def check_env() -> None:
    """检查当前 Python 环境是否具备录制所需依赖。"""
    try:
        import lerobot  # noqa: F401
    except ImportError:
        print("[错误] 当前环境找不到 lerobot，请先激活 onero_lerobot 环境：")
        print("    micromamba activate onero_lerobot")
        sys.exit(1)
    try:
        import datasets  # noqa: F401
    except ImportError:
        print("[错误] 缺少数据集录制依赖，请执行：")
        print("    micromamba activate onero_lerobot")
        print("    pip install 'datasets>=4.8.0,<5.0.0' 'pandas>=2.0.0,<3.0.0' "
              "'pyarrow>=21.0.0,<30.0.0' 'av>=15.0.0,<16.0.0' 'jsonlines>=4.0.0,<5.0.0'")
        sys.exit(1)


def check_devices(master_port: str, slave_port: str, camera_top: str, camera_wrist: str) -> None:
    """检查主臂/从臂/相机的稳定设备路径是否存在，缺失时给出明确提示。"""
    required = {}
    if master_port:
        required["主臂"] = master_port
    if slave_port:
        required["从臂"] = slave_port
    if camera_top.lower() != "none":
        required["顶部相机"] = camera_top
    if camera_wrist.lower() != "none":
        required["腕部相机"] = camera_wrist

    missing = [name for name, path in required.items() if not os.path.exists(path)]
    if missing:
        print("[错误] 以下设备未检测到：" + "、".join(missing))
        print("请检查设备连接。当前系统中存在的稳定链接：")
        for base, label in (("/dev/serial/by-id", "串口"), ("/dev/v4l/by-path", "相机"), ("/dev/v4l/by-id", "相机by-id")):
            if os.path.isdir(base):
                for f in sorted(os.listdir(base)):
                    print(f"  {label}: {f}")
        sys.exit(1)


def build_camera_config(path: str, width: int, height: int, fps: int) -> dict:
    """构造单个 OpenCV 相机的 LeRobot 配置字典。"""
    return {
        "type": "opencv",
        "index_or_path": path,
        "width": width,
        "height": height,
        "fps": fps,
        "fourcc": "MJPG",
        "backend": "V4L2",  # 必须显式指定，否则本机 OpenCV set() 校验会失败
    }


def main() -> int:
    parser = argparse.ArgumentParser(
        description="OneRobotics A1 主从遥操 + 零力控制 + 摄像头数据录制",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--repo_id", required=True,
                        help="数据集标识，建议 {用户名}/{数据集名}，例如 your_name/onero_pick")
    parser.add_argument("--task", required=True, help="任务描述，例如 把红色积木放进左边的盒子")
    parser.add_argument("--num_episodes", type=int, default=10, help="录制回合数")
    parser.add_argument("--episode_time_s", type=float, default=15.0, help="每回合录制秒数")
    parser.add_argument("--reset_time_s", type=float, default=5.0, help="每回合之间复位环境的秒数")
    parser.add_argument("--fps", type=int, default=MEASURED_SUSTAINABLE_FPS,
                        help="录制帧率（双摄实测最高稳定 15fps）")
    parser.add_argument("--root", default=None,
                        help="数据集保存根目录（默认 $HF_LEROBOT_HOME/数据集名）")
    parser.add_argument("--master_port", default=DEFAULT_MASTER_PORT, help="主臂串口")
    parser.add_argument("--slave_port", default=DEFAULT_SLAVE_PORT, help="从臂串口")
    parser.add_argument("--master_model", default="a1_r", help="主臂型号")
    parser.add_argument("--slave_model", default="a1_l", help="从臂型号")
    parser.add_argument("--camera_top", default=DEFAULT_CAMERA_TOP,
                        help="顶部相机设备（传 none 禁用）")
    parser.add_argument("--camera_wrist", default=DEFAULT_CAMERA_WRIST,
                        help="腕部相机设备（传 none 禁用）")
    parser.add_argument("--camera_width", type=int, default=640, help="相机宽度")
    parser.add_argument("--camera_height", type=int, default=480, help="相机高度")
    parser.add_argument("--control_mode", choices=["mit", "movej"], default="mit",
                        help="从臂控制模式：mit=高刚度位置跟踪，movej=SDK 轨迹模式")
    parser.add_argument("--no-stamp", action="store_true",
                        help="不自动给 repo_id 追加时间戳")
    parser.add_argument("--no-push", action="store_true",
                        help="录制完成后不上传数据集到 Hugging Face Hub（默认会上传）")
    parser.add_argument("--display-data", action="store_true",
                        help="录制时用 Rerun 实时显示图像")
    parser.add_argument("--no-streaming", action="store_true",
                        help="关闭流式视频编码（默认开启，节省磁盘且回合保存更快）")
    parser.add_argument("--dry-run", action="store_true",
                        help="只打印将执行的命令，不真正录制")
    args = parser.parse_args()

    if args.root and os.path.isdir(os.path.expanduser(args.root)):
        print(f"[错误] --root 目录已存在：{args.root}")
        print("这个版本的 LeRobot 把 --root 当作数据集目录本身，且要求该目录必须不存在（不会复用或覆盖）。")
        print("请换一个新的目录路径重新运行；如果旧目录是录制中途崩溃留下的残缺数据，")
        print("确认没有价值后可以删除它，再使用原路径。")
        return 1

    if args.fps > MEASURED_SUSTAINABLE_FPS:
        print(f"[警告] 双摄同时采集实测只能稳定 {MEASURED_SUSTAINABLE_FPS}fps，"
              f"设置 {args.fps}fps 会导致丢帧，建议使用 --fps={MEASURED_SUSTAINABLE_FPS}")

    check_devices(args.master_port, args.slave_port, args.camera_top, args.camera_wrist)

    if not args.dry_run:
        check_env()

    cameras = {}
    for cam_key, cam_path in (("top", args.camera_top), ("wrist", args.camera_wrist)):
        if cam_path.lower() == "none":
            print(f"[信息] 禁用相机 {cam_key}")
            continue
        cameras[cam_key] = build_camera_config(cam_path, args.camera_width, args.camera_height, args.fps)
    if not cameras:
        print("[错误] 至少需要启用一个相机")
        return 1

    cmd = [
        sys.executable, "-m", "lerobot.scripts.lerobot_record",
        "--robot.type=onero_a1_single_arm",
        f"--robot.port={args.slave_port}",
        f"--robot.robot_model={args.slave_model}",
        f"--robot.control_mode={args.control_mode}",
        f"--robot.cameras={json.dumps(cameras)}",
        "--teleop.type=onero_a1_leader",
        f"--teleop.port={args.master_port}",
        f"--teleop.robot_model={args.master_model}",
        "--teleop.zero_force_enabled=true",
        f"--dataset.repo_id={args.repo_id}",
        f"--dataset.single_task={args.task}",
        f"--dataset.num_episodes={args.num_episodes}",
        f"--dataset.episode_time_s={args.episode_time_s}",
        f"--dataset.reset_time_s={args.reset_time_s}",
        f"--dataset.fps={args.fps}",
    ]
    if args.root:
        cmd.append(f"--dataset.root={args.root}")
    if args.no_stamp:
        cmd.append("--dataset.no_stamp=true")
    if args.no_push:
        cmd.append("--dataset.push_to_hub=false")
    if not args.no_streaming:
        cmd.append("--dataset.streaming_encoding=true")
    if args.display_data:
        cmd.append("--display_data=true")

    print("=" * 70)
    print("  执行命令：")
    print("=" * 70)
    print(shlex.join(cmd))
    print("=" * 70)
    print("  录制开始后，请拖动主臂执行任务；按 Ctrl+C 可提前结束。")
    print("=" * 70)

    if args.dry_run:
        return 0

    try:
        subprocess.run(cmd, check=True)
    except subprocess.CalledProcessError as e:
        print(f"[错误] 录制失败，退出码 {e.returncode}")
        return e.returncode
    return 0


if __name__ == "__main__":
    sys.exit(main())
