# OneRobotics A1 摄像头接入 + 模仿学习数据录制指南

本文档说明如何让 A1 双臂在「主从遥操 + 零力控制」的基础上接入摄像头，
录制标准 LeRobot 数据集，并用它训练模仿学习策略（以 ACT 为例）。

## 1. 硬件与端口约定

| 设备 | 节点 | 说明 |
| ---- | ---- | ---- |
| 主臂 Leader | `/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_207936AC4343-if00` | 序列号 207936AC4343（当前 ttyACM0），`a1_r`，零力控制（重力补偿），用手拖动 |
| 从臂 Follower | `/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209636A54343-if00` | 序列号 209636A54343（当前 ttyACM1），`a1_l`，MIT 高刚度跟踪从臂 |
| 顶部相机 | `/dev/v4l/by-path/pci-0000:04:00.4-usb-0:1:1.0-video-index0` | 建议俯视工作台 |
| 腕部相机 | `/dev/v4l/by-path/pci-0000:01:00.0-usb-0:2:1.0-video-index0` | 建议装在手爪附近（`video1/3` 是元数据节点，不要使用） |

> 上述路径是内核按 USB 物理端口生成的稳定链接（/dev/v4l/by-path/），只要两只相机固定插在同一 USB 口，重启/拔插后也不会变。

> 注意：两只相机固件里的 USB 序列号相同（CN02KX4NLG0004ABK00），无法用 by-id 区分，只能靠物理 USB 口（by-path）区分，两相机必须固定插在各自的口上，不可互换。

> 实测双摄同时以 MJPG 640×480 采集时，USB 带宽限制为稳定 15fps，
> 因此录制帧率固定使用 `--fps=15`。

## 2. 环境准备

使用 `onero_lerobot` 环境（已装好 lerobot、oneroarm、torch，以及本次补充的
`datasets/pandas/pyarrow/av/jsonlines` 录制依赖）：

```bash
micromamba activate onero_lerobot
```

> 注意：`~/onero_venv` 的 numpy/scipy 版本不兼容，不要用它跑录制。

## 3. 一键录制

```bash
cd ~
python record_onero_camera.py \
    --repo_id your_name/onero_pick \
    --task "把红色积木放进左边的盒子"
```

常用参数（全部参数见 `python record_onero_camera.py --help`）：

```bash
python record_onero_camera.py \
    --repo_id your_name/onero_pick \
    --task "把红色积木放进左边的盒子" \
    --num_episodes 20 \
    --episode_time_s 15 \
    --reset_time_s 5 \
    --root ~/datasets \
    --no-stamp          # 保持 repo_id 不变，不加时间戳
```

- 录制过程中拖动主臂，从臂实时跟随，双摄像头同时录像。
- 每回合结束后有 `reset_time_s` 秒复位环境（不录数据）。
- 按 `Ctrl+C` 可提前结束当前回合。
- 想看实时画面加 `--display-data`（需要 `rerun-sdk`，可 `pip install 'rerun-sdk>=0.24,<0.34'`）。

### 底层命令（等价于上面的脚本）

```bash
lerobot-record \
  --robot.type=onero_a1_single_arm \
  --robot.port=/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_209636A54343-if00 \
  --robot.robot_model=a1_l \
  --robot.control_mode=mit \
  '--robot.cameras={"top": {"type": "opencv", "index_or_path": "/dev/v4l/by-path/pci-0000:04:00.4-usb-0:1:1.0-video-index0", "width": 640, "height": 480, "fps": 15, "fourcc": "MJPG", "backend": "V4L2"}, "wrist": {"type": "opencv", "index_or_path": "/dev/v4l/by-path/pci-0000:01:00.0-usb-0:2:1.0-video-index0", "width": 640, "height": 480, "fps": 15, "fourcc": "MJPG", "backend": "V4L2"}}' \
  --teleop.type=onero_a1_leader \
  --teleop.port=/dev/serial/by-id/usb-Openlight_Labs_CANable2_b158aa7_github.com_normaldotcom_canable2.git_207936AC4343-if00 \
  --teleop.robot_model=a1_r \
  --teleop.zero_force_enabled=true \
  --dataset.repo_id=your_name/onero_pick \
  --dataset.single_task="把红色积木放进左边的盒子" \
  --dataset.num_episodes=20 \
  --dataset.episode_time_s=15 \
  --dataset.fps=15 \
  --dataset.streaming_encoding=true
```

## 4. 录出来的数据

数据集目录结构示例：

```
datasets/your_name_onero_pick/
├── meta/
│   ├── info.json
│   ├── episodes.jsonl
│   └── tasks.jsonl
├── data/
│   └── chunk-000/
│       ├── observation.images.top/
│       │   └── episode_000000.parquet   # 图像帧索引
│       ├── observation.images.wrist/
│       ├── episode_000000.parquet       # 关节状态与动作
│       └── episode_000000.json
└── videos/
    ├── observation.images.top/
    │   └── chunk-000/file-000.mp4
    └── observation.images.wrist/
        └── chunk-000/file-000.mp4
```

特征字段：

- `observation.images.top` / `observation.images.wrist`：两路 RGB 图像视频
- `observation.state`：从臂 7 关节位置（动作由主臂镜像映射到从臂）
- `action`：发送给从臂的关节目标位置

## 5. 训练模仿学习策略（ACT 示例）

```bash
micromamba activate onero_lerobot

lerobot-train \
  --policy.type=act \
  --dataset.repo_id=your_name/onero_pick \
  --dataset.root=~/datasets \
  --output_dir=outputs/train/act_pick \
  --steps=20000 \
  --batch_size=8 \
  --policy.optimizer_lr=1e-5
```

- 训练前建议用 `lerobot-dataset-viz --dataset.repo_id=your_name/onero_pick --dataset.root=~/datasets`
  检查数据质量。
- 训练完成后，在从臂上部署推理用 `lerobot-rollout`（机器人作为 follower，
  相机配置与录制时保持一致）。

## 6. 常见问题

**Q: 提示 `failed to set capture_width` / 相机连接失败？**
A: 相机配置必须显式指定 `backend: V4L2`（本机 OpenCV 在 `ANY` 后端下
`set()` 返回 False，校验会误报）。录制脚本已默认带上。

**Q: 录制时日志提示 record loop 慢于目标 fps？**
A: 检查是否用了 `--fps=15`。双摄同时采集 USB 带宽只有 15fps，帧率设高了必然丢帧。

**Q: 想只用一个相机？**
A: 例如只用顶部相机：`--camera_wrist=none`。

**Q: 从臂跟手不够顺？**
A: 默认 `--control_mode=mit`（高刚度位置跟踪）。如果想对比，可改
`--control_mode=movej`。MIT 增益在
`lerobot/src/lerobot/robots/onero_a1/onero_a1.py` 的 `_mit_kp/_mit_kd` 中调节。

**Q: 零力手感太重/太轻？**
A: 调节 `lerobot_teleop_onero_a1` 的 `OneroA1LeaderConfig.zero_force_kp/kd`
（默认 5.0 / 0.5），或把主臂直接设为 kp=kd=0 的纯零刚度（参考
`lerobot/master_slave_mit_teleop.py`）。

## 7. 代码改动清单

- `lerobot/src/lerobot/robots/onero_a1/config_onero_a1.py`：新增 `cameras`、`control_mode` 字段
- `lerobot/src/lerobot/robots/onero_a1/onero_a1.py`：单臂/双臂相机连接、读取、断开，
  观测特征包含图像，`action_features` 不再包含相机键
- `lerobot/src/lerobot/scripts/lerobot_record.py`：注册 onero 机器人/遥操作类型
- `lerobot_robot_onero_a1/`：同步相同改动
- `record_onero_camera.py`：一键录制脚本（本文档第 3 节）
- `lerobot_teleop_example.py`：修正主臂导入路径（`lerobot.teleoperators.onero_a1_leader`）并把端口对调为当前主从映射
