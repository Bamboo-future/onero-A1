# Onero A1 LeRobot 插件（主从遥操 + 零力控制 + 录制训练）

基于 LeRobot 的 OneRobotics A1 双臂主从遥操与数据录制方案：
主臂零力（重力补偿）拖动，从臂 MIT 高刚度实时跟随，双摄像头同步录制数据集，用于 ACT 等模仿学习训练。

## 硬件与端口映射（重要）

| 设备 | 端口 | 型号 | USB 序列号 |
|---|---|---|---|
| 主臂 Leader | /dev/ttyACM0 | a1_r | 207936AC4343 |
| 从臂 Follower | /dev/ttyACM1 | a1_l | 209636A54343 |
| 顶部相机 | /dev/v4l/by-path/pci-0000:04:00.4-usb-0:1:1.0-video-index0 | - | - |
| 腕部相机 | /dev/v4l/by-path/pci-0000:01:00.0-usb-0:2:1.0-video-index0 | - | - |

> 两台相机 USB 序列号相同，只能靠物理 USB 口（by-path）区分，不要互换插口。

## 依赖安装

1. `oneroarm` SDK：专有库，请从 OneRobotics 厂商渠道安装（本仓库不包含）。
2. LeRobot：clone [huggingface/lerobot](https://github.com/huggingface/lerobot) 后应用本仓库补丁（见下）。
3. 插件包：

```bash
pip install -e lerobot_robot_onero_a1
pip install -e lerobot_teleop_onero_a1
```

## 对上游 lerobot 的修改

```bash
cd /path/to/lerobot
git apply /path/to/onero-A1/lerobot_patch/lerobot_upstream_changes.diff
# 然后复制新增模块：
cp -r /path/to/onero-A1/lerobot_patch/src/lerobot/robots/onero_a1 src/lerobot/robots/
cp -r /path/to/onero-A1/lerobot_patch/src/lerobot/teleoperators/onero_a1_leader src/lerobot/teleoperators/
```

`lerobot_patch/src/` 下的模块即运行时使用的适配器（含串口并发修复）。

## 使用

### 主从遥操（手动测试）

```bash
micromamba run -n onero_lerobot python3 master_slave_mit_teleop.py
```

### 录制数据集（主从遥操 + 双摄）

```bash
micromamba run -n onero_lerobot python record_onero_camera.py \
    --repo_id your_name/onero_pick \
    --task "把红色积木放进左边的盒子" \
    --num_episodes 10 \
    --no-push
```

### 训练（ACT 示例）

```bash
lerobot-train \
  --policy.type=act \
  --dataset.repo_id=your_name/onero_pick \
  --dataset.root=~/datasets \
  --output_dir=outputs/train/act_pick \
  --steps=20000 \
  --batch_size=8
```

### 紧急关电机

```bash
python disable_both_motors.py
```

## 安全提示

- `enable_motors()` 使能时会自动回零位，运行前请把机械臂放在接近零位的安全位置。
- `disable_motors()` 会使机械臂自由下落，失能前确保臂处于安全位置。
- 录制时不要让回合中途连续 Ctrl+C，避免数据未落盘；异常退出后可用 `lerobot-dataset-viz` 检查数据。

## 许可证

部分代码源自 LeRobot（Apache-2.0），相关文件保留原版权头。`oneroarm` 为 OneRobotics 专有库，不在本仓库内。

## 部署（模型推理）

训练好的模型部署到从臂自主执行，推理可在另一台 GPU 电脑完成。详见 [deployment/README.md](deployment/README.md)。

```bash
# B 电脑：启动推理服务
python deployment/infer_server.py

# A 电脑：启动部署循环（需机械臂+双摄，灯光/场景与录制一致）
micromamba run -n onero_lerobot python deployment/run_inference.py
```
