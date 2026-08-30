# 部署（模型推理）

训练好的 ACT 模型部署到从臂执行，推理在另一台 GPU 电脑（B）上完成，本机（A）负责读观测、控制从臂。

## 架构

```
本机 A（接机械臂+相机）              B 电脑（GPU 推理）
读从臂关节+双摄 ──观测(JSON+JPEG)──> 加载模型，输出动作
执行动作(movej) <──动作(7关节)─────  TCP 8000
```

## 文件

- `infer_server.py`：推理服务，运行在 B 电脑（需 GPU、torch、lerobot、模型 checkpoint）
- `run_inference.py`：部署循环，运行在 A 电脑（需机械臂、相机、lerobot）

## 使用

1. **B 电脑**：把模型 checkpoint 放到 `~/models/pretrained_model`，启动服务：

```bash
~/venv312/bin/python /home/am430/infer_server.py
```

2. **A 电脑**：确认从臂与双摄就位、灯光与录制时一致，运行：

```bash
micromamba run -n onero_lerobot python run_inference.py
```

3. 停止：Ctrl+C 后执行 `disable_both_motors.py` 关闭电机。

## 注意事项

- 部署时的灯光、相机位置、桌面布局、从臂起始姿态必须尽量与录制时一致，否则模型可能输出异常动作；
- `run_inference.py` 内置起始姿态（`START_POSE`）和每步限幅（`MAX_DELTA=0.1`），如需调整直接改文件头部常量；
- 推理服务端口 8000，A/B 需同一局域网互通。
