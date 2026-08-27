# 主从臂全MIT控制实现

## 修改内容

### 1. 单臂类 `OneroA1SingleArm` 新增功能

#### 新增成员变量：
- `_mit_mode_enabled` - MIT模式启用状态
- `_mit_thread` - MIT控制线程
- `_mit_stop_event` - 线程停止事件
- `_mit_target_q` - MIT目标位置
- `_mit_lock` - 线程锁
- `_mit_time_step` - 控制周期（0.01s = 100Hz）
- `_mit_kp` / `_mit_kd` - MIT控制增益

#### 新增方法：
1. `_start_mit_control()` - 启动MIT控制线程
2. `_stop_mit_control()` - 停止MIT控制
3. `_mit_control_loop()` - MIT控制主循环（100Hz）
4. `set_mit_target(target_positions)` - 设置目标位置
5. `enable_mit_mode(enabled)` - 启用/禁用MIT模式

#### 修改的方法：
- `disconnect()` - 增加MIT线程停止逻辑
- `send_action()` - 增加MIT模式分支

### 2. 双臂类 `OneroA1` 新增功能

#### 新增方法：
1. `_enable_mit_control()` - 配置并启用双MIT模式
   - 主臂：kp=0, kd=0（零力控制）
   - 从臂：高刚度跟踪

#### 修改的方法：
- `connect()` - 增加 `enable_mit_mode` 参数
- `master_slave_follow()` - 增加MIT模式直接更新路径

---

## 使用方法

### 方式一：测试脚本运行

```bash
# 1. 安装包
cd /home/exp/lerobot_robot_onero_a1
pip install -e .

# 2. 快速单臂测试
python3 quick_mit_test.py

# 3. 主从臂全MIT测试
python3 test_mit_control.py

# 4. 单臂MIT测试（调试用）
python3 test_mit_control.py single
```

### 方式二：代码中使用

```python
from lerobot_robot_onero_a1 import OneroA1, OneroA1Config, OneroA1SingleArmConfig

# 配置
config = OneroA1Config(
    master_arm_config=OneroA1SingleArmConfig(port="/dev/ttyACM0", robot_model="a1_l"),
    slave_arm_config=OneroA1SingleArmConfig(port="/dev/ttyACM1", robot_model="a1_r"),
    mirror_mode=True,
)

robot = OneroA1(config)

# 连接并启用MIT模式
robot.connect(calibrate=True, enable_mit_mode=True)

# 主循环：让从臂跟随主臂
while True:
    robot.master_slave_follow()
```

### 方式三：单独启用从臂MIT

```python
from lerobot_robot_onero_a1 import OneroA1SingleArm, OneroA1SingleArmConfig

cfg = OneroA1SingleArmConfig(port="/dev/ttyACM0", robot_model="a1_l")
arm = OneroA1SingleArm(cfg)

arm.connect()
arm.enable_mit_mode(True)  # 启用MIT模式

# 设置目标位置
arm.set_mit_target([0.0] * 7)
```

---

## MIT增益参数调整

### 默认增益（从臂位置跟踪）：
```python
kp = [150.0, 150.0, 150.0, 150.0, 30.0, 30.0, 30.0]
kd = [4.0, 4.0, 4.0, 4.0, 1.0, 1.0, 1.0]
```

### 调整方法：
```python
# 修改从臂增益（更高刚度 = 更快响应但可能震荡）
robot.slave_arm._mit_kp = [200.0] * 7
robot.slave_arm._mit_kd = [5.0] * 7
```

### 增益调参指南：
| 现象 | 解决方案 |
|------|----------|
| 跟随太慢 | 增大kp（如+50） |
| 有震荡/超调 | 减小kp，增大kd |
| 某关节响应差 | 单独调整该关节的kp/kd |

---

## 预期效果

### MIT模式 vs movej模式：

| 指标 | movej模式 | MIT模式 |
|------|-----------|---------|
| 延迟 | ~100ms（轨迹缓冲） | <10ms（直接控制） |
| 平滑度 | 好（插值） | 取决于增益 |
| 响应速度 | 中等 | 快 |
| 跟随精度 | 有轨迹缓冲延迟 | 实时跟随 |

---

## 故障排查

### 问题：手臂不动
- 检查MIT线程是否启动：`arm._mit_mode_enabled`
- 检查目标是否设置：`arm._mit_target_q`
- 查看错误日志

### 问题：手臂震荡
- 降低kp增益
- 增大kd增益
- 检查是否有外部干扰

### 问题：跟随有延迟
- 确认MIT模式已启用
- 检查更新频率是否达到100Hz
- 尝试增大kp增益

---

## 技术细节

### MIT控制循环（100Hz）：
```python
while running:
    # 1. 读取当前状态
    state = arm.get_arm_state_from_motor()

    # 2. 获取目标位置
    target_q = get_target()

    # 3. 计算重力补偿
    tau_g = arm.compute_gravity_torque(state.positions)

    # 4. 发送MIT控制
    arm.control_mit(kp, kd, target_q, [0]*7, tau_g)

    # 5. 保持100Hz
    time.sleep(0.01)
```

### 主从臂数据流：
```
主臂（零力MIT）
    ↓ 100Hz读取位置
主从镜像处理
    ↓ 直接更新目标
从臂（高刚度MIT）
    ↓ 100Hz跟踪
平滑跟随运动
```
