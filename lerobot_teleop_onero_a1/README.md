# LeRobot Teleoperator Plugin for OneRobotics A1

This package provides a LeRobot teleoperator that uses an OneRobotics A1 arm as the leader device for teleoperation.

## Installation

```bash
cd /home/exp/lerobot_teleop_onero_a1
pip install -e .
```

## Usage with LeRobot CLI

### Record Episode with Leader Arm

```bash
micromamba activate onero_lerobot

lerobot-record \
  --robot.type=generic \
  --teleop.type=onero_a1_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.robot_model=a1_l \
  --teleop.mirror_mode=true \
  --dataset.repo_id=your_name/onero_a1_dataset \
  --dataset.task="pick and place"
```

### Teleoperate with Leader Arm

```bash
lerobot-teleoperate \
  --robot.type=generic \
  --teleop.type=onero_a1_leader \
  --teleop.port=/dev/ttyACM0 \
  --teleop.robot_model=a1_l \
  --teleop.mirror_mode=true
```

## Configuration Parameters

- `port`: Serial port (default: "/dev/ttyACM0")
- `robot_model`: "a1_l" or "a1_r" (default: "a1_l")
- `mirror_mode`: Mirror joints for left-right setup (default: true)

## Hardware Setup

The leader arm should be connected to `/dev/ttyACM0` (or configured port).
