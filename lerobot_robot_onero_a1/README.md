# LeRobot Plugin for OneRobotics A1

This package provides LeRobot integration for the OneRobotics A1 series robot arms.

## Installation

```bash
cd /home/exp/lerobot_robot_onero_a1
pip install -e .
```

## Usage

### Single Arm

```python
from lerobot_robot_onero_a1 import OneroA1SingleArm, OneroA1SingleArmConfig

config = OneroA1SingleArmConfig(
    port="/dev/ttyACM0",
    robot_model="a1_l",
)
arm = OneroA1SingleArm(config)
arm.connect()
obs = arm.get_observation()
action = arm.send_action(obs)
arm.disconnect()
```

### Bimanual Master-Slave

```python
from lerobot_robot_onero_a1 import OneroA1, OneroA1Config

config = OneroA1Config(
    master_arm_config=OneroA1SingleArmConfig(
        port="/dev/ttyACM0",
        robot_model="a1_l",
    ),
    slave_arm_config=OneroA1SingleArmConfig(
        port="/dev/ttyACM1",
        robot_model="a1_r",
    ),
    mirror_mode=True,
)
robot = OneroA1(config)
robot.connect()

# Make slave follow master
sent_action = robot.master_slave_follow()

# Or control independently
obs = robot.get_observation()
action = {"master_joint1.pos": 0.5, ...}
robot.send_action(action)

robot.disconnect()
```

## Hardware Setup

- Connect master arm to `/dev/ttyACM0`
- Connect slave arm to `/dev/ttyACM1`
- Install oneroarm package from `/home/exp/Downloads/ArmApi-main/python/conda_channel/linux`
