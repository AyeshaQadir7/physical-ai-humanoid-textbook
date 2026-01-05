---
id: hw-unitree-deployment
title: "Optional: Unitree G1 Humanoid Robot"
sidebar_position: 3
sidebar_label: "Unitree G1 Deployment"
description: "Reference for deploying capstone project to Unitree G1 humanoid robot platform"
keywords: [Unitree, humanoid, robot, deployment, G1, physical robot]
---

## Overview

This chapter describes the **optional** path for deploying the capstone project to a real humanoid robot platform: the **Unitree G1**. This is **not required** for course completion—all learning objectives are achievable through simulation.

**Key Point**: This chapter is provided for:
- Research institutions with Unitree access
- Students in advanced robotics labs
- Capstone projects extending to physical validation

---

## Unitree G1 Specifications

### Physical Dimensions

| Property | Specification |
| --- | --- |
| **Height** | 158 cm (5'2") |
| **Weight** | 40 kg (88 lbs) |
| **Degrees of Freedom** | 23 DOF (7 per arm, 6 per leg, 2 head/torso) |
| **Leg Design** | Bipedal with passive compliance |
| **Arm Design** | 7-DOF anthropomorphic arms |
| **Workspace** | 5m radius operational area |
| **Max Velocity** | 1.0 m/s walking, 2.0 m/s running (experimental) |

### Sensors

| Sensor Type | Model | Purpose |
| --- | --- | --- |
| **IMU** | 6-axis in torso | Balance, orientation estimation |
| **Joint Encoders** | On all actuators | Position feedback |
| **Force Sensors** | Under feet | Gait control, terrain detection |
| **Cameras** | 2× RGB cameras | Vision-based perception |
| **Microphone** | Onboard array | Voice input (optional upgrade) |

### Control Interface

| Interface | Protocol | Latency |
| --- | --- | --- |
| **Ethernet** | TCP/IP, UDP | under 10 ms |
| **ROS 2 Bridge** | Native plugin | under 5 ms |
| **SDK** | C++, Python | Proprietary API |

---

## Hardware Setup (Unitree G1)

### Prerequisites

- ✅ Access to Unitree G1 (institution/lab ownership)
- ✅ Safety certification and clearance
- ✅ Onsite supervisor trained in robot operation
- ✅ Insurance and liability coverage
- ✅ Designated testing space (5m×5m minimum)

### Communication Stack

**Option 1: ROS 2 Native** (Recommended)

```bash
# Unitree provides ROS 2 bridge
git clone https://github.com/unitreerobotics/unitree_ros2.git ~/unitree_ros2_ws/src

cd ~/unitree_ros2_ws
colcon build

# Launch robot driver
ros2 launch unitree_ros2_interface unitree_g1.launch.py
```

**Topics Published**:
```
/unitree/state/imu             (sensor_msgs/Imu)
/unitree/state/joint_states    (sensor_msgs/JointState)
/unitree/state/foot_contact    (unitree_msgs/FootContact)
/unitree/vision/camera_left    (sensor_msgs/Image)
/unitree/vision/camera_right   (sensor_msgs/Image)
```

**Topics Subscribed**:
```
/unitree/command/joint_command (unitree_msgs/JointCommand)
/unitree/command/motion        (unitree_msgs/MotionCommand)
/unitree/command/mode          (std_msgs/String)  # "walk", "stand", "sit"
```

**Option 2: Unitree SDK** (Direct Control)

```python
# Proprietary Unitree API
from unitree_sdk import QuadrupedRobot  # Note: G1 API structure

robot = QuadrupedRobot()
robot.init()

# Stand up
robot.motion_cmd.mode = "stand"
robot.send_command()

# Walk forward
robot.motion_cmd.velocity = [0.5, 0, 0]  # m/s in x, y, yaw
robot.send_command()
```

---

## Deployment Workflow

### Phase 1: Simulation Validation (Weeks 1–12)

1. **Develop and test** entire capstone project in Isaac Sim with humanoid model
2. **Validate control algorithms** with simulated G1 dynamics
3. **Record behavior**: Screenshot/video of simulation capstone

### Phase 2: Real Robot Integration (Week 13, Post-Capstone)

:::warning
**Timeline**: Physical robot deployment extends beyond 13-week course
**Recommendation**: Plan for 4-6 weeks post-course for physical integration
:::

1. **Install ROS 2 bridge** on Unitree compute module
2. **Adapt capstone code** to real robot:
   - Replace simulated `/unitree/state` with real sensor data
   - Calibrate motion commands (simulated vs. real acceleration)
   - Implement safety constraints

3. **Test incrementally**:
   - Week 1: Stationary perception (cameras, microphone)
   - Week 2: Simple motion (walk forward, turn)
   - Week 3: Voice command recognition
   - Week 4: Integrated voice-controlled movement
   - Week 5–6: Full capstone behavior with safety validation

---

## Safety Protocols (Critical for Physical Robot)

### Pre-Operation Checklist

- [ ] **Space**: Testing area is clear of people/objects for 5m radius
- [ ] **Supervision**: Trained operator present and monitoring
- [ ] **E-Stop**: Emergency stop button functional and accessible
- [ ] **Battery**: Fully charged, under 30 min runtime budgeted
- [ ] **Network**: Ethernet or WiFi connection stable
- [ ] **Tether** (optional): Safety line attached if indoors
- [ ] **PPE**: Operator wearing appropriate safety gear

### Operational Constraints

**Hard Limits** (enforced in software):
```python
# Maximum velocity constraints
max_linear_velocity = 0.3  # m/s (reduced for safety)
max_angular_velocity = 0.2  # rad/s

# Joint torque limits
joint_torque_limits = [50, 50, 30, 30, 20, 20, 20]  # Nm per joint

# Emergency stop condition
if obstacle_detected or operator_presses_estop:
    disable_all_motors()
    engage_brakes()
    log_incident()
```

**Operational Zones**:
- **Green Zone**: 1–3 m radius → Full autonomy enabled
- **Yellow Zone**: 3–5 m radius → Reduced speed (0.2 m/s max)
- **Red Zone**: >5 m or obstacles detected → Stop immediately

### Voice Command Whitelist (VLA Safety)

For capstone with voice control, restrict commands to safe subset:

```python
SAFE_COMMANDS = {
    "stand": GoalStateStand(),
    "sit": GoalStateSit(),
    "walk forward": MotionCommand(velocity=[0.2, 0, 0]),
    "walk backward": MotionCommand(velocity=[-0.1, 0, 0]),
    "turn left": MotionCommand(velocity=[0, 0, 0.15]),
    "turn right": MotionCommand(velocity=[0, 0, -0.15]),
    "stop": MotionCommand(velocity=[0, 0, 0]),
    # Explicitly disallow:
    # "run", "jump", "spin", etc.
}

def parse_voice_command(transcribed_text):
    """Only allow whitelisted commands."""
    for safe_cmd, robot_action in SAFE_COMMANDS.items():
        if safe_cmd in transcribed_text.lower():
            return robot_action
    # Unknown command → stop
    return SAFE_COMMANDS["stop"]
```

### Emergency Response Procedures

| Scenario | Response |
| --- | --- |
| **Robot losing balance** | Operator hits E-stop; robot lowers to ground safely |
| **Network disconnection** | Robot halts all motion; waits for reconnection (timeout: 2 sec) |
| **Unexpected person in zone** | Autonomous stop; operator takes manual control |
| **Motor failure** | Log error; switch to sit mode; await manual inspection |
| **Battery low** | Autonomous return to start position; graceful shutdown |

---

## Code Integration: Simulation ↔ Real Robot

### Abstraction Layer Pattern

To easily switch between simulation and real robot:

```python
"""
RobotController abstraction supporting both sim and real hardware
"""

from abc import ABC, abstractmethod
import rclpy
from geometry_msgs.msg import Twist
from unitree_msgs.msg import JointCommand

class RobotController(ABC):
    """Abstract robot interface."""

    @abstractmethod
    def move(self, velocity: Twist):
        """Move robot with given velocity."""
        pass

    @abstractmethod
    def get_imu(self):
        """Read IMU data."""
        pass

    @abstractmethod
    def get_joint_states(self):
        """Read joint states."""
        pass

class SimulationRobotController(RobotController):
    """Control robot in Gazebo/Isaac Sim."""

    def __init__(self, node):
        self.node = node
        self.cmd_pub = node.create_publisher(Twist, '/cmd_vel', 10)
        self.imu_sub = node.create_subscription(Imu, '/imu', self.imu_callback, 10)
        self.joint_sub = node.create_subscription(JointState, '/joint_states', self.joint_callback, 10)

    def move(self, velocity):
        self.cmd_pub.publish(velocity)

    # ... other methods

class RealRobotController(RobotController):
    """Control real Unitree G1 robot."""

    def __init__(self, node):
        self.node = node
        self.cmd_pub = node.create_publisher(JointCommand, '/unitree/command/joint_command', 10)
        self.imu_sub = node.create_subscription(Imu, '/unitree/state/imu', self.imu_callback, 10)
        self.joint_sub = node.create_subscription(JointState, '/unitree/state/joint_states', self.joint_callback, 10)

    def move(self, velocity):
        """Convert Twist to joint commands for G1."""
        # G1 kinematics mapping here
        joint_cmd = self.inverse_kinematics(velocity)
        self.cmd_pub.publish(joint_cmd)

    # ... other methods

# Usage
def create_robot_controller(use_real_robot: bool, node):
    """Factory function."""
    if use_real_robot:
        return RealRobotController(node)
    else:
        return SimulationRobotController(node)

# In main capstone code
robot = create_robot_controller(use_real_robot=True, node=node)  # or False for sim
robot.move(velocity_cmd)  # Works on both!
```

### Calibration: Simulation vs. Real

**Motor Response Differences**:

| Parameter | Simulation | Real G1 | Correction |
| --- | --- | --- | --- |
| **Acceleration** | Instant | 0.1–0.2 s lag | Add low-pass filter |
| **Max velocity** | Ideal | 0.8× ideal | Scale command by 0.8 |
| **Friction** | Low | High on carpet | Increase motor effort |
| **Latency** | under 1 ms | 10–50 ms | Predictive control |

**Calibration Script**:

```python
def calibrate_motor_response():
    """Measure real robot response and adjust gains."""
    print("Running motor calibration...")

    # Test 1: Acceleration response
    start_time = time.time()
    robot.move(Twist(linear=Point(x=0.5)))  # 50% velocity
    response_time = measure_response_time()

    # Test 2: Max velocity
    robot.move(Twist(linear=Point(x=1.0)))
    real_max_vel = measure_actual_velocity()

    # Test 3: Friction
    forces = robot.get_foot_forces()

    # Compute correction factors
    accel_factor = 0.3 / response_time
    velocity_factor = real_max_vel / 1.0
    friction_factor = sum(forces) / 40.0  # Expected force

    print(f"Correction factors: accel={accel_factor}, velocity={velocity_factor}, friction={friction_factor}")
    return {
        'acceleration': accel_factor,
        'velocity': velocity_factor,
        'friction': friction_factor
    }
```

---

## Documentation & Reporting

After physical deployment, document:

1. **Differences from Simulation**:
   - Motion latency
   - Balance behavior
   - Sensor noise characteristics

2. **Calibration Results**:
   - Final correction factors
   - Video of side-by-side sim/real comparison

3. **Safety Incidents** (if any):
   - What happened
   - How it was resolved
   - Changes made to prevent recurrence

4. **Lessons Learned**:
   - What surprised you
   - Design improvements for next iteration

---

## Cost of Physical Deployment

| Item | Cost | Notes |
| --- | --- | --- |
| **Unitree G1 Robot** | $35,000–50,000 | Institutional purchase; not per-student |
| **Jetson Orin AGX** | $999 | (G1 compute module not student-purchasable) |
| **Safety Equipment** | $500–1,000 | E-stop, barrier, padding |
| **Insurance** | $5,000–10,000/year | Institutional liability |
| **Development Time** | ~4–6 weeks | Post-capstone engineering |

**Total**: ~$40,000–60,000 for institutional research setup

---

## Alternatives to Unitree G1

### Budget Alternatives

| Robot | Cost | Humanoid? | ROS 2 Support | Notes |
| --- | --- | --- | --- | --- |
| **Unitree H1** | $150,000+ | ✅ Yes | ✅ Partial | Higher performance; overkill for learning |
| **Unitree Go2** | $1,500 | ❌ Quadruped | ✅ Yes | Good for sim-to-real; not humanoid |
| **Open Humanoid** | $5,000–10,000 | ✅ Yes | ✅ Community | Open-source; requires assembly |
| **NAO (Softbank)** | $6,000–8,000 | ✅ Yes | ✅ Yes | Smaller; fewer DOF |
| **Pepper (Softbank)** | $20,000 | ✅ Partially | ❌ Limited | More social; less manipulation |

### Recommendation

For most institutions, **Unitree G1 is the sweet spot**:
- ✅ Full-size humanoid (realistic movement)
- ✅ Active ROS 2 support
- ✅ Research-grade performance
- ✅ Reasonable cost for group ownership

---

## Next Steps (Post-Capstone)

If pursuing physical robot deployment:

1. **Week 13**: Complete simulation-based capstone
2. **Week 14–16**: Install ROS 2 bridge on G1; validate sensor data
3. **Week 17–18**: Adapt capstone code to real robot; implement safety
4. **Week 19–20**: Incremental testing (stationary → walking → voice control)
5. **Final**: Document findings; compare sim vs. real performance

**Estimated Post-Course Time**: 4–6 weeks for a team of 2–3 students

---

## Further Resources

- **Unitree Documentation**: [https://www.unitreerobotics.com/](https://www.unitreerobotics.com/)
- **Unitree ROS 2 Bridge**: [https://github.com/unitreerobotics/unitree_ros2](https://github.com/unitreerobotics/unitree_ros2)
- **G1 Technical Specs**: [https://www.unitreerobotics.com/products](https://www.unitreerobotics.com/products)
- **Humanoid Robotics Research**: [IEEE Humanoid Robot Conference](https://www.humanoids.org/)

---

**Last Updated**: 2025-12-10
**Relevant For**: Advanced capstone extensions (post-course)
**Capstone Connection**: Optional validation platform for voice-controlled humanoid robot
