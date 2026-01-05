---
id: module-2-lab-3
title: "Lab 2.3: Control Robot in Simulation"
sidebar_position: 12
sidebar_label: "Lab 2.3"
description: "Send ROS 2 commands to move robot in Gazebo simulation"
keywords: [ROS 2, control, joint commands, velocity, simulation, lab]
---

# Lab 2.3: Control Robot in Simulation

## Lab Objective

**Goal**: Create ROS 2 nodes to send velocity commands and move the simulated robot.

**Skills**: ROS 2 publishers, joint velocity control, robot feedback loops.

**Time**: 45 minutes

---

## Prerequisites

- ✅ Lab 2.1-2.2 complete (robot with sensors)
- ✅ Module 1 chapters on nodes/topics
- ✅ Familiar with ROS 2 publishers

---

## Setup (5 minutes)

Use workspace from Lab 2.1:

```bash
cd ~/robotics_ws
source install/setup.bash
```

---

## Step 1: Add Joint Controllers to URDF (10 minutes)

Gazebo simulates physics but needs **controllers** to move joints via ROS 2 commands.

Add these `<gazebo>` sections to your URDF (before closing `</robot>`):

```xml
  <!-- Joint state publisher (broadcasts current joint positions) -->
  <gazebo>
    <plugin name="gazebo_ros_joint_state_broadcaster"
            filename="libgazebo_ros_joint_state_broadcaster.so">
      <ros>
        <remapping>~/out:=joint_states</remapping>
      </ros>
      <update_rate>100</update_rate>
    </plugin>
  </gazebo>

  <!-- Velocity controller for left arm -->
  <gazebo>
    <plugin name="gazebo_ros_joint_trajectory_controller"
            filename="libgazebo_ros_joint_trajectory_controller.so">
      <ros>
        <remapping>~/joint_trajectory:=/left_arm_controller/joint_trajectory</remapping>
        <remapping>~/state:=/left_arm_controller/state</remapping>
      </ros>
      <joint_name>left_arm_joint</joint_name>
    </plugin>
  </gazebo>

  <!-- Velocity controller for right arm -->
  <gazebo>
    <plugin name="gazebo_ros_joint_trajectory_controller"
            filename="libgazebo_ros_joint_trajectory_controller.so">
      <ros>
        <remapping>~/joint_trajectory:=/right_arm_controller/joint_trajectory</remapping>
        <remapping>~/state:=/right_arm_controller/state</remapping>
      </ros>
      <joint_name>right_arm_joint</joint_name>
    </plugin>
  </gazebo>

  <!-- Velocity controller for left leg -->
  <gazebo>
    <plugin name="gazebo_ros_joint_trajectory_controller"
            filename="libgazebo_ros_joint_trajectory_controller.so">
      <ros>
        <remapping>~/joint_trajectory:=/left_leg_controller/joint_trajectory</remapping>
        <remapping>~/state:=/left_leg_controller/state</remapping>
      </ros>
      <joint_name>left_leg_joint</joint_name>
    </plugin>
  </gazebo>

  <!-- Velocity controller for right leg -->
  <gazebo>
    <plugin name="gazebo_ros_joint_trajectory_controller"
            filename="libgazebo_ros_joint_trajectory_controller.so">
      <ros>
        <remapping>~/joint_trajectory:=/right_leg_controller/joint_trajectory</remapping>
        <remapping>~/state:=/right_leg_controller/state</remapping>
      </ros>
      <joint_name>right_leg_joint</joint_name>
    </plugin>
  </gazebo>
```

---

## Step 2: Create a Control Publisher Node (15 minutes)

Create `~/robotics_ws/src/my_robot/src/robot_controller.py`:

```python
#!/usr/bin/env python3

import rclpy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class RobotController(rclpy.node.Node):
    def __init__(self):
        super().__init__('robot_controller')

        # Publishers for joint commands
        self.left_arm_pub = self.create_publisher(
            JointTrajectory,
            '/left_arm_controller/joint_trajectory',
            10
        )
        self.right_arm_pub = self.create_publisher(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            10
        )
        self.left_leg_pub = self.create_publisher(
            JointTrajectory,
            '/left_leg_controller/joint_trajectory',
            10
        )
        self.right_leg_pub = self.create_publisher(
            JointTrajectory,
            '/right_leg_controller/joint_trajectory',
            10
        )

        # Subscriber for joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Timer to send commands periodically
        self.timer = self.create_timer(0.5, self.control_callback)
        self.time_step = 0
        self.current_joint_positions = {}

    def joint_state_callback(self, msg):
        """Store current joint positions"""
        for i, name in enumerate(msg.name):
            self.current_joint_positions[name] = msg.position[i]

    def send_joint_command(self, publisher, joint_name, target_position):
        """Send a joint trajectory command"""
        traj = JointTrajectory()
        traj.joint_names = [joint_name]

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = [target_position]
        point.time_from_start.sec = 1  # 1 second duration

        traj.points = [point]
        publisher.publish(traj)

    def control_callback(self):
        """Control callback - move arms and legs in a wave pattern"""
        time = self.time_step * 0.5  # 0.5 second timesteps

        # Arm wave: oscillate arms
        arm_angle = math.sin(time) * 1.0  # ±1.0 radians
        self.send_joint_command(self.left_arm_pub, 'left_arm_joint', arm_angle)
        self.send_joint_command(self.right_arm_pub, 'right_arm_joint', -arm_angle)

        # Leg wave: oscillate legs (opposite phase)
        leg_angle = math.cos(time) * 0.5  # ±0.5 radians
        self.send_joint_command(self.left_leg_pub, 'left_leg_joint', leg_angle)
        self.send_joint_command(self.right_leg_pub, 'right_leg_joint', -leg_angle)

        self.time_step += 1

        # Log joint positions
        if 'left_arm_joint' in self.current_joint_positions:
            self.get_logger().info(
                f"Left arm: {self.current_joint_positions['left_arm_joint']:.2f} rad"
            )

def main(args=None):
    rclpy.init(args=args)
    node = RobotController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Step 3: Build and Test (15 minutes)

### 3.1: Build package

```bash
cd ~/robotics_ws
colcon build
source install/setup.bash
```

### 3.2: Launch Gazebo

Terminal 1:
```bash
source ~/robotics_ws/install/setup.bash
ros2 launch my_robot gazebo.launch.xml
```

Wait for Gazebo to fully load (30 seconds).

### 3.3: Run controller

Terminal 2:
```bash
source ~/robotics_ws/install/setup.bash
python3 ~/robotics_ws/src/my_robot/src/robot_controller.py
```

**Expected output**:
```
[INFO] Left arm: 0.89 rad
[INFO] Left arm: 0.65 rad
[INFO] Left arm: 0.23 rad
...
```

### 3.4: Observe in Gazebo

Watch the robot in Gazebo window:
- Arms should wave side-to-side
- Legs should move in opposite phase
- Movement smooth and continuous

---

## Step 4: Verify Feedback Loop (10 minutes)

Create `~/robotics_ws/src/my_robot/src/robot_monitor.py`:

```python
#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import JointState

class RobotMonitor(rclpy.node.Node):
    def __init__(self):
        super().__init__('robot_monitor')
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        """Monitor all joint states"""
        self.get_logger().info("=== Joint States ===")
        for i, name in enumerate(msg.name):
            pos = msg.position[i]
            vel = msg.velocity[i] if i < len(msg.velocity) else 0.0
            self.get_logger().info(f"{name}: pos={pos:.2f}, vel={vel:.2f}")

def main(args=None):
    rclpy.init(args=args)
    node = RobotMonitor()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run in Terminal 3:
```bash
source ~/robotics_ws/install/setup.bash
python3 ~/robotics_ws/src/my_robot/src/robot_monitor.py
```

---

## Expected Output

```
Gazebo shows:
✓ Robot arms waving smoothly
✓ Robot legs moving in opposite phase
✓ Movement continuous and stable
✓ No joint violations (angles within limits)

Terminal output:
✓ Joint positions printed
✓ Velocity feedback shown
✓ No ROS 2 errors
```

---

## Verification Checklist

- [ ] Gazebo loads without errors
- [ ] `ros2 topic list` shows `/joint_states`
- [ ] Robot moves when controller publishes
- [ ] Movement is smooth (not jerky)
- [ ] Joint limits respected (arms don't overextend)
- [ ] Monitor node prints joint positions

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Robot doesn't move | Check `/joint_states` is publishing |
| Movement jerky | Increase trajectory time duration |
| Joint limit violations | Reduce amplitude of oscillation |
| Controller won't publish | Ensure gazebo_ros plugins loaded |
| Performance slow | Run Gazebo headless in background |

---

## Extension Activities

1. **Keyboard control**: Create node to read keyboard input and move robot
2. **Inverse kinematics**: Use IK to position robot end-effector
3. **Walking pattern**: Implement gait controller for bipedal walking
4. **Path planning**: Move robot to target position while avoiding obstacles

---

## Summary

**Lab 2.3 accomplishes**:
- Add controllers to robot URDF ✓
- Publish joint trajectory commands ✓
- Observe robot movement in Gazebo ✓
- Verify feedback loop ✓
- Ready for Module 3 ✓

---

## Navigation

- **Previous Lab**: [Lab 2.2: Sensors](./lab-2-2-publish-sensor-data.md)
- **Next**: [Module 2 Summary](./06-module-2-summary.md)
