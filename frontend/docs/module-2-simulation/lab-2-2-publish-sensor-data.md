---
id: module-2-lab-2
title: "Lab 2.2: Publish Sensor Data"
sidebar_position: 11
sidebar_label: "Lab 2.2"
description: "Add camera and LiDAR sensors to URDF and publish data to ROS 2 topics"
keywords: [sensors, camera, LiDAR, ROS 2 topics, sensor data, lab]
---

# Lab 2.2: Publish Sensor Data

## Lab Objective

**Goal**: Add camera and LiDAR sensors to the robot URDF and verify sensor data publishes to ROS 2 topics.

**Skills**: URDF sensor integration, Gazebo plugins, sensor data visualization.

**Time**: 45 minutes

---

## Prerequisites

- ✅ Lab 2.1 complete (robot loads in Gazebo)
- ✅ Module 2 Chapter 3 read (Sensors in Gazebo)
- ✅ ROS 2 topic tools available (`ros2 topic list`, `ros2 topic echo`)

---

## Setup (5 minutes)

Use workspace from Lab 2.1:

```bash
cd ~/robotics_ws
source install/setup.bash
```

---

## Step 1: Update URDF with Sensors (15 minutes)

Edit `~/robotics_ws/src/my_robot/urdf/humanoid.urdf` and add these links/joints before closing `</robot>`:

### 1.1: Camera Link

```xml
  <!-- Camera link (on head) -->
  <link name="camera_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
      <material name="black">
        <color rgba="0 0 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="camera_joint" type="fixed">
    <parent link="head"/>
    <child link="camera_link"/>
    <origin xyz="0 0 0.08" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo camera plugin -->
  <gazebo reference="camera_link">
    <sensor type="camera" name="camera">
      <always_on>true</always_on>
      <update_rate>30</update_rate>
      <visualize>true</visualize>
      <camera>
        <horizontal_fov>1.047</horizontal_fov>
        <image>
          <width>640</width>
          <height>480</height>
          <format>R8G8B8</format>
        </image>
        <clip>
          <near>0.05</near>
          <far>100</far>
        </clip>
      </camera>
      <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
        <ros>
          <remapping>~/image_raw:=/camera/image_raw</remapping>
          <remapping>~/camera_info:=/camera/camera_info</remapping>
        </ros>
        <camera_name>camera</camera_name>
        <frame_name>camera_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```

### 1.2: LiDAR Link

```xml
  <!-- LiDAR link (on top of head) -->
  <link name="lidar_link">
    <visual>
      <geometry>
        <cylinder radius="0.04" length="0.03"/>
      </geometry>
      <material name="purple">
        <color rgba="1 0 1 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.04" length="0.03"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="0.2"/>
      <inertia ixx="0.001" iyy="0.001" izz="0.001" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="lidar_joint" type="fixed">
    <parent link="head"/>
    <child link="lidar_link"/>
    <origin xyz="0 0 0.12" rpy="0 0 0"/>
  </joint>

  <!-- Gazebo LiDAR plugin -->
  <gazebo reference="lidar_link">
    <sensor type="gpu_lidar" name="lidar">
      <always_on>true</always_on>
      <update_rate>10</update_rate>
      <lidar>
        <scan>
          <horizontal>
            <samples>360</samples>
            <resolution>1</resolution>
            <min_angle>0</min_angle>
            <max_angle>6.283</max_angle>
          </horizontal>
        </scan>
        <range>
          <min>0.08</min>
          <max>100</max>
          <resolution>0.01</resolution>
        </range>
      </lidar>
      <plugin name="gpu_lidar" filename="libgazebo_ros_gpu_lidar.so">
        <ros>
          <remapping>~/scan:=/scan</remapping>
        </ros>
        <frame_name>lidar_link</frame_name>
      </plugin>
    </sensor>
  </gazebo>
```

---

## Step 2: Build and Launch (10 minutes)

```bash
cd ~/robotics_ws
colcon build
source install/setup.bash
ros2 launch my_robot gazebo.launch.xml
```

**Verify launch succeeds** with no URDF errors.

---

## Step 3: Verify Topics (10 minutes)

In a new terminal:

```bash
source ~/robotics_ws/install/setup.bash

# List all topics
ros2 topic list
```

**Expected output**:
```
/camera/image_raw
/camera/camera_info
/scan
```

### 3.1: View Camera Data

```bash
# Show one image message
ros2 topic echo /camera/image_raw --once

# Or use RViz
ros2 run rviz2 rviz2
# Add "Image" display → Topic: /camera/image_raw
```

### 3.2: View LiDAR Data

```bash
# Show one scan message
ros2 topic echo /scan --once

# Or use RViz
# Add "LaserScan" display → Topic: /scan
```

---

## Step 4: Create a Sensor Subscriber Node (10 minutes)

Create `~/robotics_ws/src/my_robot/src/sensor_subscriber.py`:

```python
#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Image, LaserScan

class SensorSubscriber(rclpy.node.Node):
    def __init__(self):
        super().__init__('sensor_subscriber')

        # Subscribe to camera
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        # Subscribe to LiDAR
        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.camera_count = 0
        self.lidar_count = 0

    def camera_callback(self, msg):
        self.camera_count += 1
        self.get_logger().info(
            f"Camera #{self.camera_count}: {msg.width}x{msg.height} {msg.encoding}"
        )

    def lidar_callback(self, msg):
        self.lidar_count += 1
        min_distance = min(msg.ranges)
        self.get_logger().info(
            f"LiDAR #{self.lidar_count}: {len(msg.ranges)} points, "
            f"min distance: {min_distance:.2f}m"
        )

def main(args=None):
    rclpy.init(args=args)
    node = SensorSubscriber()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Make executable and run:

```bash
chmod +x ~/robotics_ws/src/my_robot/src/sensor_subscriber.py
python3 ~/robotics_ws/src/my_robot/src/sensor_subscriber.py
```

**Expected output**:
```
[INFO] Camera #1: 640x480 rgb8
[INFO] LiDAR #1: 360 points, min distance: 2.34m
[INFO] Camera #2: 640x480 rgb8
[INFO] LiDAR #2: 360 points, min distance: 2.33m
```

---

## Expected Output

```
Gazebo window shows:
✓ Robot with camera (black box on head)
✓ Robot with LiDAR (purple cylinder on head)
✓ Camera and LiDAR data streaming
✓ Sensor subscriber prints data
```

---

## Verification Checklist

- [ ] URDF builds without errors
- [ ] Topics `/camera/image_raw` and `/scan` appear in `ros2 topic list`
- [ ] Can echo camera image data
- [ ] Can echo LiDAR scan data
- [ ] Sensor subscriber prints at 30 Hz (camera) and 10 Hz (LiDAR)

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Topic not appearing | Check gazebo logs: `~/.gazebo/server-default/default.log` |
| Plugin not found | Ensure `gazebo_ros` package installed: `apt install ros-humble-gazebo-ros-pkgs` |
| Image malformed | Check camera format is supported (R8G8B8) |
| LiDAR shows zero range | Move closer to object in world |

---

## Extension Activities

1. **Add IMU**: Attach IMU sensor to torso
2. **Record data**: Save camera images and LiDAR scans to files
3. **Visualize fusion**: Combine camera + LiDAR in RViz
4. **Add noise**: Increase sensor noise in URDF plugins

---

## Summary

**Lab 2.2 accomplishes**:
- Add sensors to URDF ✓
- Configure Gazebo plugins ✓
- Verify sensor data streams ✓
- Foundation for Lab 2.3 ✓

---

## Navigation

- **Previous Lab**: [Lab 2.1: Load Robot](./lab-2-1-load-robot-in-gazebo.md)
- **Next Lab**: [Lab 2.3: Control Robot](./lab-2-3-control-robot-in-simulation.md)
