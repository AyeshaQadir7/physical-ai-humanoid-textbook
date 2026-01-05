---
id: module-2-sensors
title: "Sensors in Gazebo"
sidebar_position: 3
sidebar_label: "Sensors"
description: "Simulate cameras, LiDAR, and IMU sensors in Gazebo and publish data to ROS 2"
keywords: [sensors, Gazebo, camera, LiDAR, IMU, sensor fusion, ROS 2 topics]
---

# Sensors in Gazebo

## Introduction

Robots perceive the world through **sensors**. In Gazebo, we simulate:
- **Cameras**: RGB images and depth (RGBD)
- **LiDAR**: Laser range measurements
- **IMU**: Accelerometer and gyroscope
- **Force/Torque sensors**: Contact forces

---

## Learning Outcomes

By the end, you will:
1. Add sensors to URDF files
2. Configure Gazebo sensor plugins
3. Publish sensor data to ROS 2 topics
4. Visualize sensor output
5. Understand sensor noise and realism

---

## Part 1: Camera Sensors

### Camera Plugin in URDF

```xml
<gazebo reference="camera_link">
  <sensor type="camera" name="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
    <visualize>true</visualize>
    <camera>
      <horizontal_fov>1.047</horizontal_fov>  <!-- 60 degrees -->
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <clip>
        <near>0.05</near>
        <far>100</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
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

### Subscribing to Camera Data

```python
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraSubscriber(rclpy.node.Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        # Process image
        cv2.imshow("Robot Camera", cv_image)
        cv2.waitKey(1)

if __name__ == '__main__':
    rclpy.init()
    node = CameraSubscriber()
    rclpy.spin(node)
```

**Topics published**:
- `/camera/image_raw` (Image)
- `/camera/camera_info` (CameraInfo)

---

## Part 2: LiDAR Sensors

### LiDAR Plugin in URDF

```xml
<gazebo reference="lidar_link">
  <sensor type="gpu_lidar" name="lidar">
    <always_on>true</always_on>
    <update_rate>40</update_rate>
    <lidar>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>0</min_angle>
          <max_angle>6.283</max_angle>
        </horizontal>
        <vertical>
          <samples>64</samples>
          <resolution>1</resolution>
          <min_angle>-0.5236</min_angle>
          <max_angle>0.5236</max_angle>
        </vertical>
      </scan>
      <range>
        <min>0.08</min>
        <max>100</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
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

### Subscribing to LiDAR Data

```python
import rclpy
from sensor_msgs.msg import LaserScan

class LiDARSubscriber(rclpy.node.Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

    def scan_callback(self, msg):
        # msg.ranges contains distance measurements
        num_points = len(msg.ranges)
        min_distance = min(msg.ranges)
        self.get_logger().info(f"LiDAR: {num_points} points, min distance: {min_distance:.2f}m")

if __name__ == '__main__':
    rclpy.init()
    node = LiDARSubscriber()
    rclpy.spin(node)
```

**Topics published**:
- `/scan` (LaserScan) - 2D point cloud
- `/scan_3d` or `/points` (PointCloud2) - 3D point cloud

---

## Part 3: IMU Sensors

### IMU Plugin in URDF

```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="imu">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.05</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.05</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.05</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    <plugin name="imu_plugin" filename="libgazebo_ros_imu.so">
      <ros>
        <remapping>~/imu:=/imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### Subscribing to IMU Data

```python
import rclpy
from sensor_msgs.msg import Imu

class IMUSubscriber(rclpy.node.Node):
    def __init__(self):
        super().__init__('imu_subscriber')
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

    def imu_callback(self, msg):
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        self.get_logger().info(f"Accel: ({ax:.2f}, {ay:.2f}, {az:.2f}) | Gyro: ({gx:.2f}, {gy:.2f}, {gz:.2f})")

if __name__ == '__main__':
    rclpy.init()
    node = IMUSubscriber()
    rclpy.spin(node)
```

**Topics published**:
- `/imu/data` (Imu)

---

## Part 4: Sensor Fusion

Combining multiple sensors for better perception:

```python
import rclpy
from sensor_msgs.msg import Image, LaserScan, Imu
from message_filters import ApproximateTimeSynchronizer, Subscriber

class SensorFusion(rclpy.node.Node):
    def __init__(self):
        super().__init__('sensor_fusion')

        # Subscribers
        image_sub = Subscriber(self, Image, '/camera/image_raw')
        scan_sub = Subscriber(self, LaserScan, '/scan')
        imu_sub = Subscriber(self, Imu, '/imu/data')

        # Synchronize messages
        ats = ApproximateTimeSynchronizer([image_sub, scan_sub, imu_sub], queue_size=10, slop=0.1)
        ats.registerCallback(self.fusion_callback)

    def fusion_callback(self, image_msg, scan_msg, imu_msg):
        self.get_logger().info("Fused sensor data received")
        # Process all three sensors together
        # Example: Use LiDAR for distance, camera for object detection, IMU for stability

if __name__ == '__main__':
    rclpy.init()
    node = SensorFusion()
    rclpy.spin(node)
```

---

## Part 5: Sensor Visualization

### Using RQT to View Sensor Data

```bash
# View camera feed
ros2 run rqt_gui rqt_gui
# Plugin → Visualization → Image View → select /camera/image_raw

# View LiDAR scan
ros2 run rviz2 rviz2
# Add LaserScan → /scan

# View IMU data
rostopic echo /imu/data
```

---

## Summary

**Sensors in Gazebo**:
- Camera: RGB/depth images
- LiDAR: Distance measurements
- IMU: Acceleration and rotation

**ROS 2 Topics**:
- `/camera/image_raw` (camera)
- `/scan` (LiDAR)
- `/imu/data` (IMU)

**Next**: Unity for high-fidelity visualization.

---

## Navigation

- **Previous**: [Chapter 2: URDF](./02-urdf-and-robot-descriptions.md)
- **Next**: [Chapter 4: Unity for Robotics](./04-unity-for-robotics.md)
