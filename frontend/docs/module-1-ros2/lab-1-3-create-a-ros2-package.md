---
id: module-1-lab-1-3
title: "Lab 1.3: Create a ROS 2 Package"
sidebar_position: 9
sidebar_label: "Lab 1.3: ROS 2 Package"
description: "Organize multiple nodes in a proper ROS 2 package with launch files"
keywords: [lab, ROS 2, package, organization, launch file, structure]
---

# Lab 1.3: Create a ROS 2 Package

## Lab Objective

Create a complete, organized ROS 2 package with:
1. Multiple nodes in proper package structure
2. Launch file to start all nodes
3. Configuration files with parameters

**Success Criteria**:
- Package builds without errors ✅
- Launch file starts all nodes ✅
- Nodes communicate via topics ✅
- Parameters load correctly ✅

---

## Prerequisites

- Completed Labs 1.1 and 1.2
- ROS 2 Humble
- ~40 minutes

---

## Part 1: Create Package

### 1.1 Create Workspace (if needed)

```bash
mkdir -p robot_ws/src
cd robot_ws
```

### 1.2 Create Package

```bash
cd src
ros2 pkg create --build-type ament_python my_robot_system
cd ..
```

---

## Part 2: Package Structure

Create this structure:

```
my_robot_system/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/my_robot_system/
├── launch/
│   ├── robot.launch.xml
│   └── robot_sim.launch.xml
├── config/
│   └── robot_params.yaml
├── my_robot_system/
│   ├── __init__.py
│   ├── robot_node.py
│   ├── sensor_node.py
│   └── controller_node.py
└── README.md
```

### 1.3 Create Directories

```bash
cd my_robot_system
mkdir -p launch config
cd ../..
```

---

## Part 3: Create Nodes

### 3.1 Sensor Node

**File**: `src/my_robot_system/my_robot_system/sensor_node.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random

class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # Declare and get parameters
        self.declare_parameter('sensor_rate', 10)
        self.declare_parameter('noise_level', 0.1)

        rate = self.get_parameter('sensor_rate').value
        self.noise = self.get_parameter('noise_level').value

        # Create publisher
        self.pub = self.create_publisher(Float32, '/sensor/temperature', 10)

        # Create timer (1/rate seconds)
        self.timer = self.create_timer(1.0 / rate, self.publish_sensor)

        self.get_logger().info(f'Sensor node started (rate: {rate} Hz)')

    def publish_sensor(self):
        # Simulate sensor reading (20-25°C with noise)
        value = 22.5 + random.uniform(-self.noise, self.noise)

        msg = Float32()
        msg.data = value
        self.pub.publish(msg)
        self.get_logger().debug(f'Temperature: {value:.2f}°C')


def main(args=None):
    rclpy.init(args=args)
    node = SensorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3.2 Controller Node

**File**: `src/my_robot_system/my_robot_system/controller_node.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
from geometry_msgs.msg import Twist

class ControllerNode(Node):
    def __init__(self):
        super().__init__('controller_node')

        # Declare parameters
        self.declare_parameter('control_rate', 20)
        self.declare_parameter('temp_threshold', 25.0)

        rate = self.get_parameter('control_rate').value
        self.threshold = self.get_parameter('temp_threshold').value

        # Subscribers
        self.temp_sub = self.create_subscription(
            Float32, '/sensor/temperature', self.temp_callback, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/robot/status', 10)

        # Timer for control loop
        self.timer = self.create_timer(1.0 / rate, self.control_loop)

        self.last_temp = 22.0
        self.get_logger().info('Controller node started')

    def temp_callback(self, msg):
        self.last_temp = msg.data
        if msg.data > self.threshold:
            self.get_logger().warn(f'Temperature HIGH: {msg.data:.2f}°C')

    def control_loop(self):
        # Publish status
        status = String()
        status.data = f'OK (temp: {self.last_temp:.1f}°C)'
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 3.3 Robot Node

**File**: `src/my_robot_system/my_robot_system/robot_node.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        self.declare_parameter('robot_name', 'my_robot')
        robot_name = self.get_parameter('robot_name').value

        # Subscriber
        self.status_sub = self.create_subscription(
            String, '/robot/status', self.status_callback, 10)

        self.get_logger().info(f'Robot node started: {robot_name}')

    def status_callback(self, msg):
        self.get_logger().info(f'Status: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Part 4: Create Launch File

### 4.1 Main Launch File

**File**: `src/my_robot_system/launch/robot.launch.xml`

```xml
<?xml version="1.0"?>
<launch>
  <arg name="robot_name" default="my_robot"/>
  <arg name="config_file" default="robot_params.yaml"/>

  <!-- Load parameters from config file -->
  <node pkg="my_robot_system" exec="sensor_node" name="sensor" output="screen">
    <param from="$(find-pkg-share my_robot_system)/config/$(var config_file)"/>
  </node>

  <node pkg="my_robot_system" exec="controller_node" name="controller" output="screen">
    <param from="$(find-pkg-share my_robot_system)/config/$(var config_file)"/>
  </node>

  <node pkg="my_robot_system" exec="robot_node" name="robot" output="screen">
    <param name="robot_name" value="$(var robot_name)"/>
  </node>
</launch>
```

---

## Part 5: Create Configuration File

**File**: `src/my_robot_system/config/robot_params.yaml`

```yaml
# Sensor parameters
sensor_rate: 10
noise_level: 0.1

# Controller parameters
control_rate: 20
temp_threshold: 25.0
```

---

## Part 6: Update Package Files

### 6.1 Update setup.py

Edit `src/my_robot_system/setup.py`:

```python
from setuptools import setup
from glob import glob

package_name = 'my_robot_system'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='My robot system package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sensor_node = my_robot_system.sensor_node:main',
            'controller_node = my_robot_system.controller_node:main',
            'robot_node = my_robot_system.robot_node:main',
        ],
    },
)
```

### 6.2 Update package.xml

Edit `src/my_robot_system/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_system</name>
  <version>0.0.1</version>
  <description>Complete robot system example</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <exec_depend>ros2launch</exec_depend>
</package>
```

---

## Part 7: Build and Test

### 7.1 Build

```bash
cd ~/robot_ws
colcon build
source install/setup.bash
```

### 7.2 Run with Launch File

```bash
ros2 launch my_robot_system robot.launch.xml
```

**Expected Output**:
```
[INFO] Sensor node started (rate: 10 Hz)
[INFO] Controller node started
[INFO] Robot node started: my_robot
[INFO] Status: OK (temp: 22.1°C)
[INFO] Status: OK (temp: 22.3°C)
...
```

### 7.3 Run with Custom Parameters

```bash
ros2 launch my_robot_system robot.launch.xml robot_name:=robot_alpha
```

---

## Part 8: Verification Checklist

- [ ] Package builds without errors
- [ ] Launch file starts all 3 nodes
- [ ] Sensor publishes temperature
- [ ] Controller receives temperature and publishes status
- [ ] Robot node receives status
- [ ] No error messages
- [ ] Can view topics with `ros2 topic list`
- [ ] Parameter loading works

---

## Part 9: Inspect Running System

```bash
# List active nodes
ros2 node list
# Output:
# /sensor
# /controller
# /robot

# List topics
ros2 topic list
# Output:
# /sensor/temperature
# /robot/status
# /parameter_events

# View one topic
ros2 topic echo /sensor/temperature

# Check parameters
ros2 param list /sensor
```

---

## Summary

You've created:
- ✅ Well-organized ROS 2 package
- ✅ Multiple communicating nodes
- ✅ Launch file to start system
- ✅ Configuration files

**Key Concepts**:
- Packages organize code, nodes, configs
- Launch files start multi-node systems
- Parameters make nodes configurable
- Nodes communicate via topics

**What's Next**: Module 1 is complete! Start Module 2 to learn simulation with Gazebo.

---

## Navigation

- **Previous Lab**: [Lab 1.2: Implement a Service](./lab-1-2-implement-a-service.md)
- **Module Summary**: [Chapter 7: Module Summary](./summary.md)
- **Next Module**: [Module 2: Gazebo Simulation](../module-2-simulation/intro.md)
