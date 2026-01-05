---
id: module-1-launch-parameters
title: "Launch Files & Parameters"
sidebar_position: 6
sidebar_label: "Launch Files & Parameters"
description: "Starting multi-node systems with launch files and managing configuration with parameters"
keywords: [ROS 2, launch files, parameters, XML, multi-node, configuration, package.xml]
---

# Launch Files & Parameters

## Introduction

So far, you've run one node at a time:

```bash
ros2 run package node_name
```

But real robots have **many nodes** running together (sensor drivers, controllers, planning, perception, etc.). **Launch files** start them all with one command:

```bash
ros2 launch package launch_file.launch.xml
```

This chapter teaches:
- Writing **launch files** (XML syntax)
- Starting **multiple nodes** with arguments
- **Configurable parameters** via launch files
- Organizing your ROS 2 project

---

## Learning Outcomes

By the end of this chapter, you will:
1. Write launch files in XML
2. Start multiple nodes with one command
3. Pass parameters to nodes via launch files
4. Understand ROS 2 package structure
5. Organize multi-node systems
6. Debug launch file issues

---

## Part 1: ROS 2 Package Structure

### Minimum Package Structure

```
my_robot_package/
├── package.xml              # Package metadata
├── setup.py                 # Install configuration
├── setup.cfg                # Install settings
├── resource/my_robot_package/
├── my_robot_package/        # Python source code
│   ├── __init__.py
│   ├── robot_node.py        # Your node code
│   └── utils.py
└── launch/                  # Launch files
    └── robot.launch.xml
```

### package.xml Template

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_package</name>
  <version>0.0.1</version>
  <description>My robot controller package</description>
  <maintainer email="you@example.com">Your Name</maintainer>
  <license>MIT</license>

  <buildtool_depend>ament_python</buildtool_depend>

  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>

  <test_depend>pytest</test_depend>

  <exec_depend>ros2launch</exec_depend>
</package>
```

**Key fields**:
- `<name>` - package name
- `<depend>` - packages this depends on
- `<exec_depend>` - runtime dependencies
- Ensure ROS 2 can find your packages

---

## Part 2: Simple Launch File

### Example 1: Launch a Single Node

**File**: `launch/simple.launch.xml`

```xml
<?xml version="1.0"?>
<launch>
  <!-- Start a single node -->
  <node pkg="my_robot_package" exec="robot_node.py" name="my_robot"/>
</launch>
```

**Run it**:
```bash
ros2 launch my_robot_package simple.launch.xml
```

### Example 2: Launch Multiple Nodes

**File**: `launch/multi_node.launch.xml`

```xml
<?xml version="1.0"?>
<launch>
  <!-- Sensor node -->
  <node pkg="my_robot_package" exec="sensor_node.py" name="sensor">
    <!-- Redirect output to screen (useful for debugging) -->
    <output>screen</output>
  </node>

  <!-- Controller node -->
  <node pkg="my_robot_package" exec="controller_node.py" name="controller">
    <output>screen</output>
  </node>

  <!-- Motor driver node -->
  <node pkg="my_robot_package" exec="motor_node.py" name="motor">
    <output>screen</output>
  </node>
</launch>
```

**Run it**:
```bash
ros2 launch my_robot_package multi_node.launch.xml
```

All three nodes start with one command! Ctrl+C stops all of them.

---

## Part 3: Parameters in Launch Files

### Example 3: Pass Parameters to Nodes

**File**: `launch/robot_with_params.launch.xml`

```xml
<?xml version="1.0"?>
<launch>
  <!-- Define arguments (input to launch file) -->
  <arg name="speed" default="0.5"/>
  <arg name="debug" default="false"/>

  <!-- Start node with parameters -->
  <node pkg="my_robot_package" exec="robot_node.py" name="robot">
    <param name="motor_speed" value="$(var speed)"/>
    <param name="debug_mode" value="$(var debug)"/>
  </node>
</launch>
```

**Run with custom values**:
```bash
# Use defaults
ros2 launch my_robot_package robot_with_params.launch.xml

# Override arguments
ros2 launch my_robot_package robot_with_params.launch.xml speed:=1.0 debug:=true
```

### Example 4: Parameter File

For complex configurations, use a YAML file:

**File**: `config/robot_params.yaml`

```yaml
robot:
  motor_speed: 0.5
  wheel_diameter: 0.1
  max_speed: 2.0
  debug_mode: false

sensor:
  lidar_range: 10.0
  lidar_update_rate: 10

control:
  loop_rate: 100
  kp: 1.0
  ki: 0.1
  kd: 0.01
```

**File**: `launch/robot_with_config.launch.xml`

```xml
<?xml version="1.0"?>
<launch>
  <!-- Load all parameters from YAML file -->
  <node pkg="my_robot_package" exec="robot_node.py" name="robot">
    <param from="$(find-pkg-share my_robot_package)/config/robot_params.yaml"/>
  </node>
</launch>
```

---

## Part 4: Advanced Launch Features

### Groups and Conditionals

```xml
<?xml version="1.0"?>
<launch>
  <arg name="use_sim" default="true"/>
  <arg name="robot_name" default="robot1"/>

  <!-- Conditional: use different nodes based on argument -->
  <group if="$(var use_sim)">
    <!-- Simulation nodes -->
    <node pkg="gazebo_ros" exec="gazebo" args="--verbose" name="gazebo">
      <output>screen</output>
    </node>
    <node pkg="my_robot_package" exec="sim_interface.py" name="interface">
      <output>screen</output>
    </node>
  </group>

  <group unless="$(var use_sim)">
    <!-- Real robot nodes -->
    <node pkg="my_robot_package" exec="hardware_interface.py" name="hardware">
      <output>screen</output>
    </node>
  </group>

  <!-- Common nodes for both paths -->
  <node pkg="my_robot_package" exec="controller.py" name="controller">
    <param name="robot_name" value="$(var robot_name)"/>
  </node>
</launch>
```

**Run with different configurations**:
```bash
# Use simulation
ros2 launch my_robot_package robot.launch.xml use_sim:=true

# Use real hardware
ros2 launch my_robot_package robot.launch.xml use_sim:=false robot_name:=real_robot_1
```

### Namespaces

Namespaces help organize multiple robots or subsystems:

```xml
<?xml version="1.0"?>
<launch>
  <!-- Robot 1 in namespace /robot1 -->
  <group ns="robot1">
    <node pkg="my_robot_package" exec="controller.py" name="controller"/>
    <node pkg="my_robot_package" exec="sensor_node.py" name="sensor"/>
  </group>

  <!-- Robot 2 in namespace /robot2 -->
  <group ns="robot2">
    <node pkg="my_robot_package" exec="controller.py" name="controller"/>
    <node pkg="my_robot_package" exec="sensor_node.py" name="sensor"/>
  </group>
</launch>
```

**Topics are now scoped**:
```
/robot1/sensor/temperature
/robot1/cmd/velocity
/robot2/sensor/temperature
/robot2/cmd/velocity
```

---

## Part 5: Complete Launch File Example

```xml
<?xml version="1.0"?>
<launch>
  <!-- Arguments -->
  <arg name="robot_name" default="my_robot"/>
  <arg name="simulation" default="true"/>
  <arg name="config_file" default="robot_params.yaml"/>

  <!-- Include another launch file -->
  <include file="$(find-pkg-share my_robot_package)/launch/sensor_stack.launch.xml"/>

  <!-- Load parameters from file -->
  <node pkg="my_robot_package" exec="robot_controller.py" name="controller" output="screen">
    <param name="name" value="$(var robot_name)"/>
    <param from="$(find-pkg-share my_robot_package)/config/$(var config_file)"/>
  </node>

  <!-- Start nodes conditionally -->
  <group if="$(var simulation)">
    <node pkg="gazebo_ros" exec="gazebo" name="gazebo" output="screen"/>
    <node pkg="my_robot_package" exec="sim_bridge.py" name="bridge"/>
  </group>

  <group unless="$(var simulation)">
    <node pkg="my_robot_package" exec="hardware_driver.py" name="hw_driver"/>
  </group>

  <!-- RViz for visualization (optional) -->
  <node pkg="rviz2" exec="rviz2" name="rviz" output="screen">
    <param name="rviz_config" value="$(find-pkg-share my_robot_package)/config/rviz.rviz"/>
  </node>
</launch>
```

---

## Part 6: Debugging Launch Files

### Common Issues

| Error | Cause | Solution |
|-------|-------|----------|
| `Package not found` | Python path wrong | Rebuild: `colcon build --symlink-install` |
| `Node not found` | Wrong executable name | Check `setup.py` entry_points |
| `Parameter not set` | Parameter name mismatch | Use `ros2 param list` to verify |
| `Port in use` | Multiple nodes on same ROS_DOMAIN_ID | Use different `ROS_DOMAIN_ID` or namespaces |

### Debug Commands

```bash
# Check if launch file syntax is valid
ros2 launch my_robot_package robot.launch.xml --validate

# See what nodes will be started (dry run)
ros2 launch my_robot_package robot.launch.xml --print

# List active nodes after launch
ros2 node list

# Check node parameters
ros2 param list /robot_node

# Check package can be found
ros2 pkg prefix my_robot_package
```

---

## Part 7: Best Practices

### Launch File Organization

```
my_robot_package/
├── launch/
│   ├── robot.launch.xml           # Main entry point
│   ├── sensor_stack.launch.xml    # Sensor nodes
│   ├── control_stack.launch.xml   # Control nodes
│   └── sim.launch.xml             # Simulation variant
├── config/
│   ├── robot_params.yaml          # Default parameters
│   ├── robot_params_sim.yaml      # Simulation-specific params
│   └── rviz.rviz                  # RViz configuration
└── my_robot_package/
    ├── robot_node.py
    ├── sensor_node.py
    └── control_node.py
```

### Naming Conventions

✅ Do:
- `robot.launch.xml` - descriptive names
- `motor_speed` - lowercase with underscores
- Namespaces for multi-robot: `/robot1/sensor`

❌ Don't:
- `main.launch.xml` - too vague
- `motorSpeed` - inconsistent
- Deeply nested namespaces: `/a/b/c/d/e`

---

## Summary

**Package Structure**:
- `package.xml` - metadata
- `setup.py` - Python installation
- `launch/` - launch files
- `config/` - parameter files

**Launch Files**:
- XML format
- Start multiple nodes with one command
- Pass parameters to nodes
- Support conditionals and namespaces

**Parameters**:
- Configure nodes without recompiling
- Via launch file or command line
- Can use YAML files for complex configs

---

## Next Steps

You've now mastered Module 1 fundamentals! The three labs will cement your knowledge:
1. **Lab 1.1**: Your first ROS 2 publisher/subscriber
2. **Lab 1.2**: Implement a service
3. **Lab 1.3**: Create a complete ROS 2 package

---

## Navigation

- **Previous**: [Chapter 5: Python with rclpy](./05-python-with-rclpy.md)
- **Labs**: [Lab 1.1: Your First Node](./lab-1-1-your-first-node.md)
