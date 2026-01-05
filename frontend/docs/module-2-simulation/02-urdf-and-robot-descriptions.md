---
id: module-2-urdf
title: "URDF & Robot Descriptions"
sidebar_position: 2
sidebar_label: "URDF"
description: "Master Unified Robot Description Format - defining robot structure for Gazebo simulation"
keywords: [URDF, robot description, links, joints, inertia, sensors, Gazebo]
---

# URDF & Robot Descriptions

## Introduction

**URDF** (Unified Robot Description Format) is the language robots use to describe their structure.

URDF files define:
- **Links**: Rigid bodies (base, arms, legs, wheels)
- **Joints**: Connections between links (revolute, prismatic, fixed)
- **Inertial properties**: Mass and moment of inertia
- **Collision shapes**: For physics simulation
- **Visual geometry**: For rendering
- **Sensors**: Cameras, LiDAR, IMU

---

## Learning Outcomes

By the end, you will:
1. Understand URDF structure and elements
2. Write basic URDF files
3. Define links, joints, and their properties
4. Compute or specify inertia
5. Add sensors to URDF
6. Load URDF into Gazebo

---

## Part 1: URDF Structure

### Basic URDF Template

```xml
<?xml version="1.0"?>
<robot name="my_robot">
  <!-- Link 1: Base -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="5.0"/>
      <inertia ixx="0.05" iyy="0.05" izz="0.05" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Link 2: Arm -->
  <link name="arm_link">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.3"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Joint: Connect base to arm -->
  <joint name="base_to_arm" type="revolute">
    <parent link="base_link"/>
    <child link="arm_link"/>
    <origin xyz="0.1 0 0.05" rpy="0 0 0"/>
    <axis xyz="0 1 0"/>
    <limit lower="0" upper="3.14" effort="100" velocity="1.57"/>
  </joint>
</robot>
```

---

## Part 2: Links

### Link Elements

```xml
<link name="unique_link_name">
  <!-- Visual: how it looks -->
  <visual>
   let <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
      <!-- or: <cylinder radius="0.05" length="0.1"/> -->
      <!-- or: <sphere radius="0.05"/> -->
      <!-- or: <mesh filename="file.dae"/> -->
    </geometry>
    <material name="red">
      <color rgba="1 0 0 1"/>
    </material>
  </visual>

  <!-- Collision: physics shape -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry>
      <box size="0.1 0.1 0.1"/>
    </geometry>
  </collision>

  <!-- Inertial: mass and moment of inertia -->
  <inertial>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <mass value="2.0"/>
    <!-- Inertia matrix about center of mass -->
    <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
  </inertial>
</link>
```

### Geometric Shapes

```xml
<!-- Box -->
<geometry>
  <box size="width depth height"/>  <!-- meters -->
</geometry>

<!-- Cylinder -->
<geometry>
  <cylinder radius="r" length="h"/>
</geometry>

<!-- Sphere -->
<geometry>
  <sphere radius="r"/>
</geometry>

<!-- Mesh (for complex shapes) -->
<geometry>
  <mesh filename="package://my_pkg/meshes/model.dae" scale="1 1 1"/>
</geometry>
```

### Inertia Calculation

For simple shapes:

**Box**: `I_xx = (1/12) * m * (height² + depth²)`
**Cylinder**: `I_xx = (1/12) * m * (3*radius² + length²)`
**Sphere**: `I_xx = (2/5) * m * radius²`

---

## Part 3: Joints

### Joint Types

| Type | Movement | Use Case |
|------|----------|----------|
| `revolute` | Rotation around axis | Robot joints, hinges |
| `prismatic` | Linear motion | Sliders, telescoping arms |
| `fixed` | No movement | Permanently attached |
| `floating` | 6-DOF (free) | Base link (rarely used) |
| `planar` | 2D motion | Rare |

### Revolute Joint Example

```xml
<joint name="shoulder_joint" type="revolute">
  <parent link="upper_arm"/>
  <child link="lower_arm"/>
  <!-- Position relative to parent's frame -->
  <origin xyz="0 0 0.3" rpy="0 0 0"/>
  <!-- Rotation axis (normalized vector) -->
  <axis xyz="0 1 0"/>  <!-- Rotate around Y-axis -->
  <!-- Joint limits (rad, radians) -->
  <limit lower="-1.57" upper="1.57" effort="50" velocity="1.0"/>
  <!-- Friction -->
  <dynamics damping="0.1" friction="0.0"/>
</joint>
```

**Axis conventions**:
- `<axis xyz="1 0 0"/>` → Rotation around X-axis
- `<axis xyz="0 1 0"/>` → Rotation around Y-axis
- `<axis xyz="0 0 1"/>` → Rotation around Z-axis

### Prismatic Joint Example

```xml
<joint name="linear_actuator" type="prismatic">
  <parent link="base"/>
  <child link="slider"/>
  <origin xyz="0 0 0" rpy="0 0 0"/>
  <axis xyz="1 0 0"/>  <!-- Move along X-axis -->
  <limit lower="0" upper="1.0" effort="100" velocity="0.5"/>
</joint>
```

---

## Part 4: Adding Sensors to URDF

### Camera Sensor

```xml
<link name="camera_link">
  <visual>
    <geometry>
      <box size="0.05 0.05 0.05"/>
    </geometry>
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
  <origin xyz="0 0 0.1" rpy="0 0 0"/>
</joint>

<!-- Gazebo plugin for camera simulation -->
<gazebo reference="camera_link">
  <sensor type="camera" name="camera">
    <always_on>true</always_on>
    <update_rate>30</update_rate>
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
      </ros>
      <camera_name>camera</camera_name>
      <frame_name>camera_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

### LiDAR Sensor

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

## Part 5: Complete Robot Example

A simple humanoid URDF:

```xml
<?xml version="1.0"?>
<robot name="humanoid">
  <!-- Torso -->
  <link name="torso">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <box size="0.2 0.2 0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="10"/>
      <inertia ixx="0.1" iyy="0.1" izz="0.1" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Head -->
  <link name="head">
    <visual>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.1"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Neck joint -->
  <joint name="neck_joint" type="revolute">
    <parent link="torso"/>
    <child link="head"/>
    <origin xyz="0 0 0.3" rpy="0 0 0"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1.57" upper="1.57" effort="10" velocity="1.0"/>
  </joint>

  <!-- Left arm -->
  <link name="left_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="left_arm_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 1.57 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="1.57"/>
  </joint>

  <!-- Right arm (mirror of left) -->
  <link name="right_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="right_arm_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_arm"/>
    <origin xyz="-0.15 0 0.1" rpy="0 1.57 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="1.57"/>
  </joint>

  <!-- Left leg -->
  <link name="left_leg">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3"/>
      <inertia ixx="0.02" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="left_leg_joint" type="revolute">
    <parent link="torso"/>
    <child link="left_leg"/>
    <origin xyz="0.05 0 -0.35" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.57"/>
  </joint>

  <!-- Right leg -->
  <link name="right_leg">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
    </visual>
    <collision>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="3"/>
      <inertia ixx="0.02" iyy="0.02" izz="0.02" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <joint name="right_leg_joint" type="revolute">
    <parent link="torso"/>
    <child link="right_leg"/>
    <origin xyz="-0.05 0 -0.35" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.57"/>
  </joint>
</robot>
```

---

## Part 6: Best Practices

✅ **Do**:
- Use meaningful link/joint names
- Specify realistic masses
- Compute inertia carefully
- Add damping to joints
- Comment complex sections

❌ **Don't**:
- Use zero mass
- Mix units (stick to SI: meters, kg, rad)
- Create disconnected links
- Ignore joint limits

---

## Summary

**URDF Definition**:
- XML-based robot description
- Links = rigid bodies
- Joints = connections
- Sensors via Gazebo plugins

**Next**: Sensors in Gazebo simulation.

---

## Navigation

- **Previous**: [Chapter 1: Gazebo Basics](./01-gazebo-basics-and-physics.md)
- **Next**: [Chapter 3: Sensors in Gazebo](./03-sensors-in-gazebo.md)
- **Lab**: [Lab 2.1: Load Robot in Gazebo](./lab-2-1-load-robot-in-gazebo.md)
