---
id: module-2-lab-1
title: "Lab 2.1: Load Robot in Gazebo"
sidebar_position: 10
sidebar_label: "Lab 2.1"
description: "Load humanoid URDF in Gazebo and verify physics simulation"
keywords: [Gazebo, URDF, physics, simulation, lab]
---

# Lab 2.1: Load Robot in Gazebo

## Lab Objective

**Goal**: Load a humanoid robot URDF into Gazebo and verify physics simulation works correctly.

**Skills**: URDF loading, Gazebo launch, physics verification, debugging.

**Time**: 45 minutes

---

## Prerequisites

- ✅ Module 1 complete (ROS 2 basics)
- ✅ Module 2 Chapters 1-2 read
- ✅ Gazebo installed: `gazebo --version` works
- ✅ ROS 2 Humble installed
- ✅ Text editor ready (VS Code, nano, vim)

---

## Setup (5 minutes)

### 1.1: Create a workspace

```bash
mkdir -p ~/robotics_ws/src
cd ~/robotics_ws
```

### 1.2: Create a ROS 2 package

```bash
cd src
ros2 pkg create --build-type ament_cmake my_robot
cd ..
colcon build
source install/setup.bash
```

### 1.3: Create directories

```bash
mkdir -p ~/robotics_ws/src/my_robot/urdf
mkdir -p ~/robotics_ws/src/my_robot/launch
mkdir -p ~/robotics_ws/src/my_robot/worlds
```

---

## Step 1: Create a Simple URDF (10 minutes)

Create `~/robotics_ws/src/my_robot/urdf/humanoid.urdf`:

```xml
<?xml version="1.0"?>
<robot name="humanoid">
  <!-- Base link (torso) -->
  <link name="base_link">
    <visual>
      <geometry>
        <box size="0.2 0.2 0.5"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 1"/>
      </material>
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
        <sphere radius="0.08"/>
      </geometry>
      <material name="yellow">
        <color rgba="1 1 0 1"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <sphere radius="0.08"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="2"/>
      <inertia ixx="0.01" iyy="0.01" izz="0.01" ixy="0" ixz="0" iyz="0"/>
    </inertial>
  </link>

  <!-- Joint: neck -->
  <joint name="neck_joint" type="revolute">
    <parent link="base_link"/>
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
      <material name="red">
        <color rgba="1 0 0 1"/>
      </material>
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
    <parent link="base_link"/>
    <child link="left_arm"/>
    <origin xyz="0.15 0 0.1" rpy="0 1.57 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-3.14" upper="3.14" effort="20" velocity="1.57"/>
  </joint>

  <!-- Right arm (mirror) -->
  <link name="right_arm">
    <visual>
      <geometry>
        <cylinder radius="0.05" length="0.4"/>
      </geometry>
      <material name="green">
        <color rgba="0 1 0 1"/>
      </material>
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
    <parent link="base_link"/>
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
      <material name="cyan">
        <color rgba="0 1 1 1"/>
      </material>
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
    <parent link="base_link"/>
    <child link="left_leg"/>
    <origin xyz="0.05 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.57"/>
  </joint>

  <!-- Right leg -->
  <link name="right_leg">
    <visual>
      <geometry>
        <cylinder radius="0.06" length="0.5"/>
      </geometry>
      <material name="magenta">
        <color rgba="1 0 1 1"/>
      </material>
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
    <parent link="base_link"/>
    <child link="right_leg"/>
    <origin xyz="-0.05 0 -0.3" rpy="0 0 0"/>
    <axis xyz="1 0 0"/>
    <limit lower="-1.57" upper="1.57" effort="50" velocity="1.57"/>
  </joint>
</robot>
```

---

## Step 2: Create a Gazebo Launch File (10 minutes)

Create `~/robotics_ws/src/my_robot/launch/gazebo.launch.xml`:

```xml
<?xml version="1.0"?>
<launch>
  <!-- Gazebo server -->
  <include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py">
    <arg name="world" value="$(find-pkg-share my_robot)/worlds/robot_world.sdf"/>
  </include>

  <!-- Spawn robot -->
  <node pkg="gazebo_ros" exec="spawn_entity.py"
        args="-entity humanoid -file $(find-pkg-share my_robot)/urdf/humanoid.urdf
               -x 0 -y 0 -z 0.5"
        output="screen"/>

  <!-- RViz for visualization -->
  <node pkg="rviz2" exec="rviz2" name="rviz2" output="screen"/>
</launch>
```

---

## Step 3: Create a Gazebo World (5 minutes)

Create `~/robotics_ws/src/my_robot/worlds/robot_world.sdf`:

```xml
<?xml version="1.0"?>
<sdf version="1.10">
  <world name="robot_world">
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
    </physics>

    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>10 10 10 0 0 0</pose>
    </light>

    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>100 100</size>
            </plane>
          </geometry>
          <material>
            <ambient>0.8 0.8 0.8 1</ambient>
            <diffuse>0.8 0.8 0.8 1</diffuse>
          </material>
        </visual>
      </link>
    </model>
  </world>
</sdf>
```

---

## Step 4: Build and Launch (10 minutes)

```bash
cd ~/robotics_ws
colcon build
source install/setup.bash
ros2 launch my_robot gazebo.launch.xml
```

**Expected output**:
- Gazebo window opens
- Colorful robot appears with arms, legs, head
- Robot falls under gravity
- Lands on ground plane

---

## Step 5: Verify Physics (5 minutes)

In Gazebo GUI:

1. **Apply forces**: Click "Select Model" → Click robot
2. **Drag robot**: Observe it moves and responds
3. **View joints**: In RViz, add "Robot Model" display → see URDF tree
4. **Check stability**: Robot should rest stably on ground

---

## Expected Output

```
Robot in Gazebo with:
✓ Torso (blue box)
✓ Head (yellow sphere) - can rotate
✓ Arms (red/green cylinders) - can swing
✓ Legs (cyan/magenta cylinders) - can move
✓ Gravity applied (robot falls initially)
✓ Physics stable (no oscillation)
```

---

## Verification Checklist

- [ ] Gazebo window opens with robot visible
- [ ] Robot falls naturally under gravity
- [ ] Robot rests on ground without oscillating
- [ ] Joint limits enforced (arms don't overextend)
- [ ] RViz shows robot model correctly
- [ ] Can interact with robot in Gazebo (drag it)

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| "URDF not found" | Check file path, ensure `.urdf` extension |
| Robot falls through floor | Check collision geometry and ground plane |
| Robot oscillates | Increase physics solver iterations |
| Gazebo very slow | Run headless: `GAZEBO_GUI_PLUGIN_PATH="" gazebo --headless` |
| Graphics not rendering | Update GPU drivers |

---

## Extension Activities

1. **Add more joints**: Create a knee joint for more realistic legs
2. **Add sensors**: Add camera or LiDAR (Chapter 3)
3. **Customize appearance**: Change colors, add textures
4. **Create obstacles**: Add boxes or walls to the world

---

## Summary

**Lab 2.1 accomplishes**:
- Load URDF into Gazebo ✓
- Verify physics simulation ✓
- Inspect robot structure in RViz ✓
- Foundation for Labs 2.2-2.3 ✓

---

## Navigation

- **Previous**: [Chapter 5: Sim-to-Real](./05-sim-to-real-considerations.md)
- **Next Lab**: [Lab 2.2: Publish Sensor Data](./lab-2-2-publish-sensor-data.md)
