---
id: module-2-gazebo-basics
title: "Gazebo Basics & Physics"
sidebar_position: 1
sidebar_label: "Gazebo Basics"
description: "Master Gazebo simulation engine, physics engines, and creating 3D robot worlds"
keywords: [Gazebo, physics, simulation, ODE, Bullet, DART, ROS 2, worlds, models]
---

# Gazebo Basics & Physics

## Introduction

**Gazebo** is the industry-standard robot simulator. It runs physics simulations, renders 3D graphics, and integrates seamlessly with ROS 2.

This chapter teaches:
- What Gazebo is and why robots use it
- Physics engines (ODE, Bullet, DART)
- Creating simulation worlds
- Spawning robots and objects
- Physics properties (gravity, friction, collisions)
- Integrating Gazebo with ROS 2

By the end, you'll have a Gazebo world running with a robot controlled via ROS 2 commands.

---

## Learning Outcomes

By the end of this chapter, you will:
1. Understand Gazebo architecture and simulation loop
2. Choose appropriate physics engines for your use case
3. Create Gazebo worlds (SDF format)
4. Spawn robots and objects into simulation
5. Configure physics (gravity, friction, collision)
6. Launch Gazebo from ROS 2 launch files
7. Debug Gazebo simulations

---

## Part 1: What is Gazebo?

### Gazebo Overview

**Gazebo** is a robot simulator that:
- Simulates physics (gravity, collisions, friction, inertia)
- Renders 3D environments
- Publishes sensor data (camera, LiDAR, IMU) to ROS 2 topics
- Receives motor commands from ROS 2 nodes
- Runs at high frequency (~1000 Hz physics, ~60 Hz graphics)

### Gazebo vs. Other Simulators

| Simulator | Strength | Weakness | Use Case |
|-----------|----------|----------|----------|
| **Gazebo** | ROS 2 integration, physics | Learning curve | Research, robotics education |
| **Isaac Sim** | High fidelity, synthetic data | Requires powerful GPU | Industrial, high-quality data |
| **CoppeliaSim** | User-friendly, flexible | Smaller community | Education, prototyping |
| **Webots** | Full learning suite | Less ROS integration | Education |

**For this course**: We use **Gazebo** (free, open-source, ROS 2 native).

---

## Part 2: Gazebo Architecture

### The Simulation Loop

Gazebo runs a physics loop at ~1000 Hz:

```
┌─────────────────────────────────────┐
│ 1. Apply ROS 2 commands             │
│    (motor velocities, joint forces) │
└─────────────────────────────────────┘
        ↓
┌─────────────────────────────────────┐
│ 2. Physics Step                     │
│    (Newton's laws, collisions)      │
└─────────────────────────────────────┘
        ↓
┌─────────────────────────────────────┐
│ 3. Compute Sensor Data              │
│    (camera renders, lidar casts rays)│
└─────────────────────────────────────┘
        ↓
┌─────────────────────────────────────┐
│ 4. Publish to ROS 2 Topics          │
│    (/camera/image, /scan, /tf)      │
└─────────────────────────────────────┘
        ↓
┌─────────────────────────────────────┐
│ 5. Render Graphics (60 Hz)          │
│    (display to screen)              │
└─────────────────────────────────────┘
```

### Key Components

- **Physics Engine**: Computes dynamics (ODE, Bullet, DART)
- **Rendering Engine**: OpenGL graphics
- **Sensor Simulation**: Camera, LiDAR, IMU plugins
- **ROS 2 Bridge**: Publishes/subscribes to ROS 2 topics
- **GUI**: 3D visualization and controls

---

## Part 3: Physics Engines

### Physics Engine Choices

Gazebo supports three physics engines. Each has tradeoffs:

#### ODE (Open Dynamics Engine)
- **Pros**: Stable, good for legged robots, default
- **Cons**: Slower than others
- **Use when**: Humanoid locomotion, complex contacts

#### Bullet
- **Pros**: Fast, good for rigid bodies
- **Cons**: Less stable with complex constraints
- **Use when**: Speed matters, simple objects

#### DART
- **Pros**: Very stable, handles complex structures
- **Cons**: Slower, requires compilation
- **Use when**: Precision dynamics, grasping

**Recommendation**: Use **ODE** for this course (default, stable for humanoids).

### Setting Physics in SDF

```xml
<?xml version="1.0"?>
<sdf version="1.10">
  <world name="robot_world">
    <!-- Physics engine -->
    <physics type="ode">
      <!-- Gravity (m/s²) -->
      <gravity>0 0 -9.81</gravity>

      <!-- Simulation step size (seconds) -->
      <max_step_size>0.001</max_step_size>

      <!-- Iterations per step (higher = more accurate) -->
      <real_time_update_rate>1000</real_time_update_rate>

      <!-- ODE-specific settings -->
      <ode>
        <solver type="quick">
          <!-- Constraint solver iterations -->
          <iters>50</iters>
          <!-- Pre-constraint force calculation -->
          <precon_iters>0</precon_iters>
          <!-- Use SSOR or Projected Gauss-Seidel -->
          <sor>1.3</sor>
        </solver>
      </ode>
    </physics>

    <!-- Rest of world definition -->
  </world>
</sdf>
```

---

## Part 4: Creating Gazebo Worlds

### SDF Format (Simulation Description Format)

Gazebo worlds are defined in **SDF** (XML-based):

```xml
<?xml version="1.0"?>
<sdf version="1.10">
  <world name="robot_world">
    <!-- Physics -->
    <physics type="ode">
      <gravity>0 0 -9.81</gravity>
      <max_step_size>0.001</max_step_size>
    </physics>

    <!-- Sun for lighting -->
    <light type="directional" name="sun">
      <cast_shadows>true</cast_shadows>
      <pose>10 10 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
    </light>

    <!-- Ground plane -->
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

    <!-- Add objects, walls, robot URDF here -->
  </world>
</sdf>
```

---

## Part 5: Spawning Models

### Method 1: Include URDF in World

```xml
<!-- Inside your world -->
<include>
  <uri>model://my_robot</uri>
  <pose>0 0 0.5 0 0 0</pose>
</include>
```

### Method 2: Spawn via ROS 2 Service

```bash
# Convert URDF to SDF and spawn
ros2 run gazebo_ros spawn_entity.py -entity robot \
  -file /path/to/robot.urdf \
  -x 0 -y 0 -z 0.5
```

### Method 3: Spawn via Launch File

```xml
<!-- In your launch file -->
<node pkg="gazebo_ros" exec="spawn_entity.py"
      args="-entity robot -file $(find-pkg-share my_pkg)/urdf/robot.urdf
             -x 0 -y 0 -z 0.5"
      output="screen"/>
```

---

## Part 6: Physics Properties

### Friction

Friction affects how objects slide:

```xml
<link name="wheel">
  <collision name="collision">
    <geometry>
      <sphere>
        <radius>0.05</radius>
      </sphere>
    </geometry>
    <!-- Friction properties -->
    <surface>
      <contact>
        <collide_bitmask>0xffff</collide_bitmask>
      </contact>
      <friction>
        <ode>
          <mu>0.7</mu>  <!-- Coefficient of friction -->
          <mu2>0.7</mu2>
          <fdir1>1 0 0</fdir1>  <!-- Friction direction -->
        </ode>
      </friction>
    </surface>
  </collision>
  <visual name="visual">
    <geometry>
      <sphere>
        <radius>0.05</radius>
      </sphere>
    </geometry>
    <material>
      <ambient>0 0 0 1</ambient>
      <diffuse>0 0 0 1</diffuse>
    </material>
  </visual>
</link>
```

**Friction coefficient guide**:
- **0.0**: Very slippery (ice)
- **0.5**: Normal surface
- **1.0+**: Rough surface (rubber on concrete)

### Inertia

Inertia affects how objects respond to forces:

```xml
<link name="body">
  <inertial>
    <mass>5.0</mass>  <!-- kg -->
    <!-- Inertia matrix (usually auto-computed) -->
    <inertia>
      <ixx>0.1</ixx>
      <iyy>0.1</iyy>
      <izz>0.1</izz>
      <ixy>0.0</ixy>
      <ixz>0.0</ixz>
      <iyz>0.0</iyz>
    </inertia>
  </inertial>
</link>
```

---

## Part 7: Running Gazebo with ROS 2

### Basic Launch

```xml
<?xml version="1.0"?>
<launch>
  <!-- Start Gazebo server (headless) -->
  <include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py">
    <arg name="world" value="$(find-pkg-share my_robot)/worlds/robot_world.sdf"/>
  </include>

  <!-- Start RViz for visualization (optional) -->
  <node pkg="rviz2" exec="rviz2" name="rviz2" output="screen">
    <param name="rviz_config" value="$(find-pkg-share my_robot)/config/rviz.rviz"/>
  </node>

  <!-- Spawn robot in Gazebo -->
  <node pkg="gazebo_ros" exec="spawn_entity.py"
        args="-entity robot -file $(find-pkg-share my_robot)/urdf/robot.urdf
               -x 0 -y 0 -z 0.5"
        output="screen"/>
</launch>
```

**Run it**:
```bash
ros2 launch my_robot robot.launch.xml
```

---

## Part 8: Troubleshooting

### Robot Stability Issues

| Problem | Cause | Solution |
|---------|-------|----------|
| Robot oscillates | Physics step too large | Reduce `max_step_size` |
| Robot falls through ground | Bad collision shapes | Check URDF collision geometry |
| Robot too slow | Gravity wrong | Verify `<gravity>0 0 -9.81</gravity>` |
| Joint wobbles | Low inertia | Increase mass or adjust inertia |

### Performance Issues

**Gazebo running slowly?**:
1. Reduce physics update rate (lower accuracy, faster)
2. Lower ODE solver iterations
3. Reduce sensor update rates
4. Close GUI, run headless

**Command**:
```bash
# Run Gazebo without GUI (faster)
GAZEBO_GUI_PLUGIN_PATH="" gazebo --headless my_world.sdf
```

---

## Summary

**Gazebo Basics**:
- Physics loop runs at ~1000 Hz
- ODE engine is default and stable
- Worlds defined in SDF (XML)
- Models spawned via URDF or SDF
- Integrates with ROS 2 via plugins

**Physics Configuration**:
- Gravity, step size, solver iterations
- Friction (mu coefficient)
- Inertia (mass, moment of inertia)
- Contact properties

**ROS 2 Integration**:
- Launch files start Gazebo
- Spawn entities via service
- Sensor plugins publish to ROS 2 topics
- Motor commands received from ROS 2

---

## Next Steps

Next chapter: **URDF & Robot Descriptions** - defining your robot's structure for Gazebo.

---

## Navigation

- **Previous**: [Module 2 Intro](./intro.md)
- **Next**: [Chapter 2: URDF & Robot Descriptions](./02-urdf-and-robot-descriptions.md)
- **Lab**: [Lab 2.1: Load Robot in Gazebo](./lab-2-1-load-robot-in-gazebo.md)
