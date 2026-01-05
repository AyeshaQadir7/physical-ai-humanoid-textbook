---
id: module-3-lab-1
title: "Lab 3.1: Create Isaac Sim Environment"
sidebar_position: 10
sidebar_label: "Lab 3.1"
description: "Set up photorealistic Isaac Sim world with humanoid robot and sensors"
keywords: [Isaac Sim, environment, sensors, photorealism, lab]
---

# Lab 3.1: Create Isaac Sim Environment

## Lab Objective

**Goal**: Create a photorealistic Isaac Sim environment with your humanoid robot, cameras, and LiDAR sensors.

**Skills**: Isaac Sim scene creation, sensor configuration, photorealistic rendering.

**Time**: 60 minutes

---

## Prerequisites

- ✅ Module 2 complete (understand URDF and Gazebo)
- ✅ NVIDIA GPU with 8+ GB VRAM
- ✅ Isaac Sim installed
- ✅ Humanoid URDF from Module 2

---

## Step 1: Create Isaac Sim Project (10 minutes)

### 1.1: Launch Isaac Sim

```bash
# Start Isaac Sim
~/.local/share/ov/pkg/isaac-sim-*/isaac-sim.sh
```

The Isaac Sim GUI opens with a blank stage.

### 1.2: Create New Scene

```
File → New
Omnivery → New Stage

Save as: humanoid_world.usd
```

---

## Step 2: Import Humanoid Robot (10 minutes)

### 2.1: Load URDF

```
File → Import
Select: humanoid.urdf (from Module 2)

Position: x=0, y=0, z=0.5
```

The colorful humanoid appears in the viewport!

### 2.2: Verify in Viewport

- Robot visible: ✓
- All links present: ✓ (torso, head, arms, legs)
- Colors correct: ✓

---

## Step 3: Add Camera Sensor (10 minutes)

### 3.1: Create Camera Prim

Right-click in outliner:
```
Create → Xform Prim → Rename to "camera"
Position: (0, 0, 0.15) relative to head
```

### 3.2: Add Camera Component

Select camera prim:
```
Create → Camera
Resolution: 640×480
Focal Length: 50mm
```

### 3.3: Configure for Rendering

```
Create → Projection
Focal Length: 24mm (wide-angle)
```

---

## Step 4: Add LiDAR Sensor (10 minutes)

### 4.1: Create LiDAR RTX

```
Create → Xform (lidar)
Position: (0, 0, 0.25) relative to head
```

### 4.2: Configure LiDAR Parameters

```python
# In script console:
lidar_prim = stage.GetPrimAtPath("/World/lidar")
lidar_prim.GetProperty("rotationsPerSecond").Set(10)
lidar_prim.GetProperty("horizontalFov").Set(360)
lidar_prim.GetProperty("verticalFov").Set(30)
lidar_prim.GetProperty("channels").Set(64)
```

---

## Step 5: Add Scene Objects (10 minutes)

### 5.1: Ground Plane

Already present (default).

### 5.2: Add Table

```
Create → Cube
Name: table
Scale: (1, 1, 0.05)
Position: (0.5, 0, 0.4)
Material: Wood texture
```

### 5.3: Add Objects to Grasp

```
# Create 3 cubes for grasping practice
for i in range(3):
    Create → Cube
    Name: cube_{i}
    Size: 0.1×0.1×0.1
    Position: (0.5 + i*0.15, 0, 0.55)
    Color: Red, Green, Blue
```

---

## Step 6: Configure Lighting (10 minutes)

### 6.1: Add Directional Light (Sun)

```
Create → DomeLight
Intensity: 1500
Color Temperature: 6500K (daylight)
```

### 6.2: Add Point Light (Lamp)

```
Create → SphereLight
Intensity: 500
Position: (1, 1, 2)
```

---

## Step 7: Configure Physics (10 minutes)

### 7.1: Set Physics Engine

```
Window → Physics → Enable
Physics Engine: PhysX
Gravity: (0, 0, -9.81)
Timestep: 0.001s
```

### 7.2: Set Material Properties

```python
# Apply to all objects
friction_coefficient = 0.7
restitution = 0.2
density = 1000  # kg/m³
```

---

## Step 8: Enable ROS 2 Bridge (10 minutes)

### 8.1: Load ROS 2 Extension

```
Window → Extensions
Search: "ros2"
Enable: "Isaac ROS 2 Bridge"
```

### 8.2: Configure Topics

```python
# Camera topic
camera_prim.GetProperty("rosTopicName").Set("/camera/image_raw")

# LiDAR topic
lidar_prim.GetProperty("rosTopicName").Set("/scan")

# Joint states
robot_prim.GetProperty("rosTopicName").Set("/joint_states")
```

---

## Step 9: Run Simulation (5 minutes)

### 9.1: Play Simulation

```
Press Play button (▶)
```

**Verify**:
- Robot doesn't fall through ground: ✓
- Camera renders image: ✓
- LiDAR generates point cloud: ✓
- Physics stable: ✓

---

## Step 10: Verify Sensor Output (5 minutes)

In terminal:

```bash
# Check ROS 2 topics
ros2 topic list | grep -E "camera|scan|joint"

# Expected output:
# /camera/image_raw
# /scan
# /joint_states
```

### 10.1: View Camera Image

```bash
ros2 run rviz2 rviz2
# Add "Image" display → Topic: /camera/image_raw
```

### 10.2: View LiDAR

```bash
ros2 run rviz2 rviz2
# Add "LaserScan" display → Topic: /scan
```

---

## Expected Output

```
Isaac Sim viewport shows:
✓ Colorful humanoid robot
✓ Ground plane
✓ Table with 3 cubes
✓ Camera and LiDAR visible
✓ Lighting realistic and clear
✓ Objects stable (no oscillation)

ROS 2 topics streaming:
✓ /camera/image_raw (640×480 RGB)
✓ /scan (64-channel LiDAR)
✓ /joint_states (robot joint positions)
```

---

## Verification Checklist

- [ ] Isaac Sim launches without GPU errors
- [ ] Humanoid URDF loads correctly
- [ ] Camera renders photorealistic image
- [ ] LiDAR scans environment
- [ ] Physics stable (objects don't jitter)
- [ ] ROS 2 topics publishing
- [ ] RViz visualizations work

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| GPU memory error | Reduce resolution to 512×384 |
| Physics unstable | Increase substeps to 4 |
| Camera black | Check lighting configuration |
| LiDAR no points | Check LiDAR range (min/max) |
| ROS 2 bridge not working | Restart Isaac Sim and ROS 2 |

---

## Extension Activities

1. **Add more objects**: Shelves, walls, furniture
2. **Change lighting**: Day/night cycles, shadows
3. **Add IMU sensor**: Complete sensor suite
4. **Record dataset**: Generate synthetic training data
5. **Add robots**: Multiple humanoids in scene

---

## Summary

**Lab 3.1 accomplishes**:
- Create photorealistic Isaac Sim environment ✓
- Import humanoid robot ✓
- Configure camera and LiDAR sensors ✓
- Enable ROS 2 integration ✓
- Verify sensor data streams ✓

**Ready for**: Lab 3.2 (SLAM implementation)

---

## Navigation

- **Previous**: [Chapter 5: Detection](./05-object-detection-and-manipulation.md)
- **Next Lab**: [Lab 3.2: SLAM Pipeline](./lab-3-2-implement-slam-pipeline.md)
