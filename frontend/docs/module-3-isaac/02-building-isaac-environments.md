---
id: module-3-environments
title: "Building Isaac Environments"
sidebar_position: 2
sidebar_label: "Environments"
description: "Creating photorealistic scenes with robots, objects, lighting, and sensors"
keywords: [Isaac Sim, scene creation, objects, sensors, lighting, robotics]
---

# Building Isaac Environments

## Introduction

Now build photorealistic worlds for your robot to perceive and navigate.

This chapter covers:
- Creating scenes in Isaac Sim
- Importing robots and objects
- Configuring sensors
- Setting physics parameters
- Lighting for perception

---

## Learning Outcomes

By the end, you will:
1. Create Isaac Sim scenes with robots
2. Add and position objects
3. Configure camera and LiDAR sensors
4. Set realistic lighting
5. Export for ROS 2 integration

---

## Part 1: Creating Scenes

### Basic Scene Setup

```python
from omni.isaac.kit import SimulationApp
app = SimulationApp()

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid, StaticPlane

# Create world
world = World()

# Add ground plane
world.scene.add_ground_plane()

# Add objects
cube = world.scene.add(
    DynamicCuboid(name="cube",
                  size=np.array([0.1, 0.1, 0.1]),
                  color=np.array([1, 0, 0]))
)

while True:
    world.step()
```

### Importing Assets

```python
from omni.isaac.core import World

world = World()

# Import humanoid robot from Module 2
humanoid_usd = "/path/to/humanoid.usd"
robot = world.scene.add(
    Articulation(name="humanoid",
                 usd_path=humanoid_usd,
                 position=[0, 0, 0.5])
)

# Import furniture from Nucleus
furniture_usd = "omniverse://nvidia/assets/Isaac/Environments/Table/table_low.usd"
table = world.scene.add(
    XFormPrim(name="table",
              usd_path=furniture_usd)
)
```

---

## Part 2: Adding Sensors

### Camera Sensor

```python
from omni.isaac.sensor import Camera

camera = world.scene.add(
    Camera(name="camera",
           parent=robot.get_link("head"),
           position=[0, 0, 0.1],
           orientation=[0, 0, 0, 1],
           width=640,
           height=480,
           frequency=30)
)

# Get RGB image
rgb = camera.get_rgb()  # [640, 480, 3]
depth = camera.get_depth()  # [640, 480] in meters
```

### LiDAR Sensor

```python
from omni.isaac.sensor import LidarRtx

lidar = world.scene.add(
    LidarRtx(name="lidar",
             parent=robot.get_link("head"),
             position=[0, 0, 0.2],
             rotation=[0, 0, 0],
             frequency=10,
             channels=64,
             range_min=0.1,
             range_max=100.0,
             horizontal_fov=360.0,
             vertical_fov=30.0)
)

# Get point cloud
points = lidar.get_current_frame()  # [N, 3] XYZ
```

---

## Part 3: Physics Configuration

### Setting Physics Parameters

```python
from omni.isaac.core import World

world = World(physics_dt=1/1000, rendering_dt=1/60)

# Set gravity
world.scene.set_world_gravity([0, 0, -9.81])

# Configure material properties
physics_context = world.get_physics_context()
physics_context.set_gravity([0, 0, -9.81])
physics_context.set_default_material_static_friction(0.5)
physics_context.set_default_material_dynamic_friction(0.5)
```

### Dynamic vs. Static Objects

```python
# Dynamic object (moves under physics)
dynamic_cube = world.scene.add(
    DynamicCuboid(name="cube",
                  mass=1.0,
                  size=[0.1, 0.1, 0.1])
)

# Static object (doesn't move)
static_table = world.scene.add(
    VisualCuboid(name="table",
                 size=[1.0, 1.0, 0.05],
                 color=[0.5, 0.5, 0.5])
)
static_table.get_prim().GetAttribute("physics:kinematicEnabled").Set(False)
```

---

## Part 4: Lighting

### Lighting Configuration

```python
from pxr import UsdLux, Gf

# Directional light (sun)
light_prim = UsdLux.DistantLight.Define(stage, "/World/Light")
light_prim.GetAngleAttr().Set(0.5)
light_prim.GetIntensityAttr().Set(1500)

# Point light (lamp)
point_light = UsdLux.SphereLight.Define(stage, "/World/Lamp")
point_light.GetRadiusAttr().Set(0.1)
point_light.GetIntensityAttr().Set(500)

# Configure for perception (photorealistic)
light_prim.GetDiffuseAttr().Set(Gf.Vec3f(1, 1, 1))
light_prim.GetSpecularAttr().Set(Gf.Vec3f(1, 1, 1))
```

### Domain Randomization

```python
import random

def randomize_scene():
    """Randomize lighting and object positions"""
    # Random lighting
    intensity = random.uniform(1000, 2000)
    light_prim.GetIntensityAttr().Set(intensity)

    # Random object positions
    cube.set_world_position([
        random.uniform(-1, 1),
        random.uniform(-1, 1),
        0.5
    ])

    # Random camera pose
    camera.set_world_position([
        random.uniform(-0.5, 0.5),
        random.uniform(-0.5, 0.5),
        1.5
    ])

# Use in synthetic data generation loop
for i in range(10000):
    randomize_scene()
    image = camera.get_rgb()
    save_image(image, f"synthetic_{i}.png")
```

---

## Part 5: Complete Scene Example

```python
from omni.isaac.kit import SimulationApp
from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.sensor import Camera, LidarRtx

app = SimulationApp()
world = World()

# Add ground and sky
world.scene.add_ground_plane()

# Add humanoid robot
robot = world.scene.add(
    Articulation(name="humanoid",
                 usd_path="humanoid.usd",
                 position=[0, 0, 0.5])
)

# Add camera on head
camera = world.scene.add(
    Camera(name="camera",
           parent=robot.get_link("head"),
           position=[0, 0, 0.1],
           width=640, height=480)
)

# Add LiDAR on head
lidar = world.scene.add(
    LidarRtx(name="lidar",
             parent=robot.get_link("head"),
             position=[0, 0, 0.2],
             channels=64)
)

# Add objects to grasp
for i in range(3):
    world.scene.add(
        DynamicCuboid(name=f"cube_{i}",
                      size=[0.05, 0.05, 0.05],
                      position=[i*0.2, 0, 0.5])
    )

# Simulation loop
frame = 0
while True:
    world.step()

    # Get sensor data
    image = camera.get_rgb()
    depth = camera.get_depth()
    points = lidar.get_current_frame()

    # Publish to ROS 2 (in next chapter)
    # ros_camera_pub.publish(image)
    # ros_lidar_pub.publish(points)

    frame += 1
    if frame == 1000:
        break
```

---

## Summary

**Scene creation**:
- Import robots (URDF/USD)
- Add objects and furniture
- Configure physics
- Set up sensors

**Sensors**:
- Camera: RGB, depth, resolution
- LiDAR: Range, channels, FOV
- Both GPU-accelerated for speed

**Lighting**:
- Directional light (sun)
- Point lights (lamps)
- Domain randomization for robustness

**Next**: SLAM and autonomous navigation.

---

## Navigation

- **Previous**: [Chapter 1: Isaac Overview](./01-isaac-sim-overview-and-workflows.md)
- **Next**: [Chapter 3: SLAM & Navigation](./03-slam-and-autonomous-navigation.md)
