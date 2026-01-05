---
id: module-3-lab-2
title: "Lab 3.2: Implement SLAM Pipeline"
sidebar_position: 11
sidebar_label: "Lab 3.2"
description: "Build visual SLAM system and verify localization and mapping"
keywords: [SLAM, localization, mapping, visual odometry, lab]
---

# Lab 3.2: Implement SLAM Pipeline

## Lab Objective

**Goal**: Build a visual SLAM system in Isaac Sim and verify robot localization and mapping.

**Skills**: SLAM implementation, feature tracking, map building, odometry validation.

**Time**: 60 minutes

---

## Prerequisites

- ✅ Lab 3.1 complete (Isaac Sim environment)
- ✅ ROS 2 SLAM tools installed
- ✅ Understanding of visual odometry (Chapter 3)

---

## Step 1: Install SLAM Tools (10 minutes)

```bash
# Install ORB-SLAM
sudo apt install ros-humble-slam-toolbox

# Install visualization
sudo apt install ros-humble-rtabmap-ros

# Verify
ros2 pkg list | grep slam
```

---

## Step 2: Configure SLAM Node (10 minutes)

Create `slam_config.yaml`:

```yaml
# SLAM parameters
use_sim_time: true
qos_overrides:
  "/camera/image_raw": {"durability": "transient_local"}

orb_slam2:
  camera_matrix:
    fx: 500.0
    fy: 500.0
    cx: 320.0
    cy: 240.0
  baseline: 0.0  # Monocular
  feature_threshold: 100
  scale_factor: 1.2
  n_levels: 8
```

---

## Step 3: Launch SLAM Node (10 minutes)

Create `slam.launch.xml`:

```xml
<?xml version="1.0"?>
<launch>
  <!-- SLAM node -->
  <node pkg="slam_toolbox" exec="async_slam_toolbox_node" name="slam">
    <param name="use_sim_time" value="true"/>
    <param name="config_file" value="slam_config.yaml"/>
    <remap from="scan" to="/scan"/>
  </node>

  <!-- TF broadcaster for map → base_link -->
  <node pkg="tf2_ros" exec="static_transform_publisher"
        args="0 0 0 0 0 0 map base_link"/>
</launch>
```

Launch it:

```bash
ros2 launch my_robot slam.launch.xml
```

---

## Step 4: Move Robot and Observe Mapping (15 minutes)

In Isaac Sim, manually move robot using keyboard or publish commands:

```bash
# Terminal 1: Move arm to observe features
ros2 topic pub /joint_commands std_msgs/Float32MultiArray "data: [0.5, 0.5, 0.5]"

# Terminal 2: Monitor SLAM
ros2 topic echo /slam_toolbox/graph_visualization --once
```

### 4.1: Verify Mapping

```bash
# Check map being built
ros2 topic list | grep slam

# Should see:
# /slam_toolbox/graph_visualization
# /slam_toolbox/map
```

---

## Step 5: Verify Odometry (15 minutes)

Subscribe to odometry:

```python
#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Odometry
import math

class OdometryMonitor(rclpy.node.Node):
    def __init__(self):
        super().__init__('odom_monitor')
        self.subscription = self.create_subscription(
            Odometry,
            '/slam_toolbox/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        distance = math.sqrt(x**2 + y**2 + z**2)
        self.get_logger().info(f"Odometry: ({x:.2f}, {y:.2f}, {z:.2f}) dist={distance:.2f}m")

if __name__ == '__main__':
    rclpy.init()
    node = OdometryMonitor()
    rclpy.spin(node)
```

Run:

```bash
python3 odometry_monitor.py
```

**Expected output**:
```
[INFO] Odometry: (0.00, 0.00, 0.00) dist=0.00m
[INFO] Odometry: (0.05, 0.02, 0.00) dist=0.05m
[INFO] Odometry: (0.10, 0.04, 0.00) dist=0.10m
...
```

---

## Step 6: Detect Loop Closure (10 minutes)

Move robot to previously visited area:

1. Start at position A
2. Move to position B
3. Return to position A (loop closure)

```bash
# Monitor loop closure detection
ros2 topic echo /slam_toolbox/loop_closure --once

# When loop is detected:
# message published with connection info
```

---

## Step 7: Visualize Map (10 minutes)

View map in RViz:

```bash
ros2 run rviz2 rviz2

# In RViz GUI:
# 1. Add "Map" display
# 2. Topic: /slam_toolbox/map
# 3. See occupancy grid
```

**Visual**:
- White: free space
- Gray: unmapped
- Black: obstacles

---

## Expected Output

```
SLAM working correctly when:
✓ Odometry updating as robot moves
✓ Map building incrementally
✓ Loop closures detected
✓ Map looks reasonable (obstacles in right places)
✓ No major drift over 5+ minutes
```

---

## Verification Checklist

- [ ] SLAM node starts without errors
- [ ] `/slam_toolbox/odom` topic publishing at >10 Hz
- [ ] Map building (visual features detected)
- [ ] Loop closure detected on revisit
- [ ] RViz map visualization shows correct layout
- [ ] Odometry error Under 5% over 1 minute

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| No odometry published | Check camera image quality |
| Map deformed | Reduce feature threshold |
| Loop closure not detected | Increase similarity threshold |
| SLAM crashes | Check Isaac Sim camera fps |

---

## Extension

1. **Quantify error**: Compare SLAM vs. ground truth
2. **Add IMU**: Improve odometry with inertial data
3. **Larger environment**: Test in complex scenes

---

## Summary

**Lab 3.2 accomplishes**:
- SLAM pipeline running ✓
- Localization working (odometry) ✓
- Mapping building incrementally ✓
- Loop closure validation ✓

**Ready for**: Lab 3.3 (autonomous navigation)

---

## Navigation

- **Previous Lab**: [Lab 3.1: Environment](./lab-3-1-create-isaac-sim-environment.md)
- **Next Lab**: [Lab 3.3: Navigation](./lab-3-3-autonomous-navigation-task.md)
