---
id: module-3-slam
title: "SLAM & Autonomous Navigation"
sidebar_position: 3
sidebar_label: "SLAM & Navigation"
description: "Simultaneous Localization and Mapping, path planning, and autonomous goal-seeking"
keywords: [SLAM, Visual Odometry, localization, mapping, navigation, path planning, RRT, Dijkstra]
---

# SLAM & Autonomous Navigation

## Introduction

**SLAM** allows robots to localize and map unknown environments. **Navigation** lets them reach goals autonomously.

This chapter covers:
- SLAM concepts (localization + mapping)
- Visual odometry and feature tracking
- Path planning algorithms
- Nav2 ROS 2 framework
- Integration with perception

---

## Learning Outcomes

By the end, you will:
1. Understand SLAM architecture
2. Implement visual SLAM pipelines
3. Apply path planning algorithms
4. Use Nav2 for autonomous navigation
5. Validate navigation in simulation

---

## Part 1: SLAM Concepts

### What is SLAM?

**SLAM** = Simultaneous Localization and Mapping

Two problems solved together:
1. **Localization**: Where am I? (estimate robot pose)
2. **Mapping**: What's around me? (build map of environment)

### The Challenge

```
Chicken-and-egg problem:
- To localize, need a map
- To build map, need to know where you are
- SLAM solves both simultaneously!

Solution: Bundle adjustment
- Optimize both robot pose and landmark positions
- Use sensor measurements as constraints
```

### SLAM Loop

```
1. Camera observes features
2. Match features to previous frames
3. Estimate camera motion (ego-motion)
4. Add camera pose to map
5. Add observed features as landmarks
6. Loop closure: detect revisited area
7. Optimize poses and landmarks
```

---

## Part 2: Visual Odometry

### Feature-Based Tracking

```python
import cv2
import numpy as np

# ORB features (fast, rotation-invariant)
orb = cv2.ORB_create(nfeatures=500)

frame1 = cv2.imread("frame1.png", cv2.IMREAD_GRAYSCALE)
frame2 = cv2.imread("frame2.png", cv2.IMREAD_GRAYSCALE)

# Find keypoints
kp1, des1 = orb.detectAndCompute(frame1, None)
kp2, des2 = orb.detectAndCompute(frame2, None)

# Match features
matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = matcher.match(des1, des2)

# Estimate motion using matched features
src_pts = np.float32([kp1[m.queryIdx].pt for m in matches]).reshape(-1, 1, 2)
dst_pts = np.float32([kp2[m.trainIdx].pt for m in matches]).reshape(-1, 1, 2)

E, mask = cv2.findEssentialMat(src_pts, dst_pts, K, cv2.RANSAC, 0.999, 1.0)
```

### Visual SLAM Algorithms

| Algorithm | Method | Accuracy | Speed |
|-----------|--------|----------|-------|
| **ORB-SLAM** | Feature-based | High | Fast |
| **DSO** | Direct | High | Slow |
| **VINS-Mono** | VIO (visual-inertial) | High | Real-time |
| **Isaac VSLAM** | Hardware-accelerated | High | Very fast |

---

## Part 3: Loop Closure Detection

### Problem: Drift

```
As robot moves:
- Odometry accumulates error
- Estimated pose drifts from true pose
- Map becomes distorted

Solution: Loop closure
- Detect when returning to known area
- Correct accumulated error
- Optimize entire trajectory
```

### Loop Closure Algorithm

```python
def detect_loop_closure(current_frame, previous_frames, threshold=0.7):
    """
    Compare current frame to previous frames.
    If similarity > threshold → loop closure detected
    """
    current_desc = get_descriptors(current_frame)

    similarities = []
    for prev_frame in previous_frames:
        prev_desc = get_descriptors(prev_frame)
        similarity = compute_similarity(current_desc, prev_desc)
        similarities.append(similarity)

    if max(similarities) > threshold:
        loop_frame_idx = np.argmax(similarities)
        return loop_frame_idx  # Loop closure!
    return None  # No loop closure
```

---

## Part 4: Path Planning

### Dijkstra's Algorithm (Optimal)

```python
import heapq

def dijkstra(graph, start, goal):
    """Find shortest path from start to goal"""
    open_set = [(0, start)]  # (cost, node)
    came_from = {}
    cost_so_far = {start: 0}

    while open_set:
        current_cost, current = heapq.heappop(open_set)

        if current == goal:
            # Reconstruct path
            path = [goal]
            while goal in came_from:
                goal = came_from[goal]
                path.append(goal)
            return path[::-1]

        for neighbor, move_cost in graph[current]:
            new_cost = cost_so_far[current] + move_cost
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                came_from[neighbor] = current
                heapq.heappush(open_set, (new_cost, neighbor))

    return None  # No path found
```

**Pros**: Optimal path
**Cons**: Slow for large graphs

### RRT (Rapidly-Exploring Random Tree) - Fast

```python
import random

def rrt(start, goal, obstacles, max_iterations=5000):
    """RRT path planning - fast but not optimal"""
    tree = [start]
    parent = {start: None}

    for _ in range(max_iterations):
        # Random point in space
        if random.random() < 0.1:  # 10% toward goal
            random_point = goal
        else:
            random_point = (random.uniform(-5, 5), random.uniform(-5, 5))

        # Nearest point in tree
        nearest = min(tree, key=lambda p: distance(p, random_point))

        # Extend toward random point
        new_point = extend(nearest, random_point, step_size=0.5)

        if not collides_with_obstacle(new_point, obstacles):
            tree.append(new_point)
            parent[new_point] = nearest

            if distance(new_point, goal) < 0.5:
                # Reached goal!
                path = [goal]
                current = new_point
                while current is not None:
                    path.append(current)
                    current = parent[current]
                return path[::-1]

    return None  # No path found
```

**Pros**: Fast for high dimensions
**Cons**: Not optimal, may be suboptimal

---

## Part 5: Nav2 Integration

### Nav2 Architecture

```
Sensor data (camera, LiDAR)
    ↓
SLAM node (produces map + odometry)
    ↓
Cost map (obstacles + free space)
    ↓
Path planner (Dijkstra or RRT)
    ↓
Controller (move robot along path)
    ↓
Robot motion
```

### NAV2 Launch File

```xml
<?xml version="1.0"?>
<launch>
  <!-- SLAM -->
  <node pkg="slam_toolbox" exec="async_slam_toolbox_node" name="slam">
    <param name="use_sim_time" value="true"/>
  </node>

  <!-- Nav2 -->
  <include file="$(find-pkg-share nav2_bringup)/launch/navigation_launch.py">
    <arg name="use_sim_time" value="true"/>
    <arg name="params_file" value="nav2_params.yaml"/>
  </include>

  <!-- Move Base to send navigation goals -->
  <node pkg="nav2_core" exec="simple_navigator_node" name="navigator"/>
</launch>
```

### Navigation Goal via ROS 2

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class Navigator:
    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node('navigator')
        self.action_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def navigate_to(self, x, y, theta):
        """Send goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = theta

        self.action_client.send_goal_async(goal_msg)

if __name__ == '__main__':
    navigator = Navigator()
    navigator.navigate_to(x=1.0, y=2.0, theta=0.0)  # Go to (1,2)
```

---

## Part 6: Validation Metrics

### Localization Error

```python
def compute_localization_error(estimated_poses, ground_truth_poses):
    """Compare estimated vs. actual poses"""
    errors = []
    for est, gt in zip(estimated_poses, ground_truth_poses):
        error = np.linalg.norm(est - gt)
        errors.append(error)
    return np.mean(errors), np.max(errors)

# Target: Under 5% error
```

### Map Quality

```python
def evaluate_map(estimated_map, ground_truth_map):
    """Evaluate map accuracy"""
    # Alignment (ICP - Iterative Closest Point)
    # Completeness (how much of true map is in estimate)
    # Correctness (how much of estimate is correct)
```

---

## Summary

**SLAM**:
- Visual odometry estimates motion
- Landmarks build map
- Loop closure corrects drift

**Path planning**:
- Dijkstra: Optimal but slow
- RRT: Fast but suboptimal
- Nav2: ROS 2 navigation framework

**Next**: Isaac ROS hardware acceleration.

---

## Navigation

- **Previous**: [Chapter 2: Environments](./02-building-isaac-environments.md)
- **Next**: [Chapter 4: Isaac ROS Integration](./04-isaac-ros-integration.md)
