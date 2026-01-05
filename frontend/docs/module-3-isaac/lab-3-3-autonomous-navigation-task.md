---
id: module-3-lab-3
title: "Lab 3.3: Autonomous Navigation Task"
sidebar_position: 12
sidebar_label: "Lab 3.3"
description: "Program robot to autonomously navigate to goals while avoiding obstacles"
keywords: [navigation, path planning, goal-seeking, Nav2, lab]
---

# Lab 3.3: Autonomous Navigation Task

## Lab Objective

**Goal**: Program robot to navigate autonomously to goal poses while avoiding obstacles.

**Skills**: Nav2 framework, goal publishing, path planning validation.

**Time**: 45 minutes

---

## Prerequisites

- ✅ Lab 3.2 complete (SLAM working)
- ✅ Nav2 installed: `sudo apt install ros-humble-nav2-bringup`
- ✅ Cost map understanding (Chapter 3)

---

## Step 1: Launch Nav2 (10 minutes)

Create `navigation.launch.xml`:

```xml
<?xml version="1.0"?>
<launch>
  <!-- SLAM (from Lab 3.2) -->
  <include file="slam.launch.xml"/>

  <!-- Nav2 -->
  <include file="$(find-pkg-share nav2_bringup)/launch/navigation_launch.py">
    <arg name="use_sim_time" value="true"/>
    <arg name="map_file" value="map.yaml"/>
    <arg name="params_file" value="nav2_params.yaml"/>
  </include>

  <!-- RViz with Nav2 config -->
  <node pkg="rviz2" exec="rviz2" name="rviz2"
        args="-d nav2_rviz.rviz"/>
</launch>
```

Launch:

```bash
ros2 launch my_robot navigation.launch.xml
```

---

## Step 2: Set Navigation Goal via RViz (10 minutes)

In RViz GUI:

1. **Click "2D Goal Pose"** button (top toolbar)
2. **Click on map** where you want robot to go
3. **Drag to orient** robot direction
4. **Release** to send goal

Robot starts moving autonomously!

---

## Step 3: Monitor Navigation Progress (10 minutes)

```python
#!/usr/bin/env python3
import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class NavigationMonitor(rclpy.node.Node):
    def __init__(self):
        super().__init__('nav_monitor')
        self.subscription = self.create_subscription(
            Path,
            '/plan',  # Global plan
            self.plan_callback,
            10
        )

    def plan_callback(self, msg):
        num_poses = len(msg.poses)
        if num_poses > 0:
            final_pose = msg.poses[-1]
            x = final_pose.pose.position.x
            y = final_pose.pose.position.y
            self.get_logger().info(f"Plan: {num_poses} waypoints, goal at ({x:.2f}, {y:.2f})")

if __name__ == '__main__':
    rclpy.init()
    node = NavigationMonitor()
    rclpy.spin(node)
```

Run:

```bash
python3 navigation_monitor.py
```

---

## Step 4: Publish Navigation Goals Programmatically (10 minutes)

Instead of RViz GUI, use ROS 2:

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient

class NavigationGoalPublisher(rclpy.node.Node):
    def __init__(self):
        super().__init__('goal_publisher')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def send_goal(self, x, y, theta):
        """Send goal to Nav2"""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)

        # Orientation from theta
        from math import sin, cos
        goal_msg.pose.pose.orientation.z = sin(theta/2)
        goal_msg.pose.pose.orientation.w = cos(theta/2)

        self.action_client.send_goal_async(goal_msg)
        self.get_logger().info(f"Sent goal: ({x}, {y})")

def main():
    rclpy.init()
    publisher = NavigationGoalPublisher()

    # Sequence of navigation goals
    goals = [
        (1.0, 0.0, 0.0),    # Move right
        (1.0, 1.0, 0.785),  # Move up-right (45°)
        (0.0, 1.0, 1.57),   # Move up
        (0.0, 0.0, 0.0),    # Return to origin
    ]

    for x, y, theta in goals:
        publisher.send_goal(x, y, theta)
        rclpy.spin_once(publisher, timeout_sec=5)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run:

```bash
python3 goal_publisher.py
```

---

## Step 5: Test Obstacle Avoidance (10 minutes)

1. **Add obstacle** in Isaac Sim (drag cube into path)
2. **Send goal** through obstacle's original location
3. **Observe**: Robot avoids obstacle and replans path

Expected behavior:
- Robot detects obstacle in costmap
- Planner finds alternate route
- Robot navigates around obstacle
- Reaches goal successfully

---

## Step 6: Measure Navigation Metrics (5 minutes)

```python
def measure_navigation_performance():
    """Measure success rate and path efficiency"""
    num_goals = 10
    successes = 0
    path_lengths = []
    times = []

    for i in range(num_goals):
        goal = random_goal()
        start_time = time.time()
        path = send_goal(goal)

        if path is not None:
            successes += 1
            path_lengths.append(len(path))
            times.append(time.time() - start_time)

    success_rate = successes / num_goals
    avg_length = np.mean(path_lengths) if path_lengths else 0
    avg_time = np.mean(times) if times else 0

    print(f"Success rate: {success_rate*100:.1f}%")
    print(f"Avg path length: {avg_length:.1f} waypoints")
    print(f"Avg navigation time: {avg_time:.1f}s")

    # Target metrics:
    # - Success rate: >90%
    # - Path efficiency: short paths
    # - Speed: Under 30s per goal
```

---

## Expected Output

```
Navigation working when:
✓ Robot reaches goal without manual intervention
✓ Path avoids all obstacles
✓ Multiple goals executed in sequence
✓ Success rate >90%
✓ Replans when obstacle appears
```

---

## Verification Checklist

- [ ] Nav2 starts without errors
- [ ] Goals received and processed
- [ ] Cost map shows obstacles
- [ ] Path planner generates feasible paths
- [ ] Robot follows path accurately
- [ ] Obstacle avoidance working
- [ ] Success rate >90% on test goals

---

## Troubleshooting

| Problem | Solution |
|---------|----------|
| Robot doesn't move | Check SLAM is running |
| Path invalid | Increase inflation radius |
| Oscillates near goal | Adjust controller parameters |
| Cost map empty | Verify LiDAR publishing |

---

## Extension

1. **Multiple robots**: Nav2 with swarms
2. **Dynamic obstacles**: Moving people
3. **Timed goals**: Sequential missions
4. **Metric evaluation**: Full benchmark

---

## Summary

**Lab 3.3 accomplishes**:
- Nav2 navigation working ✓
- Goal-seeking implementation ✓
- Obstacle avoidance verified ✓
- Path planning validated ✓

**Next**: Module 4 (Voice control integration)

---

## Navigation

- **Previous Lab**: [Lab 3.2: SLAM](./lab-3-2-implement-slam-pipeline.md)
- **Next Module**: [Module 4: VLA](../module-4-vla/intro.md)
