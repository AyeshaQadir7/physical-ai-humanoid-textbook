---
id: module-1-lab-1-1
title: "Lab 1.1: Your First ROS 2 Node"
sidebar_position: 7
sidebar_label: "Lab 1.1: First Node"
description: "Build a publisher and subscriber - your first ROS 2 nodes"
keywords: [lab, ROS 2, publisher, subscriber, talker, listener, first node]
---

# Lab 1.1: Your First ROS 2 Node

## Lab Objective

Build two ROS 2 nodes:
1. **Talker** (publisher): Publishes "Hello ROS 2" messages
2. **Listener** (subscriber): Receives and prints messages

**Success Criteria**:
- Talker publishes message every 1 second ✅
- Listener receives and prints each message ✅
- Both nodes run without errors ✅

---

## Prerequisites

- Ubuntu 22.04 or WSL 2
- ROS 2 Humble installed ([Install Guide](https://docs.ros.org/en/humble/Installation.html))
- Terminal and text editor
- ~30 minutes

---

## Part 1: Setup

### 1.1 Create Workspace

```bash
# Create directory structure
mkdir -p ros2_ws/src
cd ros2_ws

# Download example code (or create it below)
git clone https://github.com/ros/examples
cd src
git clone -b humble https://github.com/ros2-examples/examples.git
cd ../..
```

**Or create from scratch**:

```bash
mkdir -p ros2_ws/src
cd ros2_ws
```

### 1.2 Create Package

```bash
cd src
ros2 pkg create --build-type ament_python my_first_ros2_pkg
cd ..
```

**Output**: New directory `src/my_first_ros2_pkg/` with template files.

---

## Part 2: Write Talker Node

### 2.1 Create Publisher Node

**File**: `src/my_first_ros2_pkg/my_first_ros2_pkg/talker.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello ROS 2: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - done automatically when node is garbage collected)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 2.2 Register Executable

Edit `setup.py`:

```python
entry_points={
    'console_scripts': [
        'talker = my_first_ros2_pkg.talker:main',
        'listener = my_first_ros2_pkg.listener:main',  # Add later
    ],
},
```

---

## Part 3: Write Listener Node

### 3.1 Create Subscriber Node

**File**: `src/my_first_ros2_pkg/my_first_ros2_pkg/listener.py`

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - done automatically when node is garbage collected)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Part 4: Build Package

```bash
cd ~/ros2_ws

# Build the package
colcon build --packages-select my_first_ros2_pkg

# Source the workspace
source install/setup.bash
```

**Output**:
```
Starting >>> my_first_ros2_pkg
Finished <<< my_first_ros2_pkg [1.23s]

Summary: 1 package finished [1.23s]
```

---

## Part 5: Run Nodes

### 5.1 Terminal 1: Run Talker

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_first_ros2_pkg talker
```

**Expected Output**:
```
[INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 0"
[INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 1"
[INFO] [minimal_publisher]: Publishing: "Hello ROS 2: 2"
...
```

### 5.2 Terminal 2: Run Listener

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run my_first_ros2_pkg listener
```

**Expected Output**:
```
[INFO] [minimal_subscriber]: I heard: "Hello ROS 2: 0"
[INFO] [minimal_subscriber]: I heard: "Hello ROS 2: 1"
[INFO] [minimal_subscriber]: I heard: "Hello ROS 2: 2"
...
```

---

## Part 6: Verification Checklist

- [ ] Talker node publishes messages every ~0.5 seconds
- [ ] Listener node receives all published messages
- [ ] Both nodes run without errors
- [ ] No "topic not found" errors
- [ ] Listener message count matches talker message count

---

## Part 7: Inspect with ROS 2 CLI

In a third terminal:

```bash
# List active nodes
ros2 node list
# Output:
# /minimal_publisher
# /minimal_subscriber

# List topics
ros2 topic list
# Output:
# /topic
# /parameter_events
# /rosout

# Show topic details
ros2 topic info /topic
# Output:
# Type: std_msgs/String
# Publisher count: 1
# Subscription count: 1

# Echo topic messages
ros2 topic echo /topic
# Output:
# data: 'Hello ROS 2: 0'
# ---
# data: 'Hello ROS 2: 1'
# ---

# Show message rate
ros2 topic hz /topic
# Output:
# average rate: 2.000 Hz
```

---

## Part 8: Troubleshooting

| Issue | Solution |
|-------|----------|
| `Package not found` | Run `source install/setup.bash` in new terminal |
| `No module named...` | Rebuild: `colcon build --symlink-install` |
| `Topic not found` | Make sure both nodes are running in separate terminals |
| Listener not receiving | Check ROS_DOMAIN_ID (default 0), should be same on both |
| Build fails | Check Python version: `python3 --version` (need 3.10+) |

---

## Part 9: Experiment

Try these modifications:

1. **Change publication rate**: Edit talker.py, change `timer_period = 0.5` to `0.1` (10 Hz)
2. **Change message**: Edit talker.py, change message text
3. **Add another subscriber**: Copy listener to `listener2.py`, run another instance
4. **View message data**: Run `ros2 topic echo /topic --format yaml`

---

## Summary

You've created:
- ✅ Publisher node (talker) sending messages
- ✅ Subscriber node (listener) receiving messages
- ✅ Topic-based communication between nodes

**Key Concepts**:
- Nodes communicate via topics
- Publishers send data
- Subscribers receive data
- Both can run independently

**What's Next**: Lab 1.2 teaches **services** for request-reply communication.

---

## References

- [ROS 2 Official Tutorial](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Py-Publisher-And-Subscriber.html)
- [ROS 2 Command Cheat Sheet](../assets/ros2-cheatsheet.md) (if available)

---

## Navigation

- **Previous Chapter**: [Chapter 6: Launch Files & Parameters](./06-launch-files-and-parameters.md)
- **Next Lab**: [Lab 1.2: Implement a Service](./lab-1-2-implement-a-service.md)
- **Module 1 Summary**: [Chapter 7: Module Summary](./summary.md)
