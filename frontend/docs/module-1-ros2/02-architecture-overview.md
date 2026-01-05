---
id: ros2-architecture-overview
title: "ROS 2 Architecture Overview"
sidebar_position: 2
sidebar_label: "Chapter 1.1: Architecture Overview"
description: "Master ROS 2 core architecture: nodes, topics, services, actions, DDS middleware, QoS, message types, and the complete communication stack. Understand why ROS 2 is the industrial standard for robot middleware."
keywords: [ROS 2, architecture, DDS, middleware, nodes, topics, services, actions, QoS, publish-subscribe, message types, type safety]
---

# Chapter 1.1: ROS 2 Architecture Overview

## Introduction

Before you write your first line of ROS 2 code, you need to understand *how it works at a fundamental level*. ROS 2 is not just a library—it's a complete ecosystem for building distributed robot systems.

In this chapter, you'll learn:
- **What ROS 2 is** and how it differs from ROS 1
- **The core components**: Nodes, topics, services, and actions
- **The DDS middleware layer**: Why Data Distribution Service, quality of service, discovery
- **Message types**: How data is structured and validated
- **The node graph**: How components connect and communicate
- **Real-world robotics patterns**: When to use topics, services, and actions

By the end of this chapter, you'll understand the architecture deeply enough to design robust robot systems and debug distributed communication problems.

---

## Learning Outcomes

By the end of this chapter, you will be able to:

1. **Understand the ROS 2 architecture and its components**
   - Explain the relationship between nodes, topics, services, and actions
   - Describe the layered architecture (application, ROS 2 client library, DDS, network)
   - Identify which communication pattern solves which robotics problem
   - Understand node lifecycles and initialization

2. **Grasp DDS middleware concepts (publish/subscribe, QoS)**
   - Explain why ROS 2 chose Data Distribution Service over custom middleware
   - Configure QoS profiles for different reliability requirements
   - Understand topic discovery and dynamic systems
   - Apply best practices for network resilience

3. **Learn node graph concepts and communication patterns**
   - Visualize and interpret ROS 2 computation graphs
   - Design topic naming hierarchies for large systems
   - Identify bottlenecks and single points of failure
   - Debug connectivity issues between nodes

4. **Compare ROS 1 vs ROS 2 evolution**
   - Explain the architectural changes from ROS 1 to ROS 2
   - Understand why these changes matter for production robots
   - Recognize ROS 1 code patterns and their ROS 2 equivalents
   - Appreciate design decisions in ROS 2

5. **Understand message types and type safety**
   - Use built-in message types (std_msgs, sensor_msgs, geometry_msgs)
   - Design custom message types for domain-specific data
   - Validate message schemas at compile time
   - Handle message versioning and backward compatibility

---

## Part 1: What is ROS 2?

### Definition

**ROS 2** (Robot Operating System 2) is a **middleware framework for building distributed robot software**. It provides:

1. **Communication** between independent software components (nodes)
2. **Coordination** of hardware (sensors, actuators) with software
3. **Standard interfaces** for common robotics tasks (navigation, manipulation, perception)
4. **Developer tools** for debugging, logging, and visualization

It is **not**:
- An operating system (despite the name—it runs on top of Linux, Windows, or macOS)
- A programming language (you write in Python, C++, or Java)
- A physics simulator (though it integrates with Gazebo and Isaac Sim)
- A control framework (though it hosts control algorithms)

### Why the Name "Robot Operating System"?

The name is historical. ROS 1 (2007) was created to be the "operating system for robots"—a unified software platform that allowed researchers to share code. Today, ROS 2 is better described as **middleware**—software that sits between your application code and the hardware/network, providing abstractions for communication.

### The Core Insight: Distributed Systems First

ROS 2's central philosophy: **A robot is a distributed system.**

Think about what your humanoid robot needs to do simultaneously:
- Read camera frames (continuous, real-time)
- Process LiDAR scans (continuous, real-time)
- Compute IMU orientation (very fast, 100+ Hz)
- Run motion planning (slower, intermittent)
- Execute motor commands (continuous, high-priority)
- Log data for debugging (background task)
- Communicate with cloud services (occasional, can be slow)

**A single-threaded, monolithic program cannot handle all of this.** ROS 2 forces you to decompose your robot into independent components that communicate via message passing.

---

## Part 2: Core Components—The ROS 2 Building Blocks

### Component 1: Nodes

A **node** is the smallest executable unit in ROS 2. It's a self-contained program that:
- Publishes data to topics (acts as a sensor, processor, or controller)
- Subscribes to topics (receives data from other nodes)
- Provides services (answers requests from other nodes)
- Calls services (requests help from other nodes)
- Offers actions (executes long-running tasks with feedback)

**Example nodes in a humanoid robot:**

```
┌──────────────────────────────────────────────────────┐
│              Humanoid Robot System                   │
├──────────────────────────────────────────────────────┤
│                                                      │
│  Perception:                 Control:               │
│  ┌─────────────────┐        ┌──────────────────┐   │
│  │ camera_driver   │        │ motor_controller │   │
│  │ lidar_driver    │        │ joint_controller │   │
│  │ imu_driver      │        │ gripper_driver   │   │
│  └─────────────────┘        └──────────────────┘   │
│           │                         ▲               │
│           └─────┬──────────────────┘                │
│                 │                                   │
│        ┌────────▼──────────────┐                    │
│        │ world_model           │                    │
│        │ (fuses all sensors)   │                    │
│        └────────┬──────────────┘                    │
│                 │                                   │
│        ┌────────▼──────────────┐                    │
│        │ motion_planner        │                    │
│        │ (ROS 2 action server) │                    │
│        └───────────────────────┘                    │
│                                                      │
└──────────────────────────────────────────────────────┘
```

**Node Properties:**
- **Unique name**: `/motor_controller`, `/perception_pipeline` (names start with `/`)
- **Namespace**: Groups related nodes (`/robot_left_arm/joint_controller`)
- **Input**: Subscriptions, services, actions
- **Output**: Publishers, services, actions
- **Lifecycle**: Created, initialized, running, shutting down

### Component 2: Topics

A **topic** is a named data stream. Nodes *publish* data to topics; other nodes *subscribe* to receive that data.

**Why topics?** One-way, fire-and-forget communication. Perfect for:
- Streaming sensor data (camera frames, LiDAR scans)
- Logging data asynchronously
- Broadcasting state changes
- Real-time command streams

**Topic Example:**
```
Camera Node publishes to /camera/image topic
                │
                ├─→ Image Logger (subscribes, logs frames)
                ├─→ Object Detector (subscribes, detects objects)
                └─→ World Model (subscribes, updates 3D scene)
```

All three subscribers receive the camera frame **independently and asynchronously**.

**Topic characteristics:**
- **Multiple publishers**: Many nodes can publish to the same topic (rare, usually avoided)
- **Multiple subscribers**: Many nodes can subscribe to the same topic (common)
- **Asynchronous**: Publisher doesn't wait for subscribers to finish processing
- **Decoupled**: Publisher and subscribers don't need to know about each other
- **Typed**: All messages on a topic have the same structure (e.g., `sensor_msgs/Image`)

### Component 3: Services

A **service** is a synchronous request-response pattern. A node (the **service server**) waits for a request, processes it, and sends back a response.

**Why services?** Two-way, synchronous communication. Perfect for:
- One-time requests ("What is your current battery level?")
- Configuration changes ("Set gripper to 50% force")
- Queries with results ("Is there an obstacle at position X?")
- Atomic operations that must complete before proceeding

**Service Example:**
```
Motion Planner (client) requests:
  "Plan a path from my current position to the goal"
         │
         ▼
Path Planner Service (server):
  1. Receive request (start, goal positions)
  2. Process (compute path avoiding obstacles)
  3. Send response (path, success/failure)
         │
         ▼
Motion Planner:
  Waits for response, then executes path
```

**Service characteristics:**
- **Synchronous**: Client blocks until server responds
- **Request-response**: Client sends request; server must send response
- **Typed**: Both request and response have defined structures
- **One service per node type**: Multiple clients can call the same service

### Component 4: Actions

An **action** is an asynchronous request-response with **feedback and cancellation**.

**Why actions?** Long-running tasks that need progress updates. Perfect for:
- Navigation goals (robot moving to a target, reporting progress)
- Manipulation (gripper opening while reporting position)
- Motion execution (robot moving joints, streaming current angles)
- Any task that takes >100ms and needs cancellation

**Action Example:**
```
Navigation Client: "Navigate to coordinates (5, 10)"
         │
         ├─→ Action Server starts execution
         │
         ├─→ Server sends feedback: "Progress: 30% of path complete"
         │   (client receives, doesn't have to wait)
         │
         ├─→ Server sends feedback: "Progress: 60%"
         │
         ├─→ Server sends feedback: "Progress: 100% - Reached goal"
         │
         ├─→ Server sends final result: {success: true, final_pose: ...}
         │
         └─→ Client receives result

(At any point, client can send "CANCEL" command)
```

**Action characteristics:**
- **Asynchronous**: Client doesn't block on every message
- **Feedback streams**: Server sends progress updates
- **Goal preemption**: Client can cancel and submit new goal
- **State machine**: Clear states (idle → executing → succeeded/aborted)

### Comparison: Topics vs. Services vs. Actions

| Criterion | Topic | Service | Action |
| --- | --- | --- | --- |
| **Communication Type** | One-way broadcast | Two-way request-response | Two-way with feedback |
| **Synchronization** | Asynchronous | Synchronous (waiter blocks) | Asynchronous (non-blocking) |
| **Use Case** | Streaming data (sensors) | One-time requests (queries) | Long-running tasks (motion) |
| **Feedback** | Continuous (multiple messages) | Single response | Continuous feedback + final result |
| **Cancellation** | N/A | N/A | Supported (cancel goal) |
| **Latency Tolerance** | Can drop old messages | Must complete | Can stream real-time progress |
| **Example** | `/camera/image` publishes frames | `/gripper/set_force` service | `/navigate_to` action |

---

## Part 3: The DDS Middleware Layer

### Why Not Just Use TCP/IP?

You might wonder: "Why don't we just use TCP sockets directly?"

**ROS 1 (2007–2020) did exactly that.**

**Problems with custom TCP implementation:**

1. **No automatic discovery**: You had to manually tell each node where to find other nodes (IP, port)
2. **Reliability issues**: Dropped packets, connection failures; custom handling needed
3. **Not real-time capable**: TCP is designed for best-effort delivery, not deterministic control
4. **Single master point of failure**: A central master node registered all nodes; if it crashed, the system failed
5. **Network inflexibility**: Didn't adapt to bandwidth limits, latency, or reliability requirements

### The Solution: Data Distribution Service (DDS)

**DDS** (Data Distribution Service) is an **industry-standard middleware** specified by the Object Management Group (OMG). It's used in aerospace, defense, robotics, and autonomous systems because it's:

- **Designed for real-time systems**: Guarantees latency bounds
- **Decentralized**: No master node required (peers discover each other)
- **Flexible**: Adapts communication based on QoS requirements
- **Industry-proven**: Used in commercial robots (Boston Dynamics, Tesla, Unitree)
- **Multiple implementations**: You can swap Fast-RTPS for Connext without changing application code

### DDS Core Concepts

#### 1. Automatic Discovery

When your node starts, DDS automatically finds other nodes on the network:

```
Node A: "Hello, I'm /camera_node. I publish to /camera/image."
Node B: "Hello, I'm /image_processor_node. I subscribe to /camera/image."
DDS: "Great! I'm connecting them."
```

**How it works:**
- Each node announces itself with a UDP broadcast (multicast)
- Nodes periodically refresh announcements
- If a node crashes, its announcement expires
- No central master needed

#### 2. Quality of Service (QoS)

DDS lets you specify how messages should be delivered. ROS 2 exposes QoS settings:

**Reliability**:
- `BEST_EFFORT`: Drop messages if network is congested (best for real-time streams like video)
- `RELIABLE`: Retry until delivered (best for critical commands)

**History**:
- `KEEP_LAST(N)`: Store only the last N messages (memory efficient)
- `KEEP_ALL`: Store every message (risk of memory growth)

**Durability**:
- `VOLATILE`: Messages exist only while subscribers are connected
- `TRANSIENT_LOCAL`: New subscribers receive the last message (useful for configuration)

**Example**: Configuring camera publishing

```python
qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,  # Video is OK to drop
    history=QoSHistoryPolicy.KEEP_LAST,            # Only keep last frame
    depth=1,                                        # Depth=1 = keep 1 frame
)
self.publisher = self.create_publisher(Image, '/camera/image', qos_profile)
```

**QoS Compatibility:**
- A subscriber with `RELIABLE` can **not** receive from a publisher with `BEST_EFFORT`
- QoS profiles must **match** for communication to work
- This prevents silent failures and bugs

### DDS in the ROS 2 Stack

```
┌──────────────────────────────────┐
│  Your ROS 2 Node (Python)        │
│  (Uses rclpy library)            │
└──────────┬───────────────────────┘
           │
           ▼
┌──────────────────────────────────┐
│  DDS Middleware Layer            │
│  (Fast-RTPS, Connext, Eclipse)   │
│  - Handles pub/sub               │
│  - Handles discovery             │
│  - Enforces QoS                  │
└──────────┬───────────────────────┘
           │
           ▼
┌──────────────────────────────────┐
│  Operating System Network        │
│  (UDP/TCP sockets)               │
└──────────────────────────────────┘
```

**Key advantage**: Your application code doesn't need to know about networking. DDS handles it all.

---

## Part 4: Message Types—Structured Data

### Built-In Message Types

ROS 2 provides standard message types for common robotics data. Understanding these prevents reinventing the wheel.

#### std_msgs: Primitive Types

```python
# std_msgs/Bool.msg
bool data

# std_msgs/Int32.msg
int32 data

# std_msgs/String.msg
string data

# std_msgs/Float64.msg
float64 data
```

**When to use**: Simple scalar data (temperature, battery percentage, discrete commands)

#### sensor_msgs: Sensor Data

```python
# sensor_msgs/Image
std_msgs/Header header
uint32 height
uint32 width
string encoding  # e.g., "rgb8", "bgr8", "mono8"
uint8 is_bigendian
uint32 step  # bytes per row
uint8[] data  # raw pixel data

# sensor_msgs/PointCloud2
std_msgs/Header header
uint32 height
uint32 width
PointField[] fields  # e.g., x, y, z, intensity
uint8 is_bigendian
uint32 point_step  # bytes per point
uint32 row_step  # bytes per row
uint8[] data
bool is_dense

# sensor_msgs/Imu
std_msgs/Header header
geometry_msgs/Quaternion orientation
float64[9] orientation_covariance
geometry_msgs/Vector3 angular_velocity
float64[9] angular_velocity_covariance
geometry_msgs/Vector3 linear_acceleration
float64[9] linear_acceleration_covariance
```

**When to use**: Raw sensor data from cameras, LiDAR, IMU, encoders

#### geometry_msgs: Positions and Orientations

```python
# geometry_msgs/Point
float64 x
float64 y
float64 z

# geometry_msgs/Quaternion
float64 x
float64 y
float64 z
float64 w

# geometry_msgs/Pose (position + orientation)
geometry_msgs/Point position
geometry_msgs/Quaternion orientation

# geometry_msgs/Twist (linear and angular velocity)
geometry_msgs/Vector3 linear
geometry_msgs/Vector3 angular
```

**When to use**: Robot position, orientation (quaternions not Euler angles!), velocities, transformations

#### nav_msgs: Navigation Data

```python
# nav_msgs/Path
std_msgs/Header header
geometry_msgs/PoseStamped[] poses  # waypoints

# nav_msgs/OccupancyGrid
std_msgs/Header header
nav_msgs/MapMetaData info
int8[] data  # 0=free, 100=occupied, -1=unknown
```

**When to use**: Paths, maps, navigation state

### Custom Message Types

For domain-specific data, you define custom messages:

**File**: `my_robot_msgs/msg/GripperCommand.msg`
```
# Request to control the gripper
string action           # "open", "close", "grip"
float64 force_percent   # 0.0 to 100.0
```

**File**: `my_robot_msgs/msg/GripperState.msg`
```
# State of the gripper
float64 position_radians  # Current angle
float64 force             # Current force (Newtons)
bool is_moving            # Is gripper in motion?
```

**Usage in Python**:
```python
from my_robot_msgs.msg import GripperCommand, GripperState

# Publish a gripper command
cmd = GripperCommand()
cmd.action = "close"
cmd.force_percent = 75.0
self.gripper_pub.publish(cmd)

# Subscribe to gripper state
def state_callback(msg: GripperState):
    print(f"Gripper position: {msg.position_radians} rad")

self.gripper_sub = self.create_subscription(
    GripperState,
    '/gripper/state',
    state_callback,
    10
)
```

### Why Type Safety Matters

In ROS 1, messages were dynamically typed:
```python
# ROS 1: No type checking
msg = {"data": 3.14}  # Could be float, string, anything
```

In ROS 2, messages are **type-checked at compile time**:
```python
# ROS 2: Mypy will catch type errors
msg: Float64 = Float64(data=3.14)  # ✓ Correct
msg: Float64 = Float64(data="hello")  # ✗ Type error—compiler catches it
```

**Why this matters**: Subtle bugs are caught before runtime. If a motion planner expects `geometry_msgs/Pose` and a node publishes `std_msgs/String`, you'll know immediately instead of crashing mid-mission.

---

## Part 5: The Node Graph—Visualizing Your System

### What is a Computation Graph?

A **computation graph** shows all nodes in your system and how they communicate via topics. It's essential for understanding large distributed systems.

**Example: Humanoid Robot Perception and Motion**

```
Sensors:
┌──────────┐  ┌──────────┐  ┌────────┐
│ Camera   │  │ LiDAR    │  │ IMU    │
└────┬─────┘  └────┬─────┘  └───┬────┘
     │             │             │
     └─────────────┼─────────────┘
                   │
          ┌────────▼──────────┐
          │ World Model Node  │
          │ (sensor fusion)   │
          └────────┬──────────┘
                   │
                   │ /world_state
                   │
          ┌────────▼──────────────────┐
          │ Motion Planner Action Svr │
          └────────┬───────────────────┘
                   │
                   │ (action goal)
                   │
          ┌────────▼───────────────────┐
          │ Joint Trajectory Executor  │
          │ (Action Client + Control)  │
          └────────┬───────────────────┘
                   │
        ┌──────────┼──────────┐
        │          │          │
┌───────▼─┐  ┌──────▼──┐  ┌──▼────────┐
│ Left Arm│  │  Torso  │  │ Right Arm │
│Controller│  │Controller│  │ Controller│
└──────────┘  └─────────┘  └───────────┘
```

### Using ros2 graph

**Visualize your system:**
```bash
ros2 graph  # Opens interactive ROS computation graph viewer
```

Or generate a static graph:
```bash
ros2 graph | grep -E "^(node|topic)" > my_graph.txt
```

### Identifying Bottlenecks in Your Graph

**Scenario**: Your humanoid walks slowly.

1. Is the motion planner slow? Subscribe to `/motion_plan` and check timestamps
2. Is the joint controller slow? Subscribe to `/joint_states` and check update frequency
3. Is the sensor fusion slow? Subscribe to `/world_state` and check latency
4. Is the camera slow? `ros2 topic hz /camera/image` shows actual frame rate

Each topic in the graph is a measurement point for debugging.

---

## Part 6: ROS 1 vs. ROS 2—The Evolution

### Key Architectural Changes

| Aspect | ROS 1 | ROS 2 | Impact |
| --- | --- | --- | --- |
| **Middleware** | Custom TCP/UDP | DDS (industry standard) | ROS 2 interops with non-ROS systems |
| **Master Node** | Required (single point of failure) | No master (peer discovery) | ROS 2 is more resilient |
| **Real-Time Support** | Limited (not designed for it) | Full support (deterministic control) | Robots can react faster |
| **Type Safety** | Dynamic (runtime errors) | Static (compile-time errors) | Fewer bugs in production |
| **Security** | Minimal (research focus) | SROS2 (encryption, authentication) | Enterprise-grade security |
| **Memory Model** | Shared memory, threading | Message-passing, isolation | ROS 2 safer for multi-node systems |
| **Discovery** | Multicast to master | Distributed DDS discovery | ROS 2 works across firewalls |
| **Cross-Language** | Python, C++, limited others | Python, C++, Java, C# | ROS 2 more language-agnostic |

### Code Migration: ROS 1 → ROS 2

**ROS 1 Node (Minimal)**:
```python
import rospy
from std_msgs.msg import String

rospy.init_node('talker')
pub = rospy.Publisher('chatter', String, queue_size=10)
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    pub.publish("hello world")
    rate.sleep()
```

**ROS 2 Node (Equivalent)**:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        msg = String()
        msg.data = 'hello world'
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Key differences:**
- ROS 2 requires explicit node class (better scoping, reusability)
- ROS 2 uses timers instead of rate objects (more flexible)
- ROS 2 spin is a separate call (more control)

### Why ROS 1 Developers Love ROS 2

1. **Type safety**: Catches errors at compile time, not at 3 AM during field testing
2. **Real-time support**: Critical for humanoid balance, manipulation
3. **Deterministic**: Predictable latency helps with control
4. **Security**: SROS2 enables deployment in secure environments
5. **No master node**: System more resilient to failures

---

## Part 7: Real-World ROS 2 System Design

### Case Study: A Humanoid Pick-and-Place Task

**Goal**: Robot moves to a table, detects an object, picks it up, places it in a container.

**System Architecture:**

```
Hardware:
┌─────────┐  ┌──────────┐  ┌─────────┐
│ Camera  │  │ Gripper  │  │ Motors  │
└────┬────┘  └────┬─────┘  └────┬────┘
     │            │             │
     ├────────────┼─────────────┤
     │            │             │
Hardware Drivers (ROS 2 nodes)
├─ camera_driver    → /camera/image
├─ gripper_driver   → /gripper/state, ← /gripper/command
└─ motor_driver     → /joint_states, ← /joint_commands

Perception Pipeline:
     /camera/image
            │
     ┌──────▼──────────┐
     │ Object Detector │  → /detected_objects
     └──────────────────┘

Planning:
     /detected_objects + /joint_states
            │
     ┌──────▼──────────┐
     │ Pick Planner    │  → /pick_goal (action)
     └──────────────────┘

Execution:
     /pick_goal (action)
            │
     ┌──────▼──────────┐
     │ Action Server   │
     │ (motion exec)   │
     └───────┬─────────┘
             │
    ┌────────┼────────────┐
    ▼        ▼             ▼
/joint_commands /gripper_commands (service call)
```

### Why This Design Works

**Modularity**: Each component (perception, planning, control) is separate
- Can develop independently
- Can test in isolation
- Can replace camera driver without touching motion planning

**Resilience**: If object detector crashes, the rest keeps running
- Emergency stop still works
- Gripper can still open/close via service

**Scalability**: Add new perception (depth camera) without rewriting core system
- Just subscribe to new camera, add data to fusion

**Reusability**: The motion planner is independent of this task
- Use same planner for different goals (push, pour, wipe)
- Use same planner on different robots

---

## Part 8: Common Patterns and Anti-Patterns

### Pattern 1: Sensor Fusion
**Problem**: Multiple sensors measure the same thing (e.g., orientation from gyro and magnetometer)

**Solution**: Create a fusion node that subscribes to both, publishes filtered result

```
/imu/raw → ┐
           ├→ Kalman Filter Node → /imu/filtered
/mag/raw → ┘
```

### Pattern 2: Hierarchical Namespaces
**Problem**: Large robots have many joints; topics become unwieldy

**Solution**: Organize by body part

```
/robot/left_arm/joint_1/state
/robot/left_arm/joint_2/state
/robot/left_arm/gripper/state
/robot/right_arm/joint_1/state
...
```

### Anti-Pattern 1: Tightly Coupled Nodes
**Bad**:
```python
# Node A hardcodes that it expects Node B
node_b = NodeB()  # Can't run without Node B
node_b.process_data()  # Direct function call
```

**Good**:
```python
# Node A publishes to topic; doesn't care who subscribes
self.pub.publish(data)  # Works with any subscriber
```

### Anti-Pattern 2: Ignoring QoS
**Bad**:
```python
# Default QoS might not match reliability needs
self.pub = self.create_publisher(Sensor, topic, 10)
```

**Good**:
```python
# Explicitly set QoS for your use case
qos = QoSProfile(reliability=QoSReliabilityPolicy.RELIABLE)
self.pub = self.create_publisher(Sensor, topic, qos)
```

### Anti-Pattern 3: Blocking in a Callback
**Bad**:
```python
def sensor_callback(msg):
    # This blocks the entire node for 5 seconds!
    result = expensive_computation(msg)  # Takes 5 seconds
    self.pub.publish(result)
```

**Good**:
```python
def sensor_callback(msg):
    # Store message; process in separate thread
    self.queue.put(msg)

def processing_thread(self):
    # Runs independently; doesn't block callbacks
    while True:
        msg = self.queue.get()
        result = expensive_computation(msg)
        self.pub.publish(result)
```

---

## Summary: Architecture at a Glance

**ROS 2 is built on:**

1. **Nodes**: Independent executables that publish, subscribe, serve
2. **Topics**: Asynchronous one-way data streams for sensors and logging
3. **Services**: Synchronous request-response for queries and commands
4. **Actions**: Asynchronous request-response with feedback for long-running tasks
5. **DDS Middleware**: Handles discovery, pub/sub delivery, and QoS enforcement
6. **Type Safety**: Compile-time message validation prevents entire classes of bugs
7. **No Master Node**: Peer discovery makes the system resilient

**Design Principle**: Decompose your robot into independent nodes that communicate via messages. This makes systems modular, testable, and resilient.

---

## Practical Exercise: Designing a System

**Your Task**: You're building a mobile manipulation robot that:
1. Reads a color camera and depth camera
2. Detects objects of interest
3. Plans a motion to pick them up
4. Executes the motion and opens/closes the gripper

**Design the ROS 2 system:**
- What nodes would you create?
- What topics would nodes publish and subscribe to?
- What services would you use? What actions?
- Draw the computation graph

**Solution** (one possible design):

```
Nodes:
├─ camera_driver     (reads hardware, publishes images)
├─ object_detector   (subscribes to camera, publishes detected objects)
├─ motion_planner    (subscribes to objects, serves as action server)
├─ joint_controller  (subscribes to joint commands, controls motors)
└─ gripper_controller (serves service: open/close)

Topics:
├─ /camera/rgb_image
├─ /camera/depth_image
├─ /detected_objects
├─ /joint_commands
└─ /joint_states

Services:
└─ /gripper/control

Actions:
└─ /motion_planner/pick
```

(Compare your design to the full example in Chapter 1.2 after reading this.)

---

## Chapter Quiz

Test your understanding of ROS 2 architecture:

### Question 1: Core Components
**Which ROS 2 component would you use to stream camera images from a camera driver node to multiple subscriber nodes (image logger, object detector, SLAM)?**

a) Service (synchronous request-response)
b) Action (asynchronous task with feedback)
c) Topic (asynchronous one-way broadcast)
d) Parameter server (configuration)

<details>
<summary>Show Answer</summary>

**Correct Answer: C**

Topics are designed exactly for this use case:
- **One-way**: Camera publishes; multiple nodes subscribe
- **Asynchronous**: Subscribers process at their own pace
- **Decoupled**: Subscribers can be added/removed without touching the camera driver
- **Streaming**: Continuous data flow (unlike services which are one-off requests)

**Why others are wrong:**
- A: Services are for synchronous queries, not continuous streaming
- B: Actions are for long-running tasks with cancellation, not raw sensor streams
- D: Parameter server stores static configuration, not dynamic data

**Key insight**: Topics enable the publish-subscribe pattern—many receivers from one sender, all decoupled.

</details>

---

### Question 2: DDS Middleware
**Why did ROS 2 adopt DDS (Data Distribution Service) instead of using ROS 1's custom TCP/UDP implementation?**

a) DDS is faster than TCP
b) DDS is an industry standard with real-time guarantees, decentralized discovery, and QoS flexibility
c) DDS is easier to implement
d) ROS 2 developers just wanted to use something different

<details>
<summary>Show Answer</summary>

**Correct Answer: B**

DDS provides critical features for production robotics:
1. **Industry standard**: Proven in aerospace, defense, robotics (Bosch, Boston Dynamics, Tesla)
2. **Real-time guarantees**: Deterministic latency bounds (critical for humanoid balance)
3. **Decentralized**: No single master node (ROS 1's weakness)
4. **QoS flexibility**: Adapt communication to reliability vs. performance tradeoffs
5. **Interoperability**: Your ROS 2 nodes can communicate with non-ROS DDS systems

**Why others are wrong:**
- A: Speed isn't the primary driver; real-time guarantees are
- C: DDS is actually more complex to implement (which is why ROS 2 uses it—someone else did the hard work)
- D: This is a technical decision, not arbitrary

**Key insight**: DDS lets ROS 2 inherit production-grade reliability from aerospace/defense systems.

</details>

---

### Question 3: QoS Configuration
**A robot's balance controller needs to receive IMU orientation updates at 100 Hz with zero dropped messages. Which QoS settings would you choose?**

a) `BEST_EFFORT` reliability, `KEEP_LAST(1)` history
b) `RELIABLE` reliability, `KEEP_LAST(1)` history
c) `RELIABLE` reliability, `KEEP_ALL` history
d) `BEST_EFFORT` reliability, `KEEP_ALL` history

<details>
<summary>Show Answer</summary>

**Correct Answer: B**

Reasoning:
- **Reliability: RELIABLE**: Balance is safety-critical. Every IMU reading matters. If a message is lost, the controller doesn't know the robot is tilting.
- **History: KEEP_LAST(1)**: Only the most recent orientation matters. Older readings are stale and useless.

**Why others are wrong:**
- A: `BEST_EFFORT` will drop messages when network is congested → robot can fall
- C: `KEEP_ALL` wastes memory storing old orientations that are irrelevant (latency matters, not history)
- D: Combines the worst of both worlds

**Real number**: IMU at 100 Hz means one reading every 10 ms. An old reading from 100 ms ago is useless for fast balance control.

**Key insight**: QoS matches your application's physical constraints—safety-critical systems need `RELIABLE`, real-time systems need small `depth`.

</details>

---

### Question 4: Service vs. Action
**Your robot needs to execute a gripper command: close the gripper around an object and report progress (0%, 25%, 50%, 75%, 100% closed). Which should you choose?**

a) Topic (publish command, subscribe to progress)
b) Service (request close, wait for response)
c) Action (request close, receive progress feedback, wait for final result)
d) Parameter (set as config and let controller read it)

<details>
<summary>Show Answer</summary>

**Correct Answer: C**

Actions are perfect for this:
1. **Request**: "Close gripper"
2. **Feedback stream**: Progress updates (0%, 25%, 50%, ...)
3. **Cancellation**: Can send "stop" if something goes wrong
4. **Final result**: ```{"success": true, "final_position": 45 degrees}```
`
**Why others are wrong:**
- A: Topics don't have response semantics; also no cancellation
- B: Services give a single response, not streaming progress
- D: Parameters are for static config, not dynamic control

**Key insight**: Actions bridge topics (streaming feedback) and services (request-response). They're essential for robot tasks that take time.

**In code**:
```python
# Action client
self.action_client = self.create_client(GripperClose, 'gripper_close')
goal = GripperClose.Goal()
self.action_client.send_goal_async(goal, feedback_callback=self.feedback_cb)
```

</details>

---

### Question 5: Type Safety and Debugging (Challenge)
**A message was published as `geometry_msgs/Pose` (position + orientation) but subscribed as `geometry_msgs/Point` (just position). When running, you get no data. Why?**

a) The network connection is broken; restart the node
b) ROS 2 type checking prevented the subscription from connecting
c) The message is corrupted; check the network
d) You need to manually convert the message type in code

<details>
<summary>Show Answer</summary>

**Correct Answer: B**

This is ROS 2's **type safety in action**:
- Publisher declares: "I send `geometry_msgs/Pose`"
- Subscriber declares: "I receive `geometry_msgs/Point`"
- ROS 2 DDS layer checks: These types don't match → subscription fails silently
- **Result**: No data flows, no error message (this is a silent failure)

**Why others are wrong:**
- A: Network is fine; it's a type mismatch
- C: Not corruption; it's intentional type-safe design
- D: Conversion would happen after the subscription exists (which it doesn't)

**How to debug**:
```bash
ros2 topic info /pose_topic  # Shows the actual message type
# Output: Type: geometry_msgs/msg/Pose

ros2 node info /my_node      # Shows what topics this node expects
# Should show geometry_msgs/Pose, not geometry_msgs/Point
```

**Key insight**: Type safety prevents bugs (good!), but can be confusing when you get silent failures. Always check topic types with `ros2 topic info`.

**Challenge**: How would you fix this?
- Option 1: Change subscriber to expect `geometry_msgs/Pose`
- Option 2: Create a converter node that subscribes to `Pose`, extracts `position`, and publishes as `Point`

</details>

---

## What's Next?

Now that you understand *how* ROS 2 works architecturally, you're ready to **build your first nodes**.

**Chapter 1.2: Nodes and Topics - Pub/Sub Communication**
- Write a Python publisher (sensor simulator)
- Write a Python subscriber (data logger)
- Debug with `ros2 topic` CLI tools
- Deploy your first multi-node system

**Ready?** Turn the page to begin coding.

---

**Chapter Completed**: December 2025
**Estimated Reading Time**: 45-60 minutes
**Estimated Comprehension Time (with quiz)**: 60-75 minutes
**Prerequisites Checked**: Understanding of Module 0, basic Python knowledge
**Next Chapter**: 1.2 (Nodes and Topics)
**ROS 2 Version**: Humble (LTS, through 2027)
