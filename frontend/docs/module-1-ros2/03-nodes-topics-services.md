---
id: module-1-nodes-topics-services
title: "Nodes, Topics & Services"
sidebar_position: 3
sidebar_label: "Nodes, Topics & Services"
description: "Deep dive into ROS 2 pub/sub and request/response patterns, core mechanisms for robot communication"
keywords: [ROS 2, nodes, topics, publishers, subscribers, services, pub/sub, request/response, communication]
---

# Nodes, Topics & Services

## Introduction

In the previous chapter, you learned that **ROS 2 is the "nervous system" of robots**. This chapter shows you how information flows through that nervous system via two fundamental patterns:

1. **Pub/Sub (Publish-Subscribe)**: One-way streaming of data (sensor data, commands)
2. **Request/Response (Services)**: Two-way synchronous calls (requesting information, executing actions)

By the end of this chapter, you'll understand:
- What nodes are and how they communicate
- When to use **topics** (streaming data)
- When to use **services** (request-reply interactions)
- Real-world examples from robotics

**Why this matters for your capstone**: Every sensor publishes to a topic. Every robot command is either published to a topic or called as a service. Master this chapter, and you master robot communication.

---

## Learning Outcomes

By the end of this chapter, you will:
1. Explain the difference between pub/sub (topics) and request/response (services)
2. Write a **publisher node** that sends sensor data to a topic
3. Write a **subscriber node** that receives and processes data
4. Write a **service server** that responds to requests
5. Write a **service client** that makes service calls
6. Design robot communication architectures using topics and services

---

## Part 1: The Pub/Sub Pattern (Topics)

### What is Pub/Sub?

**Publish-Subscribe** is a pattern where:
- **Publishers** send data to a **topic** (like shouting in a room)
- **Subscribers** listen to that topic (like people in the room hearing the shout)
- **Multiple publishers** can write to the same topic
- **Multiple subscribers** can read from the same topic
- **No acknowledgment**: Publishers don't know if anyone is listening

### Topic Basics

```
Publisher Node                    Topic                    Subscriber Nodes
      ↓                            ↓                              ↓
  Sensor Data          /robot/sensor/lidar              LiDAR Processor
  (LiDAR)              ───────────────────→             Path Planner
                                                        Obstacle Detector
```

### Topic Naming Conventions

- All lowercase
- Forward slashes separate hierarchy: `/robot/sensor/lidar`, `/cmd/velocity`
- **Global**: `/camera/image` (accessible from anywhere)
- **Relative**: `sensor/lidar` (scoped to current namespace)
- **Private**: `~/sensor/lidar` (private to node)

### Message Types

Topics carry **typed messages**. Common message types:

| Message Type | Use Case | Example |
|--------------|----------|---------|
| `std_msgs/String` | Text data | "Hello ROS 2" |
| `std_msgs/Float32` | Single number | Temperature reading |
| `std_msgs/Int32` | Integer | Counter, ID |
| `sensor_msgs/Image` | Camera image | RGB camera data |
| `sensor_msgs/LaserScan` | LiDAR data | 360° laser scan |
| `geometry_msgs/Twist` | Velocity command | Linear + angular velocity |
| `nav_msgs/Odometry` | Robot position | Position + orientation |
| Custom message | Your data | Your type definition |

---

## Part 2: Publisher Node Example

### Example 1: Temperature Sensor Publisher

Here's a real node that reads temperature and publishes it:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import random  # Simulated sensor reading

class TemperaturePublisher(Node):
    def __init__(self):
        super().__init__('temperature_publisher')

        # Create a publisher for Float32 messages on /robot/temperature topic
        self.publisher = self.create_publisher(Float32, '/robot/temperature', 10)

        # Create a timer to publish every 1 second
        self.timer_period = 1.0  # seconds
        self.timer = self.create_timer(self.timer_period, self.publish_temperature)

        self.get_logger().info('Temperature publisher started')

    def publish_temperature(self):
        """Called every 1 second to publish temperature"""
        # In real application, read from actual sensor
        # Here we simulate: range 20-25°C
        temperature = random.uniform(20.0, 25.0)

        msg = Float32()
        msg.data = temperature

        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing temperature: {temperature:.1f}°C')


def main(args=None):
    rclpy.init(args=args)
    publisher = TemperaturePublisher()
    rclpy.spin(publisher)  # Keep running until Ctrl+C
    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key Concepts**:
- `self.create_publisher()`: Register as publisher
- `self.publisher.publish(msg)`: Send message
- `self.create_timer()`: Call function periodically
- `rclpy.spin()`: Keep node running

---

## Part 3: Subscriber Node Example

### Example 2: Temperature Monitor (Subscriber)

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32

class TemperatureMonitor(Node):
    def __init__(self):
        super().__init__('temperature_monitor')

        # Create a subscriber to /robot/temperature topic
        # When a message arrives, call self.temperature_callback
        self.subscription = self.create_subscription(
            Float32,
            '/robot/temperature',
            self.temperature_callback,
            10  # Queue size
        )

        self.get_logger().info('Temperature monitor started, listening to /robot/temperature')

    def temperature_callback(self, msg):
        """Called when a message arrives on the topic"""
        temperature = msg.data

        # Check if temperature is in safe range
        if temperature < 18:
            self.get_logger().warn(f'Temperature too LOW: {temperature:.1f}°C')
        elif temperature > 25:
            self.get_logger().warn(f'Temperature too HIGH: {temperature:.1f}°C')
        else:
            self.get_logger().info(f'Temperature OK: {temperature:.1f}°C')


def main(args=None):
    rclpy.init(args=args)
    monitor = TemperatureMonitor()
    rclpy.spin(monitor)
    monitor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

**Key Concepts**:
- `self.create_subscription()`: Listen to a topic
- `callback`: Function called when message arrives
- Callbacks are **asynchronous** (no blocking)

---

## Part 4: Running Pub/Sub Nodes

### In Terminal 1: Start Publisher

```bash
cd ros2_workspace
source install/setup.bash
ros2 run your_package temperature_publisher.py
```

**Output**:
```
[INFO] Temperature publisher started
[INFO] Publishing temperature: 22.3°C
[INFO] Publishing temperature: 23.1°C
[INFO] Publishing temperature: 21.8°C
...
```

### In Terminal 2: Start Subscriber

```bash
ros2 run your_package temperature_monitor.py
```

**Output**:
```
[INFO] Temperature monitor started, listening to /robot/temperature
[INFO] Temperature OK: 22.3°C
[INFO] Temperature OK: 23.1°C
[INFO] Temperature OK: 21.8°C
...
```

### Inspect with ROS 2 CLI

```bash
# List all active topics
ros2 topic list
# Output: /robot/temperature

# Show message type
ros2 topic info /robot/temperature
# Output:
# Type: std_msgs/Float32
# Publisher count: 1
# Subscription count: 1

# Echo messages in real-time
ros2 topic echo /robot/temperature
# Output:
# data: 22.3
# ---
# data: 23.1
# ---

# Show message rate and bandwidth
ros2 topic hz /robot/temperature
# Output: average rate: 1.000 Hz

# Publish manually (for testing)
ros2 topic pub /robot/temperature std_msgs/Float32 'data: 25.5'
```

---

## Part 5: The Request/Response Pattern (Services)

### What is Request/Response?

**Services** are a synchronous pattern where:
- **Client** sends a request and **waits** for a response
- **Server** receives request, processes it, sends response back
- **Blocking**: Client waits for server to finish
- Use for: queries, configuration changes, one-off actions

### Service vs Topic: When to Use Each?

| Scenario | Use Topics | Use Services |
|----------|------------|--------------|
| Continuous sensor stream | ✅ | ❌ |
| Robot command (e.g., "move arm") | ❌ | ✅ |
| Camera feed | ✅ | ❌ |
| Request robot status | ❌ | ✅ |
| Emergency stop | Depends* | Usually ✅ |

*Emergency stop is often a topic for speed and simplicity

### Service Interface Definition

First, define the **service message** (request + response):

**File**: `srv/GetTemperatureSafety.srv`

```
#Request
float32 threshold
---
#Response
bool is_safe
string status
```

### Example 6: Temperature Service Server

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger  # Built-in service: no params, bool response

class RobotHealthServer(Node):
    def __init__(self):
        super().__init__('robot_health_server')

        # Create a service server
        self.service = self.create_service(
            Trigger,
            '/robot/check_health',
            self.check_health_callback
        )

        self.get_logger().info('Robot health service server started')

    def check_health_callback(self, request, response):
        """
        Handle service request
        request: (empty for Trigger service)
        response: success (bool), message (str)
        """
        # Simulate health check
        all_systems_ok = True

        health_checks = {
            'Battery': 85,
            'CPU': 45,
            'Memory': 60,
            'Motors': 'OK'
        }

        # Build response
        response.success = all_systems_ok
        response.message = f'Health check complete: {health_checks}'

        self.get_logger().info(f'Health check requested, responding: {response.success}')
        return response


def main(args=None):
    rclpy.init(args=args)
    server = RobotHealthServer()
    rclpy.spin(server)
    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Part 6: Service Client Example

### Example 7: Service Client (Request Health Check)

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

class RobotHealthChecker(Node):
    def __init__(self):
        super().__init__('robot_health_checker')

        # Create a service client
        self.client = self.create_client(Trigger, '/robot/check_health')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Connected to health check service')

        # Send request
        self.send_request()

    def send_request(self):
        """Send a health check request"""
        request = Trigger.Request()

        # Call service (blocking)
        future = self.client.call_async(request)

        # Handle response
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """Called when server responds"""
        try:
            response = future.result()
            self.get_logger().info(f'Health check: {response.success}')
            self.get_logger().info(f'Status: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    checker = RobotHealthChecker()
    rclpy.spin(checker)
    checker.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Part 7: Real-World Robot Example

### Example 8: Complete Robot Sensor/Command System

Imagine a mobile robot with:
- LiDAR sensor → publishes to `/scan` topic
- Motor controller → subscribes to `/cmd_vel` topic
- Safety check → service `/check_collision`

```python
# lidar_publisher.py
class LiDARPublisher(Node):
    def __init__(self):
        super().__init__('lidar_publisher')
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.publish_scan)

    def publish_scan(self):
        # Simulate LiDAR scan (360 degrees, 1 meter range)
        msg = LaserScan()
        msg.ranges = [1.0] * 360
        self.pub.publish(msg)


# motor_controller.py
class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        self.sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

    def cmd_vel_callback(self, msg):
        # msg.linear.x = forward speed
        # msg.angular.z = rotation speed
        self.drive_motor(msg.linear.x, msg.angular.z)

    def drive_motor(self, linear, angular):
        print(f'Driving: linear={linear}, angular={angular}')


# collision_checker.py
class CollisionChecker(Node):
    def __init__(self):
        super().__init__('collision_checker')
        self.service = self.create_service(GetInt, '/check_collision', self.check_callback)

    def check_callback(self, request, response):
        # Check if collision in requested direction
        # 0=forward, 1=left, 2=right
        response.data = 0  # No collision
        return response
```

**Communication Flow**:
```
LiDAR Sensor (publishes)
     ↓
/scan topic
     ↓ (subscribed)
Motor Controller (consumes scan data)
     ↓ (publishes)
/cmd_vel topic
     ↓ (subscribed)
Motor Driver (executes movement)

Collision Service:
Client requests → Server checks LiDAR → Responds "safe" or "blocked"
```

---

## Part 8: Best Practices

### Topic Design
- ✅ Use descriptive names: `/robot/sensor/lidar` not `/data`
- ✅ Organize hierarchically: namespace/subsystem/sensor
- ✅ Use standard message types when possible
- ❌ Don't create massive monolithic messages; break into separate topics

### Service Design
- ✅ Use services for queries (get_status, check_collision)
- ✅ Use services for configuration (set_speed, load_map)
- ❌ Don't use services for high-frequency data (use topics instead)
- ❌ Don't block in callbacks; use async patterns

### Performance
- **Queue size**: Larger = more memory but less dropped messages (default 10)
- **Topic frequency**: Publish at appropriate rate (e.g., 10 Hz for sensors, 100 Hz for control)
- **Service latency**: Keep short; heavy computation in separate thread

---

## Summary

**Pub/Sub (Topics)**:
- One-way streaming of data
- Publishers don't know about subscribers
- Good for sensor data, continuous streams

**Request/Response (Services)**:
- Two-way synchronous communication
- Client blocks waiting for response
- Good for queries, configuration, one-off actions

**Robot Design**:
- Sensors publish to topics
- Controllers subscribe to sensor topics
- Commands often use services
- High-frequency loops use topics, not services

---

## Glossary Links

- **ROS 2 Topic**: [ROS 2 Docs](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Topics.html)
- **ROS 2 Service**: [ROS 2 Docs](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Services.html)
- **Message Types**: [Message Packages](https://docs.ros.org/en/humble/Concepts/Intermediate/About-ROS-Interfaces.html)

---

## Next Steps

Now that you understand nodes, topics, and services, the next chapter covers:
- **Actions** for long-running tasks (like "move arm to position")
- **Timers and Callbacks** for periodic execution
- How to combine all three patterns in real robots

**Next Chapter**: [Chapter 4: Actions & Timers](./04-actions-and-timers.md)

---

## Navigation

- **Previous**: [Chapter 2: ROS 2 Architecture](./02-architecture-overview.md)
- **Next**: [Chapter 4: Actions & Timers](./04-actions-and-timers.md)
- **Lab**: [Lab 1.2: Implement a Service](./lab-1-2-implement-a-service.md)
