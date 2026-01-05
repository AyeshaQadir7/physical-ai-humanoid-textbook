---
id: module-1-python-rclpy
title: "Python with rclpy"
sidebar_position: 5
sidebar_label: "Python with rclpy"
description: "Writing production-ready ROS 2 nodes in Python using rclpy library"
keywords: [ROS 2, Python, rclpy, node lifecycle, callbacks, logging, parameters, best practices]
---

# Python with rclpy

## Introduction

**rclpy** is the ROS 2 Python client library. It's what you use to write ROS 2 nodes in Python.

This chapter teaches you:
- Node **lifecycle** (creation, initialization, shutdown)
- **Callback patterns** (event-driven programming)
- **Logging** for debugging
- **Parameters** for configurable nodes
- **Best practices** for clean, maintainable code

By the end, you'll write professional ROS 2 nodes that other developers will be happy to use.

---

## Learning Outcomes

By the end of this chapter, you will:
1. Understand node lifecycle and when code runs
2. Write nodes with proper initialization and cleanup
3. Use logging effectively for debugging
4. Implement configurable parameters
5. Handle errors gracefully
6. Write testable, maintainable ROS 2 code

---

## Part 1: Node Lifecycle

### The Node Lifecycle

Every ROS 2 node has a lifecycle:

```
1. __init__()           → Node created, setup initialization
2. rclpy.init()         → ROS 2 system initialized
3. rclpy.spin()         → Node running, processing callbacks
4. (Ctrl+C or error)    → Shutdown signal
5. destroy_node()       → Node cleanup
6. rclpy.shutdown()     → ROS 2 system shutdown
```

### Complete Node Template

```python
import rclpy
from rclpy.node import Node

class MyRobot(Node):
    def __init__(self):
        """Called once when node is created"""
        super().__init__('my_robot_node')

        # Setup publishers, subscribers, services, etc.
        self.publisher = self.create_publisher(String, '/topic', 10)

        self.get_logger().info('MyRobot node initialized')

    def cleanup(self):
        """Optional: called during shutdown"""
        self.get_logger().info('Cleaning up resources')


def main(args=None):
    """Main entry point"""
    # 1. Initialize ROS 2
    rclpy.init(args=args)

    # 2. Create node
    node = MyRobot()

    try:
        # 3. Run node (blocking, until Ctrl+C)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 4. Cleanup
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Part 2: Logging

### Effective Logging

Logging is essential for debugging. ROS 2 provides a logger for each node:

```python
import rclpy
from rclpy.node import Node

class LoggingExample(Node):
    def __init__(self):
        super().__init__('logging_example')

        # Access logger via self.get_logger()
        self.get_logger().debug('This is a DEBUG message (not shown by default)')
        self.get_logger().info('This is an INFO message')
        self.get_logger().warn('This is a WARNING message')
        self.get_logger().error('This is an ERROR message')
        self.get_logger().fatal('This is a FATAL message')
```

### Log Levels (Verbosity)

| Level | Use Case | Example |
|-------|----------|---------|
| **DEBUG** | Detailed info for developers | Variable values, loop iterations |
| **INFO** | General informational | Node started, service called |
| **WARN** | Something unexpected | Battery low, sensor drift |
| **ERROR** | Error, but continues | Service failed, bad input |
| **FATAL** | Critical, program stops | Out of memory, hardware failure |

### Viewing Logs

```bash
# Run node (INFO level and above shown by default)
ros2 run package_name node_name

# View with timestamps
ros2 run package_name node_name --ros-args --log-level info

# View DEBUG messages
ROS_LOG_DIR=/tmp ros2 run package_name node_name --ros-args --log-level debug
```

### Log Formatting

```python
# Basic logging
self.get_logger().info('Battery level: 85%')

# Formatted logging (f-strings recommended)
battery = 85
self.get_logger().info(f'Battery level: {battery}%')

# Log exceptions
try:
    result = 10 / 0
except ZeroDivisionError as e:
    self.get_logger().error(f'Math error: {e}')
```

---

## Part 3: Node Parameters

### Declaring Parameters

Parameters are values you can change without recompiling. Example: sensor calibration values, motor speeds, timeouts.

```python
import rclpy
from rclpy.node import Node

class ConfigurableRobot(Node):
    def __init__(self):
        super().__init__('configurable_robot')

        # Declare parameters with defaults
        self.declare_parameter('motor_speed', 0.5)  # Default: 0.5
        self.declare_parameter('wheel_diameter', 0.1)  # meters
        self.declare_parameter('debug_mode', False)  # boolean

        # Get parameter value
        motor_speed = self.get_parameter('motor_speed').value
        wheel_diameter = self.get_parameter('wheel_diameter').value

        self.get_logger().info(f'Motor speed: {motor_speed}')
        self.get_logger().info(f'Wheel diameter: {wheel_diameter}')
```

### Setting Parameters at Runtime

**Method 1: Launch file** (covered in next chapter)

**Method 2: Command line**
```bash
ros2 run package_name node_name --ros-args -p motor_speed:=1.0 -p debug_mode:=true
```

**Method 3: Parameter server (set before launch)**
```bash
# In terminal 1
ros2 param set /my_robot_node motor_speed 1.0
```

### Listen for Parameter Changes

```python
class AdaptiveRobot(Node):
    def __init__(self):
        super().__init__('adaptive_robot')

        self.declare_parameter('speed', 0.5)

        # Subscribe to parameter changes
        from rcl_interfaces.msg import ParameterEvent
        self.param_subscription = self.create_subscription(
            ParameterEvent,
            '/parameter_events',
            self.param_callback,
            10
        )

    def param_callback(self, msg):
        """Called when any parameter changes"""
        for changed_param in msg.changed_parameters:
            if changed_param.name == 'speed':
                new_speed = changed_param.value.double_value
                self.get_logger().info(f'Speed changed to {new_speed}')
                self.adjust_speed(new_speed)

    def adjust_speed(self, new_speed):
        pass
```

---

## Part 4: Callback Patterns

### Callback Types

ROS 2 uses **callbacks** for asynchronous event handling:

| Callback | Triggered When | Example |
|----------|---|---|
| **Timer callback** | Timer fires | `def timer_callback(self):` |
| **Subscription callback** | Message arrives | `def msg_callback(self, msg):` |
| **Service callback** | Request arrives | `def service_callback(self, request, response):` |
| **Action callback** | Goal received | `def action_callback(self, goal_handle):` |

### Example: Combining Callbacks

```python
class MultiCallbackNode(Node):
    def __init__(self):
        super().__init__('multi_callback')

        # Timer callback (every 1 second)
        self.timer = self.create_timer(1.0, self.timer_callback)

        # Subscription callback (when message arrives)
        self.sub = self.create_subscription(
            String,
            '/sensor_data',
            self.sensor_callback,
            10
        )

        # Service callback
        self.service = self.create_service(
            GetInt,
            '/get_sensor_count',
            self.service_callback
        )

        self.sensor_count = 0

    def timer_callback(self):
        """Called every 1 second"""
        self.get_logger().info(f'Timer tick (total sensors: {self.sensor_count})')

    def sensor_callback(self, msg):
        """Called when sensor message arrives"""
        self.sensor_count += 1
        self.get_logger().info(f'Sensor data received: {msg.data}')

    def service_callback(self, request, response):
        """Called when service is requested"""
        response.data = self.sensor_count
        return response
```

**Key insight**: All callbacks are event-driven. ROS 2 calls them when events occur. Your code should **not block** in callbacks.

---

## Part 5: Error Handling

### Graceful Error Handling

```python
class RobustRobot(Node):
    def __init__(self):
        super().__init__('robust_robot')
        self.publisher = self.create_publisher(String, '/command', 10)

    def send_command(self, command):
        """Send command, handle errors"""
        try:
            # Validate input
            if not isinstance(command, str) or not command:
                raise ValueError('Command must be non-empty string')

            # Send message
            msg = String()
            msg.data = command
            self.publisher.publish(msg)

            self.get_logger().info(f'Command sent: {command}')

        except ValueError as e:
            self.get_logger().error(f'Invalid command: {e}')
        except Exception as e:
            self.get_logger().fatal(f'Unexpected error: {e}')
```

### Handling Timeouts

```python
class ClientWithTimeout(Node):
    def __init__(self):
        super().__init__('timeout_client')
        self.client = self.create_client(GetInt, '/my_service')

    def call_service(self):
        """Call service with timeout"""
        # Wait for service with 5-second timeout
        if not self.client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Service not available (timeout)')
            return None

        request = GetInt.Request()
        future = self.client.call_async(request)

        # Wait for response with timeout
        try:
            response = future.result(timeout_sec=5.0)
            return response
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')
            return None
```

---

## Part 6: Context Managers (With Statement)

### Resource Management

Python's `with` statement ensures cleanup:

```python
import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node('my_node')

    # Automatically calls destroy_node() when exiting with block
    with node:
        rclpy.spin_once(node, timeout_sec=1.0)
        node.get_logger().info('Node ran once')

    # Node is automatically destroyed here
    rclpy.shutdown()
```

---

## Part 7: Best Practices

### Code Organization

```python
# my_robot.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyRobot(Node):
    """Robot controller node"""

    def __init__(self):
        super().__init__('my_robot')
        self._setup_parameters()
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_timers()

    def _setup_parameters(self):
        """Declare all parameters"""
        self.declare_parameter('speed', 0.5)
        self.speed = self.get_parameter('speed').value

    def _setup_publishers(self):
        """Create publishers"""
        self.cmd_publisher = self.create_publisher(String, '/cmd', 10)

    def _setup_subscribers(self):
        """Create subscribers"""
        self.sensor_sub = self.create_subscription(
            String,
            '/sensor',
            self.sensor_callback,
            10
        )

    def _setup_timers(self):
        """Create timers"""
        self.control_timer = self.create_timer(0.1, self.control_loop)

    def sensor_callback(self, msg):
        self.get_logger().info(f'Sensor: {msg.data}')

    def control_loop(self):
        # Control logic here
        pass


def main(args=None):
    rclpy.init(args=args)
    robot = MyRobot()
    try:
        rclpy.spin(robot)
    except KeyboardInterrupt:
        pass
    finally:
        robot.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### Naming Conventions

✅ Do:
- `self.sensor_publisher` - clear, descriptive names
- `def sensor_callback(self, msg):` - verb_noun for callbacks
- `CONSTANT = 3.14` - uppercase for constants

❌ Don't:
- `self.pub` - too vague
- `def callback(self, msg):` - which callback?
- `my_value = 3.14` - lowercase for constants

### Documentation

```python
class WellDocumentedRobot(Node):
    """Main robot controller node.

    Subscribes to sensor topics and publishes motor commands.
    Responds to service requests for status checks.
    """

    def __init__(self):
        """Initialize robot node with publishers and subscribers."""
        super().__init__('well_documented_robot')

    def control_loop(self):
        """Execute main control loop.

        Reads sensor data, computes motor commands, publishes results.
        Called every 100ms by a timer.
        """
        pass
```

---

## Summary

**Node Lifecycle**:
1. `__init__()` - setup
2. `rclpy.init()` - system init
3. `rclpy.spin()` - run (event-driven callbacks)
4. `destroy_node()` + `shutdown()` - cleanup

**Logging**: Use appropriate levels (debug, info, warn, error, fatal) for debugging.

**Parameters**: Make nodes configurable without recompiling.

**Callbacks**: Event-driven; keep them non-blocking.

**Error Handling**: Expect failures; handle gracefully.

---

## Next Steps

Next chapter: **Launch Files & Parameters** - running multi-node systems and coordinating them.

**Next Chapter**: [Chapter 6: Launch Files & Parameters](./06-launch-files-and-parameters.md)

---

## Navigation

- **Previous**: [Chapter 4: Actions & Timers](./04-actions-and-timers.md)
- **Next**: [Chapter 6: Launch Files & Parameters](./06-launch-files-and-parameters.md)
