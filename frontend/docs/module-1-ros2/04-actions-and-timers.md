---
id: module-1-actions-and-timers
title: "Actions & Timers"
sidebar_position: 4
sidebar_label: "Actions & Timers"
description: "Long-running tasks with Actions and periodic execution with Timers in ROS 2"
keywords: [ROS 2, actions, timers, callbacks, long-running tasks, periodic execution, feedback]
---

# Actions & Timers

## Introduction

You've learned **topics** (streaming data) and **services** (request-reply). Now meet the third communication pattern: **actions**.

**Actions** are like services, but for **long-running tasks**:
- You request an action and get immediate feedback (progress updates)
- The server sends status as it works
- The client can cancel if needed

**Timers** let you run code **periodically** (every 100ms, every 1 second, etc.).

By the end of this chapter, you'll understand:
- When to use actions vs services
- How to implement action servers and clients
- How to use timers for periodic tasks
- Real-world robot examples

**Why this matters for your capstone**: Robot movements take time. If you tell a robot "walk 5 meters," it won't happen instantly. You need a way to command it and get progress updates. That's what actions do.

---

## Learning Outcomes

By the end of this chapter, you will:
1. Explain the difference between services and actions
2. Write an **action server** that executes long-running tasks
3. Write an **action client** that sends goals and receives feedback
4. Use **timers** for periodic execution
5. Cancel actions and handle timeouts

---

## Part 1: Service vs Action

### When Use Service?
- ✅ Fast operations (< 1 second)
- ✅ Simple request-reply
- Examples: "get status", "check collision", "load file"

### When to Use Action?
- ✅ Long-running operations
- ✅ Need progress updates
- ✅ Client might want to cancel
- Examples: "walk 5 meters", "move arm to position", "take a photo"

### Comparison

| Aspect | Service | Action |
|--------|---------|--------|
| Blocking | Yes (synchronous) | No (asynchronous) |
| Duration | Quick (Under 1s) | Long (seconds to minutes) |
| Feedback | None | Real-time progress |
| Cancellation | Can't cancel | Can cancel mid-action |
| Use case | Queries, config | Movement, navigation, tasks |

**Example**:
```
Service: "Is the door open?" → "Yes" (instant)
Action: "Open the door" → "Starting..." → "60% open..." → "Done!"
```

---

## Part 2: Action Structure

An action has three messages:

```
ACTION REQUEST (Goal)
├─ What to do? (e.g., "move arm to position X")

ACTION FEEDBACK (periodic updates)
├─ Current progress (e.g., "arm is 50% of the way to target")

ACTION RESULT (final response)
└─ Did it succeed? (e.g., "arm reached position, final position: X")
```

---

## Part 3: Timer Example

### Example 1: Periodic Sensor Polling

Before diving into actions, let's cover **timers**, which you'll use in actions and everywhere else:

```python
import rclpy
from rclpy.node import Node

class SensorPoller(Node):
    def __init__(self):
        super().__init__('sensor_poller')

        # Create a timer that calls timer_callback every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.count = 0

    def timer_callback(self):
        """Called every 0.5 seconds"""
        self.count += 1
        self.get_logger().info(f'Polling sensors (call #{self.count})')

        # Read actual sensor data here
        # Example: battery_level = read_battery()
        # Example: self.publish_battery(battery_level)


def main(args=None):
    rclpy.init(args=args)
    poller = SensorPoller()
    rclpy.spin(poller)
```

**Key points**:
- `self.create_timer(interval, callback)` - calls callback every `interval` seconds
- Callback is non-blocking; ROS 2 will call it on schedule
- Use for periodic polling, control loops, heartbeats

---

## Part 4: Action Server Example

### Example 2: Arm Movement Action Server

Here's a robot arm that responds to "move arm to position" requests:

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from action_tutorials_interfaces.action import Fibonacci

class ArmMoveServer(Node):
    def __init__(self):
        super().__init__('arm_move_server')

        # Create action server
        # (normally use custom action type; here using example)
        self.action_server = ActionServer(
            self,
            Fibonacci,
            'move_arm',  # Action name
            self.execute_callback,
            callback_group=ReentrantCallbackGroup()
        )

        self.get_logger().info('Arm move action server started')

    def execute_callback(self, goal_handle):
        """
        Called when client sends a goal
        goal_handle.request = the goal
        goal_handle.feedback = send updates
        goal_handle.result = final result
        """
        self.get_logger().info(f'Executing goal: {goal_handle.request}')

        # Simulate arm movement over 5 seconds
        feedback_msg = Fibonacci.Feedback()

        for i in range(5):
            if goal_handle.is_cancel_requested():
                self.get_logger().info('Goal canceled!')
                goal_handle.canceled()
                return Fibonacci.Result()

            # Move arm 20% per second
            feedback_msg.sequence = [i * 20]  # Progress: 0, 20, 40, 60, 80%
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Arm at {i * 20}% of target position')

            # Sleep for 1 second (yielding to event loop)
            import time
            time.sleep(1)

        # Action succeeded
        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = [100]  # Final position: 100% (reached target)

        self.get_logger().info('Arm movement completed!')
        return result


def main(args=None):
    rclpy.init(args=args)
    server = ArmMoveServer()
    rclpy.spin(server)
```

**Key concepts**:
- `ActionServer()` - register action
- `goal_handle.is_cancel_requested()` - check if client canceled
- `goal_handle.publish_feedback()` - send progress updates
- `goal_handle.succeed()` / `goal_handle.abort()` - finish action
- Return `result`

---

## Part 5: Action Client Example

### Example 3: Action Client (Request Arm Movement)

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from action_tutorials_interfaces.action import Fibonacci

class ArmMoveClient(Node):
    def __init__(self):
        super().__init__('arm_move_client')

        # Create action client
        self.action_client = ActionClient(self, Fibonacci, 'move_arm')

        self.get_logger().info('Arm move client started')

        # Wait for action server to be available
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        # Send goal
        self.send_goal()

    def send_goal(self):
        """Send a goal to the action server"""
        goal = Fibonacci.Goal()
        goal.order = 10  # Example parameter

        self.get_logger().info('Sending goal...')

        # Send goal (non-blocking)
        future = self.action_client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )

        # When server accepts goal, call goal_response_callback
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when server accepts or rejects the goal"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return

        self.get_logger().info('Goal accepted!')

        # Request the result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        """Called when server sends feedback"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback.sequence}')

    def result_callback(self, future):
        """Called when action completes"""
        result = future.result().result
        status = future.result().status

        if status == 4:  # Goal succeeded
            self.get_logger().info(f'Action succeeded! Result: {result.sequence}')
        else:
            self.get_logger().error(f'Action failed with status {status}')


def main(args=None):
    rclpy.init(args=args)
    client = ArmMoveClient()
    rclpy.spin(client)
```

**Key concepts**:
- `ActionClient()` - create client
- `send_goal_async()` - send goal (non-blocking)
- `feedback_callback` - receive progress updates
- `result_callback` - final result

---

## Part 6: Running Action Example

### Terminal 1: Start Action Server

```bash
ros2 run your_package arm_move_server.py
```

**Output**:
```
[INFO] Arm move action server started
[INFO] Executing goal: ...
[INFO] Arm at 0% of target position
[INFO] Arm at 20% of target position
[INFO] Arm at 40% of target position
[INFO] Arm at 60% of target position
[INFO] Arm at 80% of target position
[INFO] Arm movement completed!
```

### Terminal 2: Start Action Client

```bash
ros2 run your_package arm_move_client.py
```

**Output**:
```
[INFO] Arm move client started
[INFO] Sending goal...
[INFO] Goal accepted!
[INFO] Received feedback: [0]
[INFO] Received feedback: [20]
[INFO] Received feedback: [40]
[INFO] Received feedback: [60]
[INFO] Received feedback: [80]
[INFO] Action succeeded! Result: [100]
```

### Inspect with ROS 2 CLI

```bash
# List actions
ros2 action list
# Output: /move_arm

# Show action info
ros2 action info /move_arm
# Output:
# Action: /move_arm
# Action servers: 1
#   /arm_move_server
# Action clients: 1
#   /arm_move_client

# Send goal manually (advanced)
ros2 action send_goal /move_arm action_tutorials_interfaces/action/Fibonacci "{order: 5}"
```

---

## Part 7: Real-World Robot Example

### Example 4: Mobile Robot Navigation Action

Imagine a mobile robot that navigates to a goal position:

```python
# navigate_to_goal_server.py
class NavigateToGoalServer(Node):
    def __init__(self):
        super().__init__('navigate_to_goal_server')
        self.action_server = ActionServer(
            self,
            NavigateToGoal,
            'navigate_to_goal',
            self.execute_callback
        )

    def execute_callback(self, goal_handle):
        """Navigate robot to goal position"""
        goal = goal_handle.request
        self.get_logger().info(f'Navigating to {goal.target_x}, {goal.target_y}')

        feedback_msg = NavigateToGoal.Feedback()
        current_x, current_y = 0, 0

        # Simulate robot moving towards goal
        while True:
            # Check for cancellation
            if goal_handle.is_cancel_requested():
                goal_handle.canceled()
                return NavigateToGoal.Result()

            # Calculate distance to goal
            dx = goal.target_x - current_x
            dy = goal.target_y - current_y
            distance = (dx**2 + dy**2) ** 0.5

            if distance < 0.1:  # Reached goal
                goal_handle.succeed()
                return NavigateToGoal.Result(success=True)

            # Move towards goal
            current_x += (dx / distance) * 0.1
            current_y += (dy / distance) * 0.1

            # Send feedback
            feedback_msg.current_x = current_x
            feedback_msg.current_y = current_y
            feedback_msg.distance_remaining = distance
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info(f'Distance to goal: {distance:.2f}m')

            import time
            time.sleep(0.1)
```

---

## Part 8: Best Practices

### Timer Usage
- ✅ Use for periodic polling, control loops
- ✅ Keep callbacks short and fast
- ❌ Don't do heavy computation in timers; use threads instead
- ❌ Don't call blocking functions (like `input()`)

### Action Usage
- ✅ Use for long-running tasks
- ✅ Send meaningful feedback
- ✅ Check for cancellation
- ✅ Handle timeout on client side
- ❌ Don't use actions for quick operations (use services instead)

### Performance
- **Feedback frequency**: Send updates regularly (not too frequent)
- **Timeout**: Set reasonable timeouts on clients
- **Cancellation**: Always check `is_cancel_requested()`

---

## Summary

**Timers**:
- Execute callbacks periodically
- Good for control loops, polling
- Keep callbacks non-blocking

**Actions**:
- Long-running tasks with feedback
- Client sends goal, server responds with progress
- Supports cancellation
- Three-part message: goal, feedback, result

**Robot Design**:
- Sensors: publish to topics
- Quick commands: use services
- Movement/long tasks: use actions
- Control loops: use timers

---

## Glossary Links

- **ROS 2 Actions**: [ROS 2 Docs](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Actions.html)
- **ROS 2 Timers**: [ROS 2 Docs](https://docs.ros.org/en/humble/Concepts/Intermediate/About-Timers.html)

---

## Next Steps

Next chapter covers **Python with rclpy** - how to write clean, production-ready ROS 2 code.

**Next Chapter**: [Chapter 5: Python with rclpy](./05-python-with-rclpy.md)

---

## Navigation

- **Previous**: [Chapter 3: Nodes, Topics & Services](./03-nodes-topics-services.md)
- **Next**: [Chapter 5: Python with rclpy](./05-python-with-rclpy.md)
- **Lab**: [Lab 1.3: Create a ROS 2 Package](./lab-1-3-create-a-ros2-package.md)
