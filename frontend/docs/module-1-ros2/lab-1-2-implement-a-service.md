---
id: module-1-lab-1-2
title: "Lab 1.2: Implement a Service"
sidebar_position: 8
sidebar_label: "Lab 1.2: Service"
description: "Build service server and client for request-reply communication"
keywords: [lab, ROS 2, service, server, client, request, reply]
---

# Lab 1.2: Implement a Service

## Lab Objective

Build service-based robot communication:
1. **Service Server**: Responds to robot status requests
2. **Service Client**: Requests robot status

**Success Criteria**:
- Server responds to requests ✅
- Client receives responses ✅
- Verify service call succeeds ✅

---

## Prerequisites

- Completed Lab 1.1
- ROS 2 Humble
- ~30 minutes

---

## Part 1: Use Built-in Service Type

For simplicity, we'll use ROS 2's built-in `Trigger` service:
- **Request**: (empty)
- **Response**: `success` (bool), `message` (str)

### 1.1 Create Service Server

**File**: `src/my_first_ros2_pkg/my_first_ros2_pkg/service_server.py`

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MinimalServiceServer(Node):
    def __init__(self):
        super().__init__('minimal_service_server')
        self.srv = self.create_service(Trigger, 'robot_status', self.handle_status_request)
        self.get_logger().info('Robot status service server ready')

    def handle_status_request(self, request, response):
        """Handle incoming status request"""
        self.get_logger().info('Status request received')

        # Simulate robot status check
        battery = 85
        cpu_usage = 45
        memory_usage = 60

        response.success = True
        response.message = f'Battery: {battery}%, CPU: {cpu_usage}%, Memory: {memory_usage}%'

        self.get_logger().info(f'Responding: {response.message}')
        return response


def main(args=None):
    rclpy.init(args=args)
    minimal_service_server = MinimalServiceServer()
    rclpy.spin(minimal_service_server)
    minimal_service_server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### 1.2 Create Service Client

**File**: `src/my_first_ros2_pkg/my_first_ros2_pkg/service_client.py`

```python
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class MinimalServiceClient(Node):
    def __init__(self):
        super().__init__('minimal_service_client')
        self.client = self.create_client(Trigger, 'robot_status')

        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        self.get_logger().info('Service available!')

        # Send request
        self.send_request()

    def send_request(self):
        """Send a status request"""
        request = Trigger.Request()

        # Call service asynchronously
        future = self.client.call_async(request)

        # Handle response when ready
        future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        """Called when service responds"""
        try:
            response = future.result()
            self.get_logger().info(f'Service response: {response.message}')
        except Exception as e:
            self.get_logger().error(f'Service call failed: {e}')


def main(args=None):
    rclpy.init(args=args)
    minimal_service_client = MinimalServiceClient()
    rclpy.spin(minimal_service_client)
    minimal_service_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

---

## Part 2: Register Executables

Edit `setup.py` entry_points:

```python
entry_points={
    'console_scripts': [
        'talker = my_first_ros2_pkg.talker:main',
        'listener = my_first_ros2_pkg.listener:main',
        'service_server = my_first_ros2_pkg.service_server:main',
        'service_client = my_first_ros2_pkg.service_client:main',
    ],
},
```

---

## Part 3: Build and Run

### 3.1 Rebuild Package

```bash
cd ~/ros2_ws
colcon build --packages-select my_first_ros2_pkg
source install/setup.bash
```

### 3.2 Terminal 1: Run Service Server

```bash
ros2 run my_first_ros2_pkg service_server
```

**Output**:
```
[INFO] Robot status service server ready
```

### 3.3 Terminal 2: Run Service Client

```bash
ros2 run my_first_ros2_pkg service_client
```

**Output**:
```
[INFO] Service available!
[INFO] Service response: Battery: 85%, CPU: 45%, Memory: 60%
```

### 3.4 Terminal 3: Inspect Service with CLI

```bash
# List services
ros2 service list
# Output:
# /robot_status
# ... (other services)

# Show service type
ros2 service type /robot_status
# Output: std_srvs/Trigger

# Call service manually
ros2 service call /robot_status std_srvs/Trigger
# Output:
# requester: Making request: std_srvs.srv.Trigger_Request()
# response:
# std_srvs.srv.Trigger_Response(success=True, message='Battery: 85%, CPU: 45%, Memory: 60%')
```

---

## Part 4: Verification Checklist

- [ ] Server starts without errors
- [ ] Client connects to server
- [ ] Service responds with robot status
- [ ] Client receives complete response
- [ ] `ros2 service list` shows `/robot_status`
- [ ] CLI service call works

---

## Part 5: Troubleshooting

| Issue | Solution |
|-------|----------|
| `Service not found` | Make sure server is running in separate terminal |
| `Connection refused` | Check ROS_DOMAIN_ID is same on both |
| `AttributeError: Trigger` | Import `std_srvs.srv` properly |
| Client doesn't respond | Server might be stuck; restart it |

---

## Part 6: Experiment

Try these modifications:

1. **Add parameters to response**: Modify server to add timestamp
2. **Call service multiple times**: Modify client to make 3 calls
3. **Add error handling**: Make server return `success=False` when battery low
4. **Use custom service type**: Create your own `.srv` file with more fields

---

## Summary

You've created:
- ✅ Service server responding to requests
- ✅ Service client making requests
- ✅ Request-reply communication pattern

**Key Concepts**:
- Services are synchronous (client waits for response)
- Good for request-reply, not streaming
- Both server and client needed for communication

**What's Next**: Lab 1.3 teaches creating complete ROS 2 packages with proper organization.

---

## Navigation

- **Previous Lab**: [Lab 1.1: Your First Node](./lab-1-1-your-first-node.md)
- **Next Lab**: [Lab 1.3: Create a ROS 2 Package](./lab-1-3-create-a-ros2-package.md)
