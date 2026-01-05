---
id: module-3-isaac-ros
title: "Isaac ROS Integration"
sidebar_position: 4
sidebar_label: "Isaac ROS"
description: "Hardware-accelerated perception on edge devices using Isaac ROS and Jetson"
keywords: [Isaac ROS, Jetson, hardware acceleration, perception pipeline, VSLAM, latency]
---

# Isaac ROS Integration

## Introduction

**Isaac ROS** adds hardware acceleration to ROS 2 perception pipelines on Jetson edge devices.

This chapter covers:
- Isaac ROS architecture
- Hardware-accelerated SLAM (VSLAM)
- Image processing on GPU
- DNN inference for detection
- Deployment on Jetson Orin Nano

---

## Learning Outcomes

By the end, you will:
1. Understand Isaac ROS packages
2. Accelerate perception with GPU
3. Measure latency improvements
4. Deploy on edge hardware
5. Validate performance

---

## Part 1: Isaac ROS Architecture

### Concept

**Isaac ROS** = Perception ROS nodes with GPU acceleration

```
Traditional ROS 2 perception:
  Camera → CPU-based image processing → CPU DNN → ROS topic
  Latency: 200-500ms on Jetson

Isaac ROS accelerated:
  Camera → GPU image processing → GPU DNN → ROS topic
  Latency: 20-50ms on Jetson
  Speed-up: 5-10x faster!
```

### Key Packages

| Package | Purpose | Speed-up |
|---------|---------|----------|
| **isaac_ros_image_proc** | Debayer, resize, format conversion | 5x |
| **isaac_ros_visual_slam** | VSLAM (Visual SLAM) | 3x |
| **isaac_ros_dnn_inference** | DNN inference on GPU | 5-10x |
| **isaac_ros_apriltag** | AprilTag detection | 2x |
| **isaac_ros_object_detection** | Object detection (YOLO) | 5x |

---

## Part 2: Isaac ROS VSLAM

### Visual SLAM on Jetson

```bash
# Install Isaac ROS VSLAM
sudo apt install ros-humble-isaac-ros-visual-slam

# Run VSLAM node
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_isaac_sim.launch.py
```

### ROS 2 Interface

```python
import rclpy
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster

class VslamSubscriber(rclpy.node.Node):
    def __init__(self):
        super().__init__('vslam_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            '/visual_slam/odom',
            self.odom_callback,
            10
        )

    def odom_callback(self, msg):
        """Receive VSLAM odometry"""
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        self.get_logger().info(f"Position: ({x:.2f}, {y:.2f}, {z:.2f})")

if __name__ == '__main__':
    rclpy.init()
    node = VslamSubscriber()
    rclpy.spin(node)
```

### Latency Measurement

```python
import time
from sensor_msgs.msg import Image

class LatencyMeasurer:
    def __init__(self):
        self.timestamps = []

    def measure_vslam_latency(self):
        """Measure end-to-end VSLAM latency"""
        # Time image capture
        capture_time = time.time()

        # VSLAM processes it
        # (happens in background)

        # Receive odometry
        odom_time = time.time()

        latency = (odom_time - capture_time) * 1000  # ms
        self.timestamps.append(latency)

        # Statistics
        avg_latency = np.mean(self.timestamps)
        p95_latency = np.percentile(self.timestamps, 95)

        print(f"Avg latency: {avg_latency:.1f}ms, P95: {p95_latency:.1f}ms")

        # Target: Under 50ms for real-time
```

---

## Part 3: Image Processing Pipeline

### Accelerated Image Processing

```bash
# Debayering (camera Bayer → RGB) on GPU
ros2 launch isaac_ros_image_proc isaac_ros_image_proc_launch.py

# Resize, crop, convert format on GPU
# 5x faster than CPU
```

### Complete Pipeline

```python
from isaac_ros_image_proc import DebayerNode, ResizeNode
from isaac_ros_dnn_inference import TrtNode  # TensorRT inference
from isaac_ros_object_detection import YoloNode

# Pipeline:
# Camera (Bayer) → Debayer (GPU) → Resize (GPU) → YOLO (TensorRT GPU) → Detection

class PerceptionPipeline:
    def __init__(self):
        # Create nodes
        self.debayer = DebayerNode()
        self.resize = ResizeNode(width=640, height=480)
        self.yolo = YoloNode(model_path="yolov3.trt")

        # Connect topics
        self.debayer.image_out → resize.image_in
        self.resize.image_out → yolo.image_in

    def run(self):
        """Process images through pipeline"""
        while True:
            raw_image = get_camera_image()
            rgb_image = self.debayer.process(raw_image)
            resized = self.resize.process(rgb_image)
            detections = self.yolo.process(resized)
            publish_detections(detections)
```

---

## Part 4: DNN Inference

### TensorRT Optimization

```python
# Convert YOLO model to TensorRT for Jetson
import tensorrt as trt

def convert_to_tensorrt(onnx_model_path):
    """Convert ONNX to TensorRT for fast inference"""
    TRT_LOGGER = trt.Logger(trt.Logger.WARNING)
    builder = trt.Builder(TRT_LOGGER)
    network = builder.create_network()
    parser = trt.OnnxParser(network, TRT_LOGGER)

    with open(onnx_model_path, 'rb') as model:
        parser.parse(model.read())

    # FP16 precision (faster)
    builder.fp16_mode = True

    # Build engine
    engine = builder.build_cuda_engine(network)
    return engine

# Result: 5-10x speedup on Jetson!
```

### Latency on Hardware

| Device | Model | Latency |
|--------|-------|---------|
| **Jetson Orin Nano (CPU)** | YOLOv3 | 200ms |
| **Jetson Orin Nano (TensorRT)** | YOLOv3 | 30ms |
| **Desktop GPU (RTX 4070)** | YOLOv3 | 5ms |

---

## Part 5: Deployment on Jetson

### Jetson Setup

```bash
# 1. Flash JetPack 5.1 to Jetson Orin Nano
# 2. Install ROS 2 Humble
sudo apt install ros-humble-desktop

# 3. Install Isaac ROS
mkdir -p ~/ws/isaac_ros && cd ~/ws/isaac_ros
git clone https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common
cd isaac_ros_common && bash ./scripts/setup_container.sh

# 4. Build perception packages
colcon build --packages-select isaac_ros_visual_slam
```

### Run on Jetson

```bash
# SSH into Jetson
ssh nvidia@jetson_ip

# Run perception pipeline
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam_jetson.launch.py \
  camera_topic:=/camera/image_raw

# Monitor performance
ros2 run isaac_ros_benchmark benchmark_visual_slam
```

---

## Part 6: Performance Validation

### Benchmark Results

```python
def benchmark_perception():
    """Measure latency and throughput"""
    timestamps = []

    for i in range(1000):
        t_start = time.perf_counter()

        # Run perception (VSLAM + detection)
        odom = process_frame()

        t_end = time.perf_counter()
        latency = (t_end - t_start) * 1000  # ms
        timestamps.append(latency)

    print(f"Mean latency: {np.mean(timestamps):.1f}ms")
    print(f"P99 latency:  {np.percentile(timestamps, 99):.1f}ms")
    print(f"Max latency:  {np.max(timestamps):.1f}ms")
    print(f"Throughput:   {1000/np.mean(timestamps):.1f} Hz")

# Target metrics:
# - Mean latency: Under 50ms
# - P99: Under 100ms
# - Throughput: >20 Hz
```

---

## Summary

**Isaac ROS**:
- GPU-accelerated perception
- 5-10x latency reduction
- Ideal for Jetson deployment

**Key packages**:
- VSLAM for localization
- Image processing acceleration
- DNN inference via TensorRT

**Performance**:
- Under 50ms latency on Jetson
- Real-time perception possible
- Edge computing for robotics

**Next**: Object detection and grasping.

---

## Navigation

- **Previous**: [Chapter 3: SLAM](./03-slam-and-autonomous-navigation.md)
- **Next**: [Chapter 5: Object Detection](./05-object-detection-and-manipulation.md)
