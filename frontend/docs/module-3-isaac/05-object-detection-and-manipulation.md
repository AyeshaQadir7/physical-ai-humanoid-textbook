---
id: module-3-detection
title: "Object Detection & Manipulation"
sidebar_position: 5
sidebar_label: "Detection & Grasping"
description: "Computer vision for object detection, segmentation, and grasp planning"
keywords: [object detection, YOLO, segmentation, grasping, vision, robotics]
---

# Object Detection & Manipulation

## Introduction

Your robot can now see and navigate. Now teach it to **detect objects** and **grasp them**.

This chapter covers:
- Object detection networks (YOLO, Mask R-CNN)
- Semantic and instance segmentation
- 3D pose estimation
- Grasp planning
- Integration with navigation and control

---

## Learning Outcomes

By the end, you will:
1. Use YOLO for real-time detection
2. Segment object instances
3. Estimate 6DOF object poses
4. Plan grasps using vision
5. Integrate detection with control

---

## Part 1: Object Detection with YOLO

### YOLOv3 Architecture

```
Input: 640x480 RGB image
    ↓
Feature extraction (backbone)
    ↓
Multi-scale detection heads
    ↓
Output: Bounding boxes + class probabilities
```

### ROS 2 Object Detection Node

```python
import cv2
import rclpy
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D
from cv_bridge import CvBridge

class YoloDetector(rclpy.node.Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.publisher = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        self.net = cv2.dnn.readNet("yolov3.weights", "yolov3.cfg")
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg)

        # YOLO inference
        blob = cv2.dnn.blobFromImage(cv_image, 1/255.0, (416, 416))
        self.net.setInput(blob)
        layer_names = self.net.getLayerNames()
        output_layers = [layer_names[i-1] for i in self.net.getUnconnectedOutLayers()]
        outputs = self.net.forward(output_layers)

        # Parse detections
        detections = Detection2DArray()
        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > 0.5:
                    x, y, w, h = detection[0:4] * np.array([640, 480, 640, 480])
                    det = Detection2D()
                    det.bbox.center.position.x = float(x)
                    det.bbox.center.position.y = float(y)
                    det.bbox.size_x = float(w)
                    det.bbox.size_y = float(h)
                    detections.detections.append(det)

        self.publisher.publish(detections)

if __name__ == '__main__':
    rclpy.init()
    node = YoloDetector()
    rclpy.spin(node)
```

---

## Part 2: Instance Segmentation

### Mask R-CNN

```python
from torchvision.models.detection import maskrcnn_resnet50_fpn

# Load pre-trained Mask R-CNN
model = maskrcnn_resnet50_fpn(weights='COCO_INSTANCE_SEGMENTATION_V1')

# Process image
image_tensor = torch.from_numpy(cv_image).permute(2, 0, 1) / 255.0
predictions = model([image_tensor])

# Results
boxes = predictions[0]['boxes']        # [N, 4] bounding boxes
scores = predictions[0]['scores']      # [N] confidence scores
labels = predictions[0]['labels']      # [N] class IDs
masks = predictions[0]['masks']        # [N, H, W] segmentation masks

# Filter high-confidence
keep = scores > 0.5
boxes = boxes[keep]
masks = masks[keep]
```

### Segmentation Visualization

```python
def visualize_segmentation(image, masks, boxes):
    """Draw segmentation masks"""
    output = image.copy()

    for mask, box in zip(masks, boxes):
        # Create colored mask
        mask_img = (mask[0].cpu().numpy() * 255).astype(np.uint8)
        mask_colored = cv2.applyColorMap(mask_img, cv2.COLORMAP_JET)

        # Blend with image
        output = cv2.addWeighted(output, 0.7, mask_colored, 0.3, 0)

        # Draw bounding box
        x1, y1, x2, y2 = box.cpu().numpy().astype(int)
        cv2.rectangle(output, (x1, y1), (x2, y2), (0, 255, 0), 2)

    return output
```

---

## Part 3: 6DOF Pose Estimation

### Using Depth + Detection

```python
def estimate_object_pose(detection, depth_image):
    """
    Estimate 3D position and orientation of detected object
    """
    x_center, y_center = detection.center

    # Get depth at detection center
    z = depth_image[int(y_center), int(x_center)]  # meters

    # Backproject to 3D using camera intrinsics
    fx, fy = camera_intrinsics['fx'], camera_intrinsics['fy']
    cx, cy = camera_intrinsics['cx'], camera_intrinsics['cy']

    x = (x_center - cx) * z / fx
    y = (y_center - cy) * z / fy

    # 3D position in camera frame
    position_camera = np.array([x, y, z])

    # Transform to robot frame
    position_robot = transform_camera_to_robot(position_camera)

    return position_robot  # [3] xyz position
```

---

## Part 4: Grasp Planning

### Grasp Strategy: Top-Down Grasping

```python
def plan_grasp(object_position, object_width):
    """
    Plan top-down grasp on detected object

    Assumes gripper can approach from above
    """
    # Grasp position: above object
    grasp_position = object_position.copy()
    grasp_position[2] += 0.15  # 15cm above object

    # Gripper orientation: vertical (z-axis down)
    grasp_orientation = quaternion_from_euler(0, 0, 0)  # No rotation

    # Gripper width: slightly more than object
    gripper_width = object_width + 0.02  # 2cm safety margin

    return grasp_position, grasp_orientation, gripper_width
```

### Grasp Execution via ROS 2

```python
import rclpy
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

class GraspController(rclpy.node.Node):
    def __init__(self):
        super().__init__('grasp_controller')
        self.arm_controller = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

    def execute_grasp(self, grasp_position, gripper_width):
        """Move arm to grasp pose and close gripper"""
        # 1. Move to grasp position (using inverse kinematics)
        joint_angles = solve_ik(grasp_position)
        self.move_arm(joint_angles)

        # 2. Close gripper
        self.close_gripper(gripper_width)

        # 3. Lift object
        self.move_arm(joint_angles - 0.2)  # Move up 20cm

    def move_arm(self, joint_angles):
        """Publish joint trajectory"""
        traj = JointTrajectory()
        traj.joint_names = ['shoulder', 'elbow', 'wrist']

        point = JointTrajectoryPoint()
        point.positions = joint_angles
        point.time_from_start.sec = 2

        traj.points = [point]
        self.arm_controller.publish(traj)
```

---

## Part 5: Integration: Navigation + Manipulation

### Complete System

```python
class PickAndPlace:
    def __init__(self):
        self.navigator = NavigatorClient()
        self.detector = YoloDetector()
        self.grasp_controller = GraspController()

    def execute_task(self, target_object, goal_location):
        """
        1. Navigate to target object location
        2. Detect object
        3. Grasp object
        4. Navigate to goal location
        5. Place object
        """
        # Step 1: Navigate
        self.navigator.go_to_location("kitchen")  # Approx location

        # Step 2: Detect
        detections = self.detector.find_objects(target_object)
        if not detections:
            self.get_logger().error("Object not detected!")
            return False

        # Step 3: Grasp
        detection = detections[0]
        object_pose = self.detector.estimate_pose(detection)
        grasp_pose, gripper_width = plan_grasp(object_pose)
        self.grasp_controller.execute_grasp(grasp_pose, gripper_width)

        # Step 4: Navigate to goal
        self.navigator.go_to_location(goal_location)

        # Step 5: Place
        self.grasp_controller.open_gripper()

        return True

if __name__ == '__main__':
    rclpy.init()
    system = PickAndPlace()
    success = system.execute_task("coffee_cup", "table")
```

---

## Summary

**Object detection**:
- YOLO: Real-time, fast inference
- Mask R-CNN: Instance segmentation
- ROS 2 integration for pipelines

**Grasping**:
- Top-down grasp planning
- Depth-based 3D positioning
- Gripper control via ROS 2

**Integration**:
- Navigation finds approximate location
- Vision detects exact object
- Grasp executes manipulation

**Next**: Hands-on labs putting it all together.

---

## Navigation

- **Previous**: [Chapter 4: Isaac ROS](./04-isaac-ros-integration.md)
- **Next**: [Lab 3.1: Isaac Environment](./lab-3-1-create-isaac-sim-environment.md)
