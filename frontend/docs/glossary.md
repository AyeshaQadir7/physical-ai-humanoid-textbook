---
id: glossary
title: "Glossary"
sidebar_position: 100
sidebar_label: "Glossary"
description: "Comprehensive glossary of robotics, ROS 2, and AI terminology used in this textbook"
keywords: [glossary, definitions, ROS 2, robotics, SLAM, Isaac, VLA]
---

## Glossary: Physical AI & Humanoid Robotics

This glossary defines technical terms used throughout the textbook. Terms are organized by category for easy reference.

---

## Embodied Intelligence & Physical AI

### Embodied Intelligence
The capability of an AI system to understand and act on the physical world through a robotic body, integrating perception, reasoning, and control in real-time.

### Physical AI
A branch of artificial intelligence focused on systems that interact with and control the physical world, combining digital learning with hardware actuation.

### Sim-to-Real Transfer
The process of training a robot control system in simulation and successfully deploying it to a real robot, accounting for differences between simulation and reality (the "reality gap").

### Reality Gap
The discrepancy between simulated robot behavior and actual physical behavior, caused by imperfect physics modeling, sensor inaccuracy, and environmental unpredictability.

---

## ROS 2 (Robot Operating System 2)

### ROS 2
Robot Operating System 2; a flexible middleware framework for building modular robot applications with support for real-time constraints, security, and distributed computing.

### Node
A ROS 2 process that performs a specific computational task (e.g., reading a sensor, controlling a motor, processing vision data). Nodes communicate via topics and services.

### Topic
A named bus for communication between ROS 2 nodes. Nodes can **publish** messages to a topic or **subscribe** to receive messages, enabling asynchronous, decoupled communication.

### Message
A data structure published to a topic or sent via a service. Examples: `sensor_msgs/Image` (camera data), `geometry_msgs/Twist` (velocity commands).

### Publisher
A ROS 2 object in a node that publishes (sends) messages to a topic at regular intervals.

### Subscriber
A ROS 2 object in a node that listens to messages published on a topic and triggers a callback function when a message arrives.

### Service
A request-response communication pattern in ROS 2. One node (server) waits for requests, processes them, and sends back a response. Example: `spawn_entity` service in Gazebo.

### Action
A asynchronous request-response pattern in ROS 2 supporting long-running tasks with feedback. Unlike services, actions can provide intermediate updates and allow cancellation.

### Action Server
A ROS 2 node component that receives action requests, executes them, and returns results with periodic feedback (e.g., a navigation server executing move commands).

### Action Client
A ROS 2 component that requests an action from an action server, receives feedback during execution, and gets the final result.

### Package
A directory containing ROS 2 code, configuration files, and metadata (`package.xml`). A package is the unit of organization and distribution in ROS 2.

### DDS (Data Distribution Service)
The middleware protocol underlying ROS 2 for reliable, efficient, real-time communication across a network without a centralized message broker.

### rclpy
The Python client library for ROS 2, allowing Python developers to create nodes and communicate with other ROS 2 components.

### rclcpp
The C++ client library for ROS 2, used for performance-critical real-time applications.

### Launch File
An XML file that specifies which ROS 2 nodes to start, their parameters, and how they connect. Simplifies starting complex multi-node systems with one command.

### Parameter
A configuration value in ROS 2 that can be set at runtime. Parameters allow nodes to adapt behavior without code changes.

### Parameter Server
A ROS 2 service that stores and distributes parameters across nodes.

### rqt
A Qt-based GUI framework for ROS 2 introspection and control. Tools like `rqt_graph`, `rqt_topic`, and `rqt_image_view` help visualize and debug running systems.

### rostopic
A ROS 2 command-line tool for inspecting and interacting with topics (e.g., `rostopic echo`, `rostopic list`).

### rosservice
A ROS 2 command-line tool for calling services (e.g., `rosservice call`).

### rosnode
A ROS 2 command-line tool for inspecting nodes (e.g., `rosnode list`, `rosnode info`).

---

## Robotics Fundamentals

### Humanoid Robot
A robot designed with a body structure resembling a human, typically with two arms, two legs, a torso, and a head, enabling it to interact with human-designed environments.

### Kinematics
The branch of mechanics describing the motion of a robot without considering forces. Forward kinematics calculates end-effector position from joint angles; inverse kinematics solves the reverse.

### Dynamics
The branch of mechanics describing how forces and torques cause motion in a robot. Essential for control systems that must balance forces and accelerations.

### Joint
A movable connection between two links of a robot. Common types: revolute (rotation), prismatic (linear), continuous (unlimited rotation).

### Link
A rigid body segment of a robot connected to other links via joints. Example: upper arm, forearm, gripper.

### End-Effector
The tool or appendage at the end of a robotic arm (e.g., gripper, camera, laser pointer).

### Actuator
A motor or solenoid that converts electrical or pneumatic energy into mechanical motion to move robot joints.

### Sensor
A device that measures physical quantities (e.g., position, velocity, force, light, sound) and converts them to electrical signals for processing.

### URDF (Unified Robot Description Format)
An XML format for describing robot structure, including links, joints, sensors, and visual/collision properties. Used by ROS 2 and simulation engines.

### SDF (Simulation Description Format)
An XML format for describing robotic worlds, including robots, objects, physics, and lighting. More powerful than URDF; used in Gazebo.

### SLAM (Simultaneous Localization and Mapping)
A technique for a robot to build a map of its environment while simultaneously determining its own position within that map, essential for autonomous navigation.

### Localization
The process of determining a robot's position and orientation in a known environment (or map).

### Mapping
The process of creating a map of an environment (typically using LiDAR or camera data) for navigation and obstacle avoidance.

### Visual Odometry (VO)
Estimating a robot's motion and position using only camera images, without requiring external markers or prior maps.

### Visual SLAM (VSLAM)
A variant of SLAM using camera images as the primary sensor, combining visual odometry with loop closure and mapping.

### Path Planning
The algorithm for computing a safe, efficient path from a robot's current position to a goal while avoiding obstacles.

### Inverse Kinematics (IK)
Calculating the joint angles required to reach a desired end-effector position and orientation.

### Forward Kinematics (FK)
Computing the end-effector position and orientation from given joint angles.

### Collision Avoidance
Techniques to prevent a robot from colliding with obstacles or itself during motion.

---

## Simulation & Physics

### Gazebo
A 3D simulation environment for robotics with built-in physics engines, sensor simulation, and tight integration with ROS 2. Used to test robot behavior before physical deployment.

### Physics Engine
Software that simulates real-world physics (gravity, collisions, friction) in a virtual environment. Gazebo supports ODE, Bullet, DART, and Simbody.

### ODE (Open Dynamics Engine)
An open-source physics engine used in Gazebo for simulating rigid body dynamics and collisions.

### Bullet Physics
An open-source physics engine known for accuracy and speed, usable with Gazebo.

### DART Physics
A physics engine emphasizing accurate dynamics simulation, used in Gazebo and research.

### Mesh
A 3D geometric representation of an object made of vertices, edges, and faces. Used for collision detection and visualization.

### Inertia
A property describing how mass is distributed in an object, affecting rotational dynamics. Every link in a URDF must specify inertia.

### Gravity
Downward acceleration (9.81 m/s² on Earth) simulated in physics engines to create realistic falling and support reactions.

### Friction
Resistance to motion between two surfaces. Gazebo allows tuning friction coefficients.

### Collision
Detection and response when two objects physically intersect, critical for realistic simulation.

### RGB-D Camera
A camera providing both color (RGB) and depth (D) information, essential for 3D vision tasks.

### Point Cloud
A set of 3D points representing a sensed environment, typically from LiDAR or RGB-D cameras.

### LiDAR (Light Detection and Ranging)
A sensor using laser pulses to measure distances to objects, producing a point cloud. Essential for SLAM and obstacle detection.

### IMU (Inertial Measurement Unit)
A sensor measuring linear acceleration and angular velocity, used for tracking robot orientation and motion.

### Depth Sensor
A camera (like Intel RealSense) providing depth information at every pixel, useful for 3D vision and obstacle avoidance.

---

## NVIDIA Isaac Platform

### NVIDIA Isaac Sim
A photorealistic simulation environment for robots, built on NVIDIA Omniverse, supporting synthetic data generation, hardware-accelerated physics, and AI-based perception training.

### NVIDIA Isaac ROS
A set of ROS 2 packages providing hardware-accelerated perception and control, optimized for NVIDIA Jetson processors and data center GPUs.

### Omniverse
NVIDIA's platform for 3D simulation and collaboration, providing rendering, physics, and AI capabilities for robotics simulation.

### Nucleus
NVIDIA's asset library and content management system, providing 3D models, materials, and environments for use in Isaac Sim.

### Synthetic Data
Artificially generated data (images, sensor readings) from simulation, used to train machine learning models without requiring real hardware.

### Hardware-Accelerated Perception
Using GPU acceleration for real-time image processing, object detection, and SLAM, critical for edge deployment.

### Visual Odometry (Isaac ROS)
A hardware-accelerated visual odometry implementation in Isaac ROS for real-time camera-based localization.

### VPX (Voxel Perception Engine)
An NVIDIA Isaac ROS module for GPU-accelerated 3D perception and scene understanding.

---

## Vision & Computer Vision

### Object Detection
Identifying and localizing objects in an image, returning bounding boxes and class labels. Example: YOLO, Faster R-CNN.

### YOLO (You Only Look Once)
A real-time object detection algorithm, commonly used in robotics for fast detection of objects in camera feeds.

### Semantic Segmentation
Classifying every pixel in an image into semantic categories (person, car, tree, etc.), useful for scene understanding.

### Instance Segmentation
Distinguishing individual objects of the same class (e.g., detecting and separating two cups in an image).

### Optical Flow
Estimating pixel-level motion between consecutive frames, used for visual odometry and motion detection.

### Feature Detection
Identifying distinctive points or regions in an image (keypoints) used for matching images and visual odometry.

### Image Processing
Algorithms for enhancing, filtering, or transforming images (e.g., edge detection, histogram equalization).

---

## Vision-Language-Action (VLA) Systems

### Vision-Language-Action (VLA)
A multimodal AI system that combines vision (image input), language (natural language commands), and action (robot motor commands) to enable robots to understand and execute human instructions.

### Multimodal AI
AI systems that process multiple types of input (vision, language, audio, proprioception) to make decisions or generate outputs.

### Large Language Model (LLM)
A deep learning model trained on vast amounts of text, capable of understanding and generating human language. Examples: GPT-4, Claude, Llama.

### Prompt Engineering
Crafting input prompts to an LLM to elicit desired behavior, critical for robotics where the model must generate executable actions.

### Grounding
Connecting abstract language concepts to concrete perceptual or motor representations, e.g., linking the word "grasp" to specific robot joint commands.

### Action Space
The set of all possible actions a robot can execute (e.g., joint velocities, gripper commands).

### Semantic Understanding
Extracting meaning from language or images beyond surface-level patterns, essential for generalization.

### Transfer Learning
Adapting a model trained on one task to perform a different task, reducing data and computation required.

### Few-Shot Learning
Learning to perform a new task from only a few examples, important for robots to adapt to new scenarios.

---

## Speech & Audio

### Automatic Speech Recognition (ASR)
Converting spoken language (audio) into text, essential for voice-controlled robots.

### Whisper (OpenAI)
An open-source ASR model capable of transcribing speech in multiple languages, commonly used in VLA robotics.

### Text-to-Speech (TTS)
Converting text into spoken audio, used for robots to provide voice feedback.

### ReSpeaker
A microphone array (e.g., 6-mic circular array) optimized for voice input, commonly used in robotics for directional audio.

### Voice Commands
Human instructions given verbally to a robot, processed by ASR and then interpreted by an LLM.

### Audio Processing
Techniques for filtering, enhancing, or analyzing audio signals (e.g., noise reduction, beamforming).

---

## Hardware & Platforms

### Jetson Orin Nano
NVIDIA's edge AI processor for robotics, providing GPU acceleration for perception and inference on a small, power-efficient platform.

### Jetson Orin Nano Developer Kit
A single-board computer with NVIDIA Orin Nano GPU, used for developing robotics applications with hardware acceleration.

### Unitree G1
A humanoid robot platform developed by Unitree, featuring full-body control, balance, and multimodal sensing for research and development.

### Boston Dynamics Atlas
A state-of-the-art humanoid research platform, showcasing advanced mobility and manipulation.

### Tesla Optimus (Bot)
A humanoid robot under development by Tesla, targeting general-purpose manipulation tasks.

### RealSense D435i
An Intel RGB-D depth camera commonly used in robotics for 3D vision and obstacle detection.

### Ubuntu 22.04 LTS
The recommended operating system for ROS 2 Humble, providing a stable, long-term support base for robotics development.

### Docker
A containerization platform for packaging robots software and dependencies in isolated, reproducible environments.

---

## Machine Learning & AI

### Deep Learning
Machine learning using neural networks with multiple layers, powerful for vision and language tasks.

### Convolutional Neural Network (CNN)
A deep learning architecture specialized for processing grid-like data (images), using convolutional layers to extract spatial features.

### Transformer
A neural network architecture based on attention mechanisms, effective for sequence processing (language, time-series).

### Attention Mechanism
A technique for models to focus on relevant parts of input, foundational for modern LLMs and vision transformers.

### Fine-Tuning
Adapting a pre-trained model to a specific task with task-specific data, more efficient than training from scratch.

### Model Quantization
Reducing the precision of model weights (e.g., from 32-bit floats to 8-bit integers) to decrease memory and computation, enabling edge deployment.

### Inference
Running a trained model on new data to make predictions or generate outputs, as opposed to training.

### Latency
The time delay between input and output, critical for real-time robotic control (must be under 100ms typically).

### Throughput
The number of samples processed per unit time, important for high-frequency sensor data processing.

---

## Control & Systems

### Control System
A system that regulates a robot's behavior by receiving sensor feedback and adjusting actuator commands.

### Feedback Control
Using sensor measurements (feedback) to adjust motor commands, enabling stability and accuracy.

### Feedforward Control
Pre-planning motor commands based on a model without real-time feedback; less robust but potentially faster.

### Closed-Loop Control
A control system with feedback, allowing real-time adjustments; more stable and adaptive.

### Open-Loop Control
A control system without feedback, simply executing pre-planned commands; sensitive to disturbances.

### PID Control
Proportional-Integral-Derivative control, a classic feedback control algorithm adjusting output based on error, error integral, and error rate.

### Stability
The property of a control system where disturbances don't cause unbounded behavior; critical for safe robot operation.

### Tracking
The ability of a robot to follow a desired trajectory or target position with high accuracy.

### Response Time
The time for a robot to react to a command or stimulus, important for real-time interaction.

### Bandwidth
The frequency range of disturbances or commands a control system can effectively handle.

---

## Safety & Ethics

### Safety-Critical System
A system where failures could cause injury or property damage, requiring rigorous validation and redundant safeguards.

### Collision Avoidance
Techniques preventing a robot from contacting objects or people during operation.

### Force Limiting
Hardware or software limiting the maximum force a robot can apply, protecting people and objects.

### Emergency Stop (E-Stop)
A button or mechanism to immediately halt robot motion in case of danger.

### Validation
Ensuring a system meets intended requirements and behaves safely in real-world conditions.

### Risk Mitigation
Strategies to reduce the likelihood and severity of potential hazards (e.g., redundancy, failsafes, training).

---

## Development & Software Engineering

### Package Manager
Software (like `apt`, `pip`, `conda`) that installs and manages libraries and dependencies.

### Colcon
A build tool for ROS 2 packages, replacing catkin, supporting C++ and Python packages in a single workspace.

### Build System
Software (like CMake) managing compilation, linking, and packaging of source code.

### Continuous Integration (CI)
Automatically building and testing code on every commit, catching errors early.

### Version Control
Tracking changes to code over time, typically using Git and GitHub.

### API (Application Programming Interface)
The interface a library or service provides, specifying how to call functions and pass data.

### JSON (JavaScript Object Notation)
A human-readable data format commonly used for configuration and data exchange.

### YAML (YAML Ain't Markup Language)
A human-readable data format used for ROS 2 configuration files (launch files, parameters).

### XML (eXtensible Markup Language)
A structured data format used for URDF, SDF, and ROS 2 launch files.

### Markdown
A lightweight text format used for documentation, including this textbook.

### Git
A version control system for tracking code changes and collaborating with others.

### GitHub
A cloud platform for hosting Git repositories, enabling collaboration and code sharing.

---

## Acronyms & Abbreviations

| Acronym | Full Form |
| --- | --- |
| ASR | Automatic Speech Recognition |
| CNN | Convolutional Neural Network |
| DDS | Data Distribution Service |
| FK | Forward Kinematics |
| GPU | Graphics Processing Unit |
| IK | Inverse Kinematics |
| IMU | Inertial Measurement Unit |
| JSON | JavaScript Object Notation |
| LiDAR | Light Detection and Ranging |
| LLM | Large Language Model |
| NLP | Natural Language Processing |
| ODE | Open Dynamics Engine |
| PID | Proportional-Integral-Derivative |
| ROS | Robot Operating System |
| RGB-D | Red-Green-Blue-Depth |
| SLAM | Simultaneous Localization and Mapping |
| SDF | Simulation Description Format |
| TTS | Text-to-Speech |
| URDF | Unified Robot Description Format |
| VO | Visual Odometry |
| VSLAM | Visual SLAM |
| VLA | Vision-Language-Action |
| YAML | YAML Ain't Markup Language |

---

## Further Learning

For deeper understanding of robotics and AI concepts, consult:

- **ROS 2 Official Documentation**: [https://docs.ros.org](https://docs.ros.org)
- **Gazebo Documentation**: [https://gazebosim.org](https://gazebosim.org)
- **NVIDIA Isaac Documentation**: [https://developer.nvidia.com/isaac](https://developer.nvidia.com/isaac)
- **Introduction to Robotics (Spong et al.)**: Classical textbook on kinematics, dynamics, and control
- **Probabilistic Robotics (Thrun et al.)**: SLAM and probabilistic reasoning for robots

---

**Last Updated**: 2025-12-10
**Maintained By**: Textbook Team
