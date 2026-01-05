---
id: module-2-intro
title: "Module 2: The Digital Twin (Gazebo & Unity)"
sidebar_position: 0
sidebar_label: "The Digital Twin (Gazebo & Unity)"
description: "Master physics-based robot simulation using Gazebo and explore high-fidelity rendering with Unity"
keywords:
  [simulation, Gazebo, physics, URDF, sensors, Unity, robotics, capstone]
---

# Module 2: The Digital Twin (Gazebo & Unity)

## Welcome to Simulation

You've mastered **ROS 2 communication** (Module 1). Now learn to **simulate robots** before deploying them to real hardware.

This module teaches:

- **Gazebo**: Physics-accurate robot simulation
- **URDF**: Robot description language
- **Sensors**: Simulating cameras, LiDAR, IMU
- **Unity**: High-fidelity rendering for visualization
- **Sim-to-Real**: Bridging simulation and physical robots

**Time commitment**: 2 weeks (Weeks 6–7)
**Hands-on content**: 3 labs, heavy Gazebo usage
**Capstone connection**: Your robot will be simulated in Gazebo before capstone submission

---

## Module Learning Outcomes

By the end of Module 2, you will be able to:

1. **Set up Gazebo environments** and understand the simulation loop
2. **Write URDF files** describing robot structure, joints, and sensors
3. **Simulate sensors** (camera, LiDAR, IMU) and publish ROS 2 topics
4. **Control simulated robots** from ROS 2 nodes
5. **Understand sim-to-real transfer** and bridge simulation to hardware
6. **Use Unity for high-fidelity visualization** when physics accuracy is less critical

---

## Why Simulation Matters

### The Problem: Real Robots Are Expensive

- **Cost**: $5,000–$100,000+
- **Safety**: Bugs cause physical crashes
- **Iteration**: Slow feedback loop (10 min to deploy, test, modify)
- **Access**: Not all students have hardware

### The Solution: Simulation First

- **Free**: Gazebo and most simulators are open-source
- **Safe**: Crash in simulation, learn from it
- **Fast**: Test changes in seconds, not minutes
- **Accessible**: Run on laptop; no special hardware

### Real-World Example: Robotics Companies

- **Boston Dynamics**: Tests in simulation first, then deploys
- **Tesla**: Simulates robotics before manufacturing scale-up
- **Waymo**: Validates autonomous driving in simulation millions of times
- **Your capstone**: You'll follow the same pattern

---

## Chapter Breakdown

### Chapter 1: Gazebo Basics & Physics

**Focus**: Setting up simulation environments

- What is Gazebo? Why use it for robotics?
- Physics engines (ODE, Bullet, DART)
- Creating worlds, spawning models
- Gravity, friction, collision simulation
- Running Gazebo from ROS 2 launch files

**Reading time**: ~40 minutes
**Key takeaway**: Gazebo is your physics simulation sandbox

---

### Chapter 2: URDF & Robot Descriptions

**Focus**: Defining robot structure

- What is URDF? (Unified Robot Description Format)
- Links (rigid bodies) and joints (connections)
- Inertial properties and mass
- Sensors in URDF (camera, LiDAR, IMU)
- From URDF to Gazebo simulation

**Reading time**: ~45 minutes
**Key takeaway**: URDF tells Gazebo how your robot is built

---

### Chapter 3: Sensors in Gazebo

**Focus**: Simulating robot perception

- Camera simulation (RGB, depth images)
- LiDAR (laser scan simulation)
- IMU (accelerometer, gyroscope)
- Publishing sensor data to ROS 2 topics
- Sensor plugins and configuration

**Reading time**: ~40 minutes
**Key takeaway**: Simulated sensors feed your perception pipeline

---

### Chapter 4: Unity for Robotics

**Focus**: When to use high-fidelity rendering

- Gazebo vs. Unity: tradeoffs
- Why use Unity? (stunning graphics, HRI visualization)
- ROS 2 integration in Unity
- When physics accuracy matters vs. when visuals do
- Simulation for human-robot interaction

**Reading time**: ~30 minutes
**Key takeaway**: Choose the right tool for your simulation goal

---

### Chapter 5: Sim-to-Real Considerations

**Focus**: Bridging simulation and physical robots

- The "reality gap" (why simulation ≠ real world)
- Domain randomization (training for variety)
- Validation on hardware
- Transfer learning from sim to real
- Safety validation before deployment

**Reading time**: ~35 minutes
**Key takeaway**: Simulation enables learning; hardware validates it

---

### Module 2 Labs

#### Lab 2.1: Load a Robot in Gazebo

- Create a simple humanoid URDF
- Spawn in Gazebo
- Apply forces, observe physics
- Verify inertia and collisions

#### Lab 2.2: Publish Sensor Data

- Add camera and LiDAR to URDF
- Simulate sensor readings
- Visualize with ROS 2 tools
- Verify data quality

#### Lab 2.3: Control Robot in Simulation

- Send ROS 2 velocity commands
- Watch robot move in Gazebo
- Implement simple control loop
- Verify joint limits and safety

---

## How This Module Connects to Your Capstone

### Module 2 Contribution: Simulation Environment

Your capstone robot will run in Gazebo:

**Week 6–7**: Gazebo + URDF setup
→ You build simulation of your robot structure

**Week 8–10**: Sensor simulation + perception
→ Camera and LiDAR feed into perception pipeline

**Week 11–13**: Full system in simulation
→ Voice control works in simulated world before real deployment

**Week 13 Capstone**: Submit simulation demo + code + report

---

## Prerequisites & Self-Check

### Required Knowledge (from Module 1)

- ✅ ROS 2 nodes and topics
- ✅ Launch files and parameter passing
- ✅ Basic Python and Linux terminal

### New Tools You'll Learn

- **Gazebo**: Physics simulation
- **URDF**: XML-based robot descriptions
- **RViz**: ROS 2 visualization
- **Xacro** (optional): Macros for URDF simplification

### Systems You'll Need

- Ubuntu 22.04 with ROS 2 Humble
- Gazebo (installed with ROS 2)
- Text editor (VS Code, nano, vim)
- Graphics card helpful (but integrated GPU works)

---

## Module Structure

```
6 chapters (theory + examples)
    ↓
3 hands-on labs (progressive difficulty)
    ↓
Complete URDF + Gazebo world
    ↓
Control simulated robot from ROS 2
    ↓
Ready for Module 3 (perception)
```

---

## Time Commitment

**Per Week**: 5–7 hours

- **Lectures/Reading**: 2–2.5 hours
- **Labs/Simulation**: 2.5–3 hours
- **Practice/Experimentation**: 1–1.5 hours

**Heavy Lab Weeks** (Weeks 6–7):

- Expect 7–9 hours/week (Gazebo debugging is time-consuming)

---

## Key Concepts Preview

### Gazebo Simulation Loop

```
1. Load URDF (robot description)
2. Apply physics (gravity, forces)
3. Integrate physics (100 Hz typically)
4. Publish sensor data (camera, LiDAR, IMU)
5. ROS 2 node receives sensor topics
6. ROS 2 node publishes motor commands
7. Gazebo applies forces to motors
8. Repeat at high frequency
```

### The URDF Pipeline

```
robot.urdf (XML)
    ↓
URDF Parser
    ↓
Link tree + Joint constraints
    ↓
Gazebo physics engine
    ↓
Simulated robot in 3D world
```

---

## Learning Resources

### Official Documentation

- [Gazebo Documentation](https://gazebosim.org/docs)
- [ROS 2 + Gazebo Integration](https://docs.ros.org/en/humble/Tutorials/Advanced/Simulators/Gazebo/Gazebo.html)
- [URDF Tutorial](http://wiki.ros.org/urdf/Tutorials)

### Interactive Resources

- Gazebo tutorials (included with installation)
- Example URDFs in `ros_tutorials` package
- RViz for sensor visualization

---

## Support & Troubleshooting

**Common Issues in Module 2**:

- Gazebo crashes or runs slowly → Graphics driver update
- Robot falls through ground → Collision shapes or inertia issue
- Sensors don't publish → Plugin configuration
- ROS 2 connection fails → Domain ID or network issue

**We'll cover troubleshooting** in each lab section.

---

## Next Steps

1. **Review Module 1**: Ensure ROS 2 fundamentals solid
2. **Install Gazebo** (if not already done):
   ```bash
   sudo apt install ros-humble-gazebo-*
   ```
3. **Test Gazebo**:
   ```bash
   gazebo --version
   ```
4. **Start Chapter 1**: Gazebo Basics & Physics

---

## Navigation

- **Previous Module**: [Module 1 Summary](../module-1-ros2/07-module-1-summary.md)
- **Next**: [Chapter 1: Gazebo Basics & Physics](./01-gazebo-basics-and-physics.md)
- **Capstone**: [Capstone Requirements](../capstone/01-requirements.md)

---

## Quick Stats

| Metric                | Value                       |
| --------------------- | --------------------------- |
| **Module duration**   | 2 weeks                     |
| **Chapters**          | 5 + 3 labs                  |
| **Estimated reading** | 3–4 hours                   |
| **Lab time**          | 8–10 hours                  |
| **Robot to simulate** | Generic humanoid (provided) |
| **Sensors simulated** | Camera, LiDAR, IMU          |
| **Physics engines**   | ODE, Bullet, DART (use ODE) |

---

**Welcome to simulation!** 🎮

By the end of this module, you'll understand how the world's leading robotics companies validate their robots safely and efficiently.

Let's begin! 🚀
