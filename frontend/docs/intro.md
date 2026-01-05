---
id: course-intro
title: "Physical AI & Humanoid Robotics Course"
sidebar_position: 1
sidebar_label: "Course Overview"
description: "13-week course integrating ROS 2, simulation, autonomous perception, and AI-based robot control"
keywords: [course, robotics, ROS 2, simulation, Isaac, VLA, humanoid, capstone]
---

# Physical AI & Humanoid Robotics

## Welcome

This course teaches you how to **build and control AI-powered robots** by bridging three worlds:

1. **Digital (AI)**: Large language models, computer vision, speech recognition
2. **Physical (Robotics)**: ROS 2 middleware, robot dynamics, sensor fusion
3. **Hybrid (Embodied AI)**: Connecting language understanding to physical action

Over 13 weeks, you'll develop a **voice-controlled humanoid robot** that understands natural language commands, perceives its environment, and acts autonomously.

---

## Course Overview

### What You'll Build

A **voice-controlled humanoid robot system** where you can say:

> "Robot, walk to the blue cube and pick it up"

And the robot will:
1. ✅ Understand your natural language command
2. ✅ Perceive the blue cube in its environment
3. ✅ Plan a path to reach it (avoiding obstacles)
4. ✅ Execute smooth motion and grasping
5. ✅ Report completion back to you with voice feedback

All of this runs either in **high-fidelity simulation** or on **edge hardware** (Jetson Orin Nano).

### Learning Outcomes

By the end of this course, you will be able to:

- **Master ROS 2**: Build distributed robot systems using pub/sub and service patterns
- **Simulate Robots**: Create physics-accurate virtual environments in Gazebo and Isaac Sim
- **Perceive the World**: Implement SLAM, object detection, and sensor fusion
- **Command with Language**: Use LLMs to map natural language to robot actions
- **Deploy at the Edge**: Run AI inference on Jetson for real-time control
- **Integrate Everything**: Build end-to-end autonomous systems from voice input to robot motion

---

## Course Structure: 13 Weeks

### Module 0: Foundations & Physical AI (Weeks 1–2)
**Goal**: Understand why robots need both simulation and real-world skills

- What is embodied intelligence?
- Why humanoid robots matter (industry + research)
- Available hardware and learning paths
- Self-assessment: Are you ready?

**Capstone Connection**: Establishes why your capstone project matters

---

### Module 1: ROS 2 Fundamentals (Weeks 3–5)
**Goal**: Master Robot Operating System 2—the robotic nervous system

- **Topic**: ROS 2 architecture, DDS middleware, pub/sub patterns
- **Services**: Request/response communication between nodes
- **Actions**: Long-running asynchronous tasks (robot movements, navigation)
- **Python**: Write your first ROS 2 node in Python
- **Launch Files**: Orchestrate multi-node systems
- **Lab 1.1**: Your first ROS 2 publisher and subscriber
- **Lab 1.2**: Implement a service (request/response)
- **Lab 1.3**: Create and launch a complete ROS 2 package

**Capstone Connection**: ROS 2 is the communication backbone for all robot commands

---

### Module 2: Simulation with Gazebo & Unity (Weeks 6–7)
**Goal**: Validate robot behavior in a physics-accurate virtual world

- **Gazebo**: Physics engine, sensor simulation, multi-body dynamics
- **URDF**: Robot model format (links, joints, sensors, inertia)
- **Sensors**: Simulate cameras, LiDAR, IMU, force sensors
- **Unity**: Alternative simulation engine for high-fidelity rendering
- **Sim-to-Real**: Bridging the gap between simulation and reality
- **Lab 2.1**: Load a humanoid robot in Gazebo
- **Lab 2.2**: Publish simulated sensor data to ROS 2 topics
- **Lab 2.3**: Send ROS 2 commands to move the robot in simulation

**Capstone Connection**: Your capstone first runs in simulation; helps validate algorithms before physical deployment

---

### Module 3: NVIDIA Isaac Platform & Autonomous Perception (Weeks 8–10)
**Goal**: Enable real-time perception and autonomous navigation

- **Isaac Sim**: Photorealistic simulation with synthetic data generation
- **SLAM**: Simultaneous localization and mapping—how robots understand where they are
- **Autonomous Navigation**: Path planning and obstacle avoidance
- **Isaac ROS**: Hardware-accelerated perception on Jetson GPUs
- **Object Detection**: YOLO and deep learning for vision-based understanding
- **Lab 3.1**: Create a photorealistic robot environment in Isaac Sim
- **Lab 3.2**: Implement a SLAM pipeline for autonomous localization
- **Lab 3.3**: Navigate a robot to a target while avoiding obstacles

**Capstone Connection**: Isaac perception enables your robot to understand and navigate its world

---

### Module 4: Vision-Language-Action (VLA) Systems (Weeks 11–13)
**Goal**: Connect natural language understanding to robot control

- **VLA Architecture**: Combining vision, language, and action
- **Language Models**: Prompt engineering for robotics (GPT, Claude, Llama)
- **Voice Interface**: Whisper ASR + text-to-speech
- **Sensor Feedback Loops**: Reactive control adapting to real-time perception
- **System Integration**: End-to-end pipeline from voice → perception → action
- **Edge Deployment**: Running VLA on Jetson for real-time performance
- **Lab 4.1**: Map natural language commands to robot actions
- **Lab 4.2**: Add voice input and receive voice feedback
- **Lab 4.3**: Capstone project—integrate all modules into a working system

**Capstone Connection**: Your capstone brings all four modules together

---

## Your Learning Path

Choose your adventure:

### Path 1: Simulator-Only (Recommended for Learning)
- ✅ No hardware required (all learning on laptop/desktop)
- ✅ Faster iteration (no waiting for physical robot)
- ✅ Complete all 13 weeks + capstone
- ✅ Low cost (~$0 beyond what you have)
- ❌ Can't test on real robot (but that's OK for learning)

**System Requirements**: Ubuntu 22.04 (or macOS/WSL2), 4GB RAM, 20GB disk space

### Path 2: Simulation + Edge Hardware (Advanced)
- ✅ Learn in simulation, validate on real sensors
- ✅ Jetson Orin Nano runs perception pipelines
- ✅ Real camera and microphone input
- ✅ Bridge between simulation and reality
- ⚠ More complex setup; requires hardware ($500 all-in)

**System**: Jetson Orin Nano + RealSense D435i + ReSpeaker

### Path 3: Full Physical Deployment (Research-Grade)
- ✅ Deploy to actual humanoid robot (Unitree G1)
- ✅ Real-world perception and control
- ✅ Publishable research potential
- ❌ Requires institutional access ($50K+ robot)
- ❌ 4–6 weeks post-course for full integration

**Note**: Path 3 is optional and extends beyond the course

---

## Prerequisites

### What You Should Know

- **Python 3.8+**: Basic programming (loops, functions, classes)
- **Linux/Terminal**: Comfortable with command-line
- **Basic Physics**: Newton's laws, vectors, orientation (OK if rusty)

### What You'll Learn (Don't Worry If You Don't Know)

- ✅ ROS 2 and robot middlewares
- ✅ Simulation and physics engines
- ✅ SLAM and autonomous navigation
- ✅ Deep learning for perception
- ✅ LLMs for language understanding
- ✅ Prompt engineering for robotics

**Self-Assessment Quiz**: Coming in Module 0—a quick self-assessment to gauge your readiness

---

## Course Format

### Weekly Structure

- **Reading** (2–3 hours): Textbook chapters with examples
- **Labs** (2–3 hours): Hands-on code exercises
- **Self-Paced**: Work at your own pace (best for this course)
- **Office Hours** (optional): Instructor Q&A sessions

### Labs & Hands-On Work

Every module includes **practical labs**:
- Step-by-step instructions
- Expected output for verification
- Troubleshooting guides
- Extension challenges for advanced learners

### Capstone Project

**Week 13**: Integrate all four modules into a working voice-controlled robot
- Code repository (GitHub)
- Demonstration video (30–60 seconds)
- Technical report (5–10 pages)

---

## Hardware Options

### Minimum (Simulator-Only)

| Component | Cost |
| --- | --- |
| Computer (laptop/desktop) | $0 (bring your own) |
| Ubuntu 22.04 (free) | $0 |
| ROS 2 (free) | $0 |
| **Total** | **$0** |

### Recommended (Simulation + Edge AI)

| Component | Cost |
| --- | --- |
| Jetson Orin Nano + power | $235 |
| RealSense D435i camera | $165 |
| ReSpeaker Mic Array | $70 |
| Storage + cooling | $65 |
| **Total** | **~$535** |

### Optional (Physical Humanoid)

| Component | Cost |
| --- | --- |
| Unitree G1 robot | $35,000–50,000 |
| **Note**: Institutional purchase; not per-student cost |

See [Hardware Setup](./hardware-setup/01-minimum-requirements.md) for detailed options

---

## Grading & Assessment

### Continuous Learning

- **Module Quizzes**: 5 questions per module (open-book)
- **Labs**: Completion-based (you must finish and verify output)
- **Participation**: Discussion forum + office hours (optional but encouraged)

### Capstone Project (40% of Grade)

- **Code Repository**: 30% (clean, tested, documented)
- **Demonstration Video**: 30% (robot responds to 3+ commands)
- **Technical Report**: 20% (system design + lessons learned)
- **Creativity/Enhancements**: 20% (advanced features beyond MVP)

**Minimum Grade to Pass**: 60% overall (2.5/5 on capstone rubric)

---

## Time Commitment

| Week | Hours | Breakdown |
| --- | --- | --- |
| Weeks 1–12 (regular) | 6–8 | 3 hrs reading + 3–5 hrs labs |
| Week 13 (capstone) | 10–12 | 4 hrs integration + 6–8 hrs video/report |
| Total | ~80–100 | 13 weeks of learning |

**Self-Paced Alternative**: Compress or extend over 4–6 months if needed

---

## Software You'll Use

| Tool | Purpose | Cost |
| --- | --- | --- |
| **ROS 2 Humble** | Robot middleware | Free |
| **Gazebo** | Physics simulation | Free |
| **NVIDIA Isaac Sim** | High-fidelity simulation | Free (community) |
| **Jetson SDK** | Edge AI platform | Free |
| **Python 3.10+** | Programming language | Free |
| **OpenAI API** | LLM access (optional) | ~$5–20/month for capstone |
| **GitHub** | Code repository | Free |

**Total Software Cost**: ~$0 (or $5–20 if using commercial LLMs for capstone)

---

## What Makes This Course Unique

✅ **Integration**: Not just ROS 2, simulation, or AI—but how they work together
✅ **Hands-On**: Every concept backed by working code and labs
✅ **Modern Stack**: ROS 2 Humble, Isaac Sim, Jetson, LLMs
✅ **Industry-Relevant**: Skills used in robotics companies worldwide
✅ **Accessible**: Simulator-first means everyone can complete it
✅ **Open-Ended Capstone**: Your ideas + our structure = unique project

---

## Getting Started

### This Week

1. ✅ Choose your learning path (simulator-only vs. hardware)
2. ✅ Set up your system (see [Minimum Requirements](./hardware-setup/01-minimum-requirements.md))
3. ✅ Take the self-assessment quiz in Module 0
4. ✅ Complete your first ROS 2 example (Module 1, Lab 1.1)

### Next Steps

- **Module 0**: Introduction to Physical AI & Humanoid Robotics (Weeks 1–2)
- **Module 1**: ROS 2 Fundamentals (Weeks 3–5)
- And so on...

---

## Support & Resources

- **Textbook**: You're reading it! Full chapters in left sidebar
- **Code Examples**: GitHub repos linked in each module
- **Glossary**: Technical terms defined [here](./glossary.md)
- **Office Hours**: [Posted schedule]
- **Discussion Forum**: Ask questions, share progress
- **Safety Guidelines**: [Before using any hardware, read this](./hardware-setup/04-safety-protocols.md)

---

## Instructor Information

| Role | Name | Email |
| --- | --- | --- |
| Course Lead | [Instructor Name] | [instructor@university.edu] |
| Lab TA | [TA Name] | [ta@university.edu] |

**Office Hours**: [Days/Times]
**Discussion Forum**: [Link]
**Course Updates**: Announced weekly

---

## Frequently Asked Questions

**Q: I don't have a robot. Can I still complete the course?**
A: Yes! Simulation is the primary path. Physical hardware is optional for validation.

**Q: How much programming experience do I need?**
A: Python basics (loops, functions, classes). We teach ROS 2-specific patterns.

**Q: Can I use a different robot platform?**
A: Absolutely. The course is built on ROS 2 (hardware-agnostic). Adapt examples to your robot.

**Q: What if I fall behind?**
A: This is self-paced. Work through modules at your own speed. Capstone is due 1 week after course end.

**Q: Can I get college credit?**
A: [Depends on institution—check with your advisor]

---

## Let's Get Started

**Ready to build your first robot?** Choose your learning path above and begin exploring the modules. Start with your setup and prerequisites, then proceed through each module sequentially.

---

**Course Created**: December 2025
**Last Updated**: 2025-12-10
**Next Cohort Starts**: [Date]

