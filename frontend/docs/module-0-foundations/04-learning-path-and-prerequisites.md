---
id: module-0-learning-path
title: "Learning Path & Prerequisites"
sidebar_position: 4
sidebar_label: "Learning Path & Prerequisites"
description: "Self-assessment quiz, prerequisite knowledge check, three hardware learning paths, and time commitment expectations"
keywords: [learning path, prerequisites, self-assessment, hardware setup, time commitment, robot learning]
---

# Learning Path & Prerequisites

## Before You Start: Self-Assessment Quiz

Before diving into ROS 2, simulation, and AI integration, take 5 minutes to honestly assess your readiness. This course assumes baseline technical knowledge.

### Question 1: Python Programming
**Can you write a Python function that takes a list of numbers and returns the sum?**

<details>
<summary>A) Yes, easily</summary>
✅ You're ready. You have Python fundamentals.
</details>

<details>
<summary>B) I can do it with a quick Google search</summary>
✅ Acceptable. You'll pick up Python syntax quickly in this course.
</details>

<details>
<summary>C) I'm not sure / Never written Python</summary>
⚠️ **Recommendation**: Spend 2-3 days reviewing [Python basics](https://www.python.org/about/gettingstarted/) before starting Module 1. Your coding will be clearer if you're comfortable with Python syntax.
</details>

---

### Question 2: Linux/Unix Terminal
**Have you used a terminal (command line) before?**

<details>
<summary>A) Yes, I'm comfortable with bash/zsh</summary>
✅ Excellent. You'll navigate ROS 2 environments easily.
</details>

<details>
<summary>B) I've used it a few times but need reminders</summary>
✅ Acceptable. Module 1 will reinforce terminal skills.
</details>

<details>
<summary>C) No, I mostly use graphical interfaces</summary>
⚠️ **Recommendation**: Spend 1 day learning basic terminal commands: `ls`, `cd`, `mkdir`, `cat`, `echo`, file permissions. The course includes terminal walkthroughs.
</details>

---

### Question 3: Mathematics (Linear Algebra)
**Can you visualize a 3D coordinate system and understand what a transformation matrix does?**

<details>
<summary>A) Yes, I've studied linear algebra or 3D graphics</summary>
✅ You'll excel in modules on robot kinematics and perception.
</details>

<details>
<summary>B) I have a vague idea but haven't studied it formally</summary>
✅ Acceptable. Module 2-3 will teach the math needed.
</details>

<details>
<summary>C) Not at all / Never studied it</summary>
⚠️ **Recommendation**: Glance at [3Blue1Brown's Linear Algebra Essentials](https://www.youtube.com/watch?v=fNk_zzaMoSY&list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) (watch first 2–3 videos). Modules 2–3 will cover what you need in-context.
</details>

---

### Question 4: Robotics Experience
**Have you worked with ROS, robotics APIs, or robot simulators before?**

<details>
<summary>A) Yes, I have ROS or robotics experience</summary>
✅ You'll move through Module 1 quickly and understand capstone integration easily.
</details>

<details>
<summary>B) No, but I'm excited to learn</summary>
✅ Perfect! This course is designed for robotics beginners.
</details>

<details>
<summary>C) I'm not sure what ROS is</summary>
✅ Totally fine. Chapter 0.1–0.3 explains everything. Module 1 starts from scratch.
</details>

---

### Your Assessment Result:

**Mostly A's?** 🎉 You're well-prepared. Jump into Module 1 immediately.

**Mix of A's and B's?** ✅ You're ready. The course teaches everything else.

**Multiple C's?** ⚠️ Spend 3–5 days on prerequisite review before starting Module 1. Links provided above.

---

## Prerequisite Knowledge Checklist

### Required (Have at Least Basic Familiarity)
- [ ] **Python 3.8+**: Variables, functions, loops, classes
- [ ] **Terminal/CLI**: Basic file navigation, running commands
- [ ] **Git** (optional but helpful): Cloning repos, basic commits

### Nice-to-Have (Helpful, Not Required)
- [ ] **Linear algebra**: Vectors, matrices, transformations
- [ ] **Calculus basics**: Derivatives (for robot dynamics in Module 2)
- [ ] **Linux/Ubuntu**: Package managers, file systems

### Will Be Taught In-Course
- [ ] ROS 2 architecture and concepts
- [ ] Gazebo/Isaac Sim usage
- [ ] Robot kinematics and dynamics
- [ ] SLAM and autonomous navigation
- [ ] Vision-Language-Action pipelines
- [ ] LLM integration for robotics

---

## Three Hardware Learning Paths

This course is **hardware-agnostic**. All labs can run in **simulation**. Choose your path based on resources and goals.

### Path 1: 🖥️ Simulation-Only (Free / Ubuntu 22.04+)
**Best for**: Students without hardware access, rapid prototyping, learning fundamentals

**What You Need:**
- Linux PC or WSL 2 (Windows Subsystem for Linux)
- Ubuntu 22.04 or equivalent
- ~10 GB disk space, 4+ GB RAM
- Estimated cost: **$0** (if you have a PC already)

**What You'll Do:**
- Install ROS 2 Humble on Ubuntu
- Use Gazebo for basic simulation
- Run all labs in Gazebo
- Run capstone in Gazebo simulation

**Capstone Output**: Simulation video + code + report (no physical robot)

**Pros:**
- ✅ No hardware cost
- ✅ Fast iteration (no shipping/setup time)
- ✅ Safe (no physical collisions)
- ✅ Fully supported by labs and examples

**Cons:**
- ❌ No real sensor feedback
- ❌ Sim-to-real transfer requires extra validation
- ❌ No physical experience

---

### Path 2: 🛠️ Edge Hardware (Jetson Orin Nano + Sensors, ~$300–500)
**Best for**: Students wanting real hardware experience without robotics platform cost

**What You Need:**
- NVIDIA Jetson Orin Nano developer kit (~$250)
- RealSense D435i camera (~$150)
- ReSpeaker Mic Array (~$100, optional for voice)
- Basic networking setup

**Total Cost**: ~$400–500 (one-time)

**What You'll Do:**
- Simulation + hardware testing on the Jetson
- Run perception algorithms on real GPU
- Capture real camera/sensor data
- Practice sim-to-real transfer learning

**Capstone Output**: Real hardware demo with Jetson + code + report

**Pros:**
- ✅ Real sensor feedback
- ✅ Hardware-accelerated AI (GPU)
- ✅ Sim-to-real practice
- ✅ Affordable ($400 vs $30,000 robot)
- ✅ Reusable for other projects

**Cons:**
- ❌ Limited mobility (no legs; arm optional)
- ❌ Setup complexity
- ❌ Some troubleshooting needed

---

### Path 3: 🤖 Full Physical Deployment (Unitree G1 / Equivalent, ~$30,000+)
**Best for**: Research labs, well-funded teams, full physical embodiment

**What You Need:**
- Humanoid robot platform (Unitree G1, Boston Dynamics Spot, similar)
- Jetson Orin AGX or equivalent (~$3,000–5,000)
- Development environment and safety setup
- Team support (robots require coordination)

**Total Cost**: ~$30,000–60,000+ (significant investment)

**What You'll Do:**
- Everything in simulation + Jetson hardware
- Deploy control algorithms directly to robot
- Iterate on bipedal walking, object manipulation, safety
- Collect real-world data for perception models

**Capstone Output**: Physical robot demo (walk, grasp, respond to voice) + code + report

**Pros:**
- ✅ Full physical embodiment
- ✅ Real sensors and actuators
- ✅ Maximum learning impact
- ✅ Publishable research outcomes

**Cons:**
- ❌ Expensive
- ❌ Requires institutional support
- ❌ Safety and liability considerations
- ❌ Maintenance overhead

---

## Choosing Your Path: Decision Tree

```
Do you have access to a robot (Unitree, Atlas, etc.)?
├─ YES → Use Path 3 (Full Physical)
└─ NO → Do you have a budget for hardware?
        ├─ YES (~$300–500) → Use Path 2 (Jetson + Sensors)
        └─ NO → Use Path 1 (Simulation-Only)

Note: Path 1 is fully supported and sufficient for a strong capstone.
Upgrade anytime; concepts transfer between paths.
```

---

## Module Roadmap

### Your 13-Week Journey

| Week | Module | Focus | Hardware | Hands-On |
|------|--------|-------|----------|----------|
| 1–2 | **Module 0** | Foundations, concepts, prerequisites | None | Self-assessment, reading |
| 3–5 | **Module 1** | ROS 2 fundamentals, nodes/topics/services | Start here | Talker/listener, services, packages |
| 6–7 | **Module 2** | Gazebo simulation, URDF, physics | Simulation | Load robot, simulate sensors, control |
| 8–10 | **Module 3** | Isaac Sim, SLAM, autonomous navigation | Simulation/Jetson | Photorealistic world, VSLAM, Nav2 |
| 11–13 | **Module 4** | VLA, voice control, LLM integration | All paths | Whisper, LLM prompt engineering, end-to-end |
| 13 | **Capstone** | Integrate all modules, submit project | Your path | Voice-controlled robot (sim or physical) |

---

## Time Commitment Expectations

### Per Week
- **Lectures/Reading**: 1.5–2 hours
- **Labs/Coding**: 3–4 hours
- **Assignments**: 1–2 hours
- **Total**: ~5–7 hours/week

### Modules With Heavy Labs (Expect More Time)
- **Module 1** (ROS 2): Expect 7–9 hours (lots of setup, debugging)
- **Module 3** (Isaac): Expect 8–10 hours (simulation setup, tuning)
- **Module 4** (VLA): Expect 8–10 hours (integrating LLMs, voice)

### Capstone Project (Week 13)
- **Planning**: 2–3 hours
- **Implementation**: 8–12 hours
- **Testing/Video**: 3–5 hours
- **Report**: 2–3 hours

**Total capstone time**: ~15–20 hours (final sprint)

---

## Success Factors

### To Complete This Course Successfully, You Should:
1. **Commit to 5–7 hours/week** for 13 weeks (~70–90 hours total)
2. **Have a working development environment** (Linux PC, Jetson, or WSL 2)
3. **Be comfortable with debugging** (Robotics has errors; you'll fix them)
4. **Engage in the labs**, not just read (Hands-on > passive reading)
5. **Ask questions** in forums/community when stuck (We're here to help!)

### Red Flags (You Might Struggle If):
- ❌ You have Under 2 hours/week to dedicate
- ❌ You avoid debugging or problem-solving
- ❌ You don't have Linux/terminal experience and won't learn it
- ❌ You're looking for a "quick robotics course" (This is comprehensive!)

---

## Support & Resources

### Learning Resources by Topic
| Topic | Resource | Time |
|-------|----------|------|
| Python basics | [Python Official Tutorial](https://docs.python.org/3/tutorial/) | 1–2 days |
| Linux terminal | [Linux Command Cheat Sheet](https://cheatography.com/davechild/cheat-sheets/linux-command-line/) | 1 day |
| Linear algebra | [3Blue1Brown Essentials](https://www.youtube.com/watch?v=fNk_zzaMoSY&list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) | 3–4 hours |
| ROS 2 intro | [ROS 2 Official Docs](https://docs.ros.org/en/humble/) | Covered in Module 1 |
| Gazebo | [Gazebo Tutorials](https://gazebosim.org/docs/garden/) | Covered in Module 2 |

### Course Community
- **Forum/Discussion**: Ask questions, share progress
- **GitHub Issues**: Report bugs, suggest improvements
- **Office Hours**: (If instructor-led) Schedule time with instructors

---

## Next Steps

1. **Complete this self-assessment** (5 minutes)
2. **Fill out the prerequisite checklist** above
3. **Choose your hardware path** (Path 1, 2, or 3)
4. **Commit to the time** (5–7 hours/week)
5. **Set up your development environment** this week:
   - Path 1: Ubuntu 22.04 + WSL 2 setup
   - Path 2: Ubuntu + Jetson order/setup
   - Path 3: Access to existing robotics lab
6. **Start Module 1** next week!

---

## Glossary References

- **ROS 2**: Robot Operating System 2; middleware for robot communication
- **Simulation**: Virtual robot environment (Gazebo, Isaac Sim)
- **SLAM**: Simultaneous Localization and Mapping; robot navigation
- **VLA**: Vision-Language-Action; connecting LLMs to robot sensors/actuators
- **Jetson**: NVIDIA's embedded AI computing platform
- **Humanoid**: Robot with human-like form (head, arms, legs, torso)

**For full definitions**, see [Glossary](../glossary.md)

---

## FAQ

**Q: Can I do this course on macOS?**
A: Mostly yes, but ROS 2 support for native macOS is limited. Use Docker or WSL 2 instead.

**Q: Do I need to buy hardware immediately?**
A: No! Start with Path 1 (simulation). Upgrade to Paths 2–3 anytime.

**Q: What if I get stuck on a lab?**
A: Each lab has a troubleshooting section. Post in forums or office hours.

**Q: Is the capstone graded harshly?**
A: Rubric is in Chapter Capstone.2. Focusing on *integration* and *understanding*, not perfection. 3+ voice commands = success!

**Q: Can I switch paths mid-course?**
A: Absolutely. Your code transfers between simulation and hardware seamlessly (ROS 2 abstraction).

---

## Ready?

You've assessed yourself, chosen your path, and understand the time commitment.

**Next chapter**: [Module 0 Summary](#) — Recap and bridge to Module 1.

**After Module 0**: Start [Module 1: ROS 2 Fundamentals](../module-1-ros2/intro.md) and begin your robotics journey.

🚀 **Let's build!**
