---
id: capstone-requirements
title: "Capstone Project Requirements"
sidebar_position: 1
sidebar_label: "Requirements"
description: "Capstone project definition: voice-controlled humanoid robot integrating ROS 2, simulation, and VLA"
keywords: [capstone, project, requirements, deliverables, specification]
---

## Overview

The **Capstone Project** is the culminating experience of the 13-week course. Students will design, implement, test, and demonstrate an **end-to-end voice-controlled humanoid robot system** integrating all four modules:

- **Module 1 (ROS 2)**: Communication backbone
- **Module 2 (Gazebo/Unity)**: Physics-accurate simulation
- **Module 3 (Isaac)**: Real-time perception and autonomous navigation
- **Module 4 (VLA)**: Natural language understanding and action planning

---

## Project Statement

**Goal**: Build a robot system that responds to natural language voice commands, perceives its environment, and executes appropriate actions autonomously.

**Success Criteria**: A working humanoid robot (simulated or physical) that:
1. ✅ Understands at least 3 natural language commands
2. ✅ Demonstrates appropriate visual/audio feedback
3. ✅ Executes multi-step sequences (e.g., "walk to the blue block and pick it up")
4. ✅ Handles edge cases gracefully (unrecognized commands, obstacles)

---

## Minimum Viable Capstone (MVP)

### What Every Student Must Deliver

#### 1. Code Repository

**GitHub repository** containing:

```
my-capstone-project/
├── README.md                          # Project overview
├── requirements.txt                   # Python dependencies
├── src/
│   ├── vla_system.py                 # Main VLA pipeline
│   ├── robot_controller.py            # Robot control abstraction
│   ├── perception_node.py             # Vision + object detection
│   ├── voice_command_parser.py        # Whisper + LLM integration
│   └── safety_manager.py              # Emergency stop + limits
├── config/
│   ├── robot_params.yaml              # Joint limits, velocities
│   ├── vla_prompts.yaml               # LLM system prompts
│   └── safety_config.yaml             # Safety thresholds
├── launch/
│   └── capstone.launch.py             # Single-command robot startup
├── tests/
│   ├── test_voice_recognition.py
│   ├── test_perception.py
│   └── test_robot_control.py
└── docs/
    ├── SETUP.md                       # How to run the system
    ├── ARCHITECTURE.md                # System design explanation
    └── KNOWN_ISSUES.md                # Limitations and workarounds
```

**Requirements**:
- Code is clean, documented, and version-controlled with Git
- All code is tested (unit tests + integration tests)
- README explains how to run the system
- Instructions work on both Ubuntu 22.04 and macOS
- Total code: 500–2000 lines across modules

#### 2. Demonstration Video

**30–60 second video** showing:

1. **Setup** (5 seconds): "Booting robot system..."
2. **Voice Command 1** (10 seconds): Say command → Robot executes → Success
3. **Voice Command 2** (10 seconds): Different command demonstrating variety
4. **Voice Command 3** (10 seconds): Complex command (multi-step or conditional)
5. **Edge Case** (10 seconds): Handling unrecognized command or obstacle
6. **Summary** (5 seconds): "Capstone complete"

**Technical Requirements**:
- Audio is clear and understandable
- Robot motion is smooth (no jerky movements)
- Commands are spoken naturally (not reading a script word-for-word)
- Video is max 2 GB, MP4 format, subtitle any overlaid text
- Include timestamp showing date recorded

**Example Narrative**:

> *[Operator speaks to robot]*
> "Hey robot, come here please."
> *[Robot walks forward 2 meters, stops]*
>
> "Now turn to your left."
> *[Robot rotates 90° left, stabilizes]*
>
> "Pick up that blue block and put it on the table."
> *[Robot perceives block, picks it up, places on table]*
>
> "Did you complete that task?"
> *[Robot responds]: "Yes, task complete. Blue block is on table."*

#### 3. Technical Report

**5–10 page** engineering document containing:

**Section 1: System Architecture** (1–2 pages)
- Block diagram of system (ROS 2 nodes, topics, services)
- Data flow from voice input → robot output
- Module dependencies (which modules feed into which)

Example: Voice input → Whisper ASR → LLM planning → ROS 2 commands → Joint actuation

**Section 2: Implementation Details** (2–3 pages)
- Voice recognition approach (Whisper model, language, confidence threshold)
- LLM integration (which model, prompt engineering strategy)
- Perception pipeline (object detection, SLAM)
- Control strategy (inverse kinematics, motion planning)
- Safety mechanisms (velocity limits, watchdog timer, emergency stop)

**Section 3: Results & Demonstrations** (1–2 pages)
- Screenshots/logs showing system in action
- Performance metrics (latency end-to-end, voice recognition accuracy, etc.)
- List of working commands with examples
- Limitations encountered and how addressed

**Section 4: Lessons Learned** (1 page)
- What surprised you
- What was harder than expected
- What you'd do differently next time
- Ideas for future improvements

**Section 5: References** (1/2 page)
- Papers, documentation, code repos used
- Credit to team members if collaborative
- Acknowledgments of online resources/tutorials

---

## Exceptional Capstone (Going Beyond MVP)

Students who complete MVP can enhance their project:

### Enhancement 1: Sensor Feedback Loops

Implement **reactive control** where robot perception affects action:

```python
# Example: "Pick up the red block"
1. Robot perceives block location via vision
2. LLM plans grasp strategy
3. Robot moves toward block
4. During motion, continuously perceive block position
5. Adjust trajectory if block moves or new obstacles appear
6. Execute grasp when reaching target
```

**Demonstration**: "Move the block and I'll pick it up anyway" → Robot follows block

### Enhancement 2: Multi-Step Reasoning

Enable robot to decompose complex commands:

```
User: "Count the objects on the table and tell me if there are more than 3"

LLM reasoning:
1. Perceive table surface → List of objects
2. Count objects
3. Compare to threshold (3)
4. Generate response: "There are 4 objects"
5. Speak response to user

Robot demonstrates understanding of task structure, not just single commands
```

### Enhancement 3: Physical Robot Deployment

Deploy capstone system to real Jetson + RealSense + ReSpeaker hardware:

**Hardware checklist**:
- [ ] All code runs on Jetson Orin Nano without modification
- [ ] Latency from voice input → robot motion is under 2 seconds
- [ ] System handles real sensor noise (camera artifacts, audio interference)
- [ ] Safety constraints are active (velocity limits, collision avoidance)

**Video shows real robot responding to voice commands**

### Enhancement 4: Conversational Interaction

Enable **multi-turn dialogue** where robot remembers context:

```
User: "Remember I have a red block on the left side of the table"
Robot: "Noted. Red block on table, left side"

User: "Move the red block to the right"
Robot: (recalls context) Moves red block to right side
```

### Enhancement 5: Comparative Analysis

Provide **sim-to-real comparison report**:
- Same code runs in Gazebo vs. Jetson
- Document differences in latency, accuracy, behavior
- Propose calibration factors
- Quantify reality gap

---

## Evaluation Rubric (MVP)

Each component is graded on a **5-point Likert scale**:

| Score | Code | Perception | Integration | Report |
| --- | --- | --- | --- | --- |
| **5 — Exceptional** | Clean, well-tested, excellent documentation | Robust perception handling edge cases | All 4 modules integrated seamlessly | Thorough analysis with insights |
| **4 — Proficient** | Well-organized, mostly tested | Works in normal conditions | 3+ modules working together | Clear explanation with results |
| **3 — Developing** | Functional but some rough edges | Basic perception working | 2–3 modules connected | Adequate documentation |
| **2 — Beginning** | Runs but limited testing | Perception works partially | 1–2 modules integrated | Basic report |
| **1 — Incomplete** | Significant bugs or missing pieces | Perception not working | Minimal integration | Missing or poor report |

**Component Weights**:
- Code implementation: **30%**
- Video demonstration: **30%**
- Technical report: **20%**
- Creativity/enhancements: **20%**

**Minimum Passing Grade**: **2.5/5.0** across all components (overall score ≥60%)

---

## Deliverables Checklist

### Code Repository
- [ ] GitHub repo created and shared with instructor
- [ ] All code in `src/` directory
- [ ] Dependencies listed in `requirements.txt`
- [ ] Launch file runs entire system with one command
- [ ] Unit tests pass: `pytest tests/`
- [ ] README has setup instructions
- [ ] Code is documented (docstrings, comments)

### Demonstration Video
- [ ] Video shows 3+ commands being recognized and executed
- [ ] Audio is clear and understandable
- [ ] Robot motion is visible and smooth
- [ ] Video duration 30–60 seconds
- [ ] Submitted as MP4, max 2 GB
- [ ] Date recorded is visible or documented

### Technical Report
- [ ] 5–10 pages, double-spaced, 12pt font
- [ ] Includes system architecture diagram
- [ ] Explains implementation approach
- [ ] Shows results/demonstrations
- [ ] Discusses limitations
- [ ] Proper citations
- [ ] Submitted as PDF

### Submission Package
- [ ] All three items (code, video, report) in single submission
- [ ] Naming: `YourName-Capstone-2025.zip` containing:
  ```
  my-capstone-project/          (code repo)
  capstone-demonstration.mp4    (video)
  technical-report.pdf          (report)
  SUBMISSION_CHECKLIST.md       (confirming all items)
  ```

---

## Timeline & Milestones

| Week | Milestone | Deliverable |
| --- | --- | --- |
| **Week 11** | Project kickoff | Project proposal (1 page: what you'll build) |
| **Week 12** | Core development | Working voice recognition + basic robot control |
| **Week 13** | Final integration | Full system working; begin video recording |
| **After Week 13** | Final submission | Code, video, and report (due 1 week after course ends) |

---

## Grading Timeline

| Task | Deadline | Grading Time |
| --- | --- | --- |
| Submit code repo link | 1 week post-course | 2–3 days |
| Submit video | 1 week post-course | 1 day (instructor review) |
| Submit report | 1 week post-course | 3–5 days (written feedback) |
| **Final grades posted** | **1 week post-course** | **~10 days total** |

---

## FAQ

**Q: Can I work in teams?**
A: Yes. Groups of 2–3 are encouraged. Each person should contribute a clear component (e.g., one handles voice recognition, another handles perception, third handles control).

**Q: Can I use existing code/libraries?**
A: Yes! Use libraries like `rclpy`, `Isaac ROS`, `OpenAI API`, `Whisper`, etc. Cite your sources. The capstone tests your ability to integrate, not reinvent the wheel.

**Q: What if I can't get voice recognition working perfectly?**
A: MVP requires at least 3 commands. They don't need 100% accuracy; demonstrate fallback behavior for edge cases.

**Q: Can I use a physical robot if I have access?**
A: Absolutely! Physical deployment is an enhancement. Simulation is sufficient for MVP.

**Q: How much code is too much?**
A: 500–2000 lines total is reasonable. 10,000+ lines suggests over-engineering; focus on clarity.

**Q: What if my hardware fails during video recording?**
A: Record once, but include a backup plan. Note any limitations honestly in your report.

---

## Resources

- **ROS 2 Humble Docs**: [https://docs.ros.org/en/humble/](https://docs.ros.org/en/humble/)
- **Isaac Sim Documentation**: [https://docs.omniverse.nvidia.com/isaacsim/](https://docs.omniverse.nvidia.com/isaacsim/)
- **OpenAI Whisper**: [https://github.com/openai/whisper](https://github.com/openai/whisper)
- **OpenAI API**: [https://platform.openai.com/docs/](https://platform.openai.com/docs/)
- **ROS 2 Examples**: [https://github.com/ros2/examples](https://github.com/ros2/examples)

---

## Support

- **Instructor Office Hours**: [Posted schedule]
- **Discussion Forum**: [Course forum link]
- **Code Review**: Submit PR to instructor for feedback (optional but encouraged)

---

**Last Updated**: 2025-12-10
**Relevant For**: Course capstone (Week 13+)
**Course Contribution**: Capstone is 40% of final grade
