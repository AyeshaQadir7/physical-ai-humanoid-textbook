---
id: capstone-grading-rubrics
title: "Capstone Grading Rubrics"
sidebar_position: 2
sidebar_label: "Grading Rubrics"
description: "Detailed 5-point Likert scale grading rubrics for capstone project evaluation"
keywords: [capstone, grading, rubric, assessment, evaluation, code, video, report]
---

# Capstone Grading Rubrics

## Overview

Your capstone project will be evaluated on **four dimensions** using a **5-point Likert scale**:

| Score | Level | Description |
|-------|-------|-------------|
| **5** | **Exceptional** | Exceeds expectations; innovative; production-quality |
| **4** | **Proficient** | Meets all requirements; clean, well-organized |
| **3** | **Developing** | Meets core requirements; minor gaps or inefficiencies |
| **2** | **Beginning** | Partial implementation; significant gaps |
| **1** | **Incomplete** | Does not meet minimum requirements |

---

## Rubric 1: Code Implementation (30% of final grade)

### Criteria: Functionality, Code Quality, and ROS 2 Integration

| Score | Functionality | Code Quality | ROS 2 Integration | Integration | Total Points |
|-------|---------------|--------------|-------------------|-------------|--------------|
| **5** | ✅ All 4+ commands work flawlessly; edge cases handled gracefully | Excellent: Clean, well-documented, modular, follows Python conventions | Advanced: Proper pub/sub, services, launch files, parameter handling | All modules seamlessly integrated (Modules 1–4 working together) | **6/6** |
| **4** | ✅ 3+ commands work reliably; most edge cases handled | Good: Code is readable, documented, well-organized | Good: Proper ROS 2 patterns, includes launch files, clear node structure | All modules integrated; minor gaps in one area | **5/6** |
| **3** | ✅ 3+ commands work; basic error handling | Fair: Code works but needs cleanup, some documentation gaps | Adequate: ROS 2 patterns present, but missing some best practices | Modules integrated but some disconnect between them | **4/6** |
| **2** | ✅ 1–2 commands work; limited error handling | Poor: Hard to follow, minimal documentation, no structure | Weak: ROS 2 usage present but not well-integrated; missing launch files | Modules partially integrated; significant gaps | **2–3/6** |
| **1** | ❌ Commands don't work or incomplete | Very poor: Unreadable, no documentation, untested | Minimal: ROS 2 barely integrated or incorrect usage | Modules not integrated; code incomplete | **0–1/6** |

### Example Scoring:

**Score 5 Example:**
```
- Voice input → Whisper ASR → Recognized ("walk forward")
- LLM planning → Structured action ({"action": "walk", "distance": 1.0})
- ROS 2 publish → /cmd_vel topic → Robot moves
- Feedback loop → Check LiDAR → Obstacle? Adjust
- Code: Clean, modular, documented, error handling for network timeouts
```

**Score 3 Example:**
```
- Voice input works but sometimes unreliable
- LLM planning works for 3 commands but occasionally fails on edge cases
- ROS 2 integration present but missing parameters/launch files
- Feedback loop basic; doesn't handle all obstacles
- Code works but lacks documentation and organization
```

---

## Rubric 2: Video Demonstration (30% of final grade)

### Criteria: Clarity, Completeness, and System Integration

| Score | Command Execution | Clarity | Integration | Professionalism | Total Points |
|-------|-------------------|---------|-------------|-----------------|--------------|
| **5** | 4+ commands executed cleanly; smooth transitions | Crystal clear narration; easy to follow workflow | All subsystems visible and working (voice → AI → robot action) | High-quality video; good lighting; clear audio; professional editing | **6/6** |
| **4** | 3+ commands executed well; mostly smooth | Clear narration; generally easy to follow | All major subsystems shown; minor gaps in one area | Good video quality; audible narration; basic editing | **5/6** |
| **3** | 3 commands executed; some hesitation | Narration present but could be clearer | Most subsystems shown; some gaps in integration flow | Acceptable video quality; narration audible | **4/6** |
| **2** | 1–2 commands work; noticeable delays/errors | Minimal narration; hard to follow | Some subsystems shown; integration unclear | Poor video quality or hard to hear narration | **2–3/6** |
| **1** | Commands fail or incomplete demo | No narration or unintelligible | Subsystems not demonstrated | Very poor video quality; inaudible | **0–1/6** |

### Video Checklist (Include All):

- [ ] **Voice input**: Microphone captures your voice clearly
- [ ] **Transcription**: Whisper ASR shows recognized text on screen
- [ ] **LLM planning**: Show the LLM prompt/response (e.g., "Planning: move to 1.0m forward")
- [ ] **Robot action**: Robot responds (moves, gestures, or responds vocally)
- [ ] **Sensor feedback**: Show camera feed or LiDAR data if available
- [ ] **Narration**: Explain what's happening at each step
- [ ] **Edge case**: Show how system handles one unrecognized command
- [ ] **Timestamps**: Label each command attempt (Command 1, Command 2, etc.)

### Example Video Structure (5–8 minutes):

```
[0:00] Intro: "This is my capstone project: a voice-controlled humanoid robot"
[0:15] System overview diagram (ROS 2 graph, modules)
[0:45] Command 1: "Robot, walk forward" → Show voice input → LLM → Robot moves
[1:30] Command 2: "Stop" → Robot stops cleanly
[2:15] Command 3: "Turn left 90 degrees" → Robot rotates
[3:00] Command 4 (Optional): Complex command + feedback
[3:45] Edge case: "Unrecognized command" → System handles gracefully
[4:15] Recap: "All modules (ROS 2, simulation/hardware, perception, VLA) working"
[4:30] End credits
```

---

## Rubric 3: Technical Report (20% of final grade)

### Criteria: Design, Architecture, and Analysis

| Score | Problem Statement | Design & Justification | Implementation Details | Testing & Results | Conclusion | Total Points |
|-------|-------------------|----------------------|------------------------|-------------------|-----------|--------------|
| **5** | Clear; well-motivated | Thoughtful design choices explained; tradeoffs discussed | Comprehensive; algorithms explained; code structure clear | Thorough testing; success metrics defined; results analyzed | Strong summary; lessons learned; future work | **5/5** |
| **4** | Clear and well-written | Good design choices explained; some tradeoffs mentioned | Implementation clear; most algorithms documented | Good testing; success metrics shown; results explained | Good summary; lessons learned | **4/5** |
| **3** | Clear problem statement | Design described; some justification provided | Implementation described; some gaps in detail | Basic testing; some success metrics; results presented | Summary provided; some lessons learned | **3/5** |
| **2** | Problem somewhat clear | Design described but limited justification | Implementation sparse details | Minimal testing; unclear success metrics | Minimal conclusion | **2/5** |
| **1** | Unclear or missing | Little design explanation | Very limited details | No testing or unclear results | No conclusion | **0–1/5** |

### Report Outline (8–12 pages):

#### 1. Introduction (1 page)
- **Motivation**: Why voice-controlled robots?
- **Problem Statement**: "Design an integrated robotic system that..."
- **Objectives**: What will you demonstrate?

#### 2. System Design (2 pages)
- **Architecture Diagram**: ROS 2 node graph (hand-drawn ASCII acceptable)
- **Component Breakdown**:
  - Voice input (Whisper)
  - LLM planning (GPT, Claude, local model)
  - Robot control (ROS 2 commands)
  - Perception (camera, LiDAR)
- **Design Choices**: Why these tools? Alternatives considered?

#### 3. Implementation (2–3 pages)
- **Module Integration**:
  - Module 1 (ROS 2): Which patterns used? (pub/sub, services, launch files)
  - Module 2 (Simulation): Gazebo/Isaac setup? URDF used?
  - Module 3 (Isaac/Perception): SLAM, navigation, or object detection?
  - Module 4 (VLA): LLM integration, voice processing pipeline
- **Key Algorithms**:
  - Command parsing logic
  - Robot action generation
  - Error handling

#### 4. Testing & Results (2 pages)
- **Test Cases**:
  - Command 1: [description] → Expected output [X] → Actual output [X] ✅
  - Command 2: [description] → Expected output [X] → Actual output [X] ✅
  - Edge case (unrecognized command): Expected behavior [X] → Actual [X] ✅
- **Metrics**:
  - Success rate (% of commands recognized correctly)
  - Average latency (voice → action in seconds)
  - Robustness (how many edge cases handled)
- **Results Analysis**: What worked well? What was challenging?

#### 5. Challenges & Solutions (1 page)
- **Challenge 1**: [Issue encountered] → [How you solved it]
- **Challenge 2**: [Issue encountered] → [How you solved it]
- **What You Learned**: Insights about robotics, ROS 2, AI integration

#### 6. Conclusion (1 page)
- **Summary**: What you built and what it demonstrates
- **Lessons Learned**: Key takeaways from 13 weeks
- **Future Work**: How would you improve? What's next?

### Writing Standards:
- Clear, technical writing (not casual)
- Diagrams/screenshots where helpful
- Proper citations for external work
- Spell-checked and grammar-reviewed

---

## Rubric 4: Integration Completeness (20% of final grade)

### Criteria: How well all four modules work together

| Score | Module 1 (ROS 2) | Module 2 (Simulation) | Module 3 (Perception) | Module 4 (VLA) | Overall Integration | Total Points |
|-------|------------------|----------------------|----------------------|------------------|-------------------|--------------|
| **5** | ✅ Full ROS 2 ecosystem (nodes, topics, services, launch) | ✅ Robot + physics fully simulated or deployed on Jetson | ✅ SLAM/perception active; robot aware of environment | ✅ Full VLA pipeline; voice→LLM→action working | Seamless; all modules contribute to capstone | **5/5** |
| **4** | ✅ ROS 2 well-integrated; all patterns used | ✅ Simulation/hardware functional; physics reasonable | ✅ Perception present; basic autonomy | ✅ VLA works for 3+ commands | All modules integrated; minor gaps | **4/5** |
| **3** | ✅ ROS 2 basic patterns used; adequate structure | ✅ Simulation/hardware works; some gaps | ✅ Basic perception; limited autonomy | ✅ VLA works but limited commands | Modules present; some disconnect | **3/5** |
| **2** | ✅ ROS 2 present but incomplete | ⚠️ Simulation/hardware partially functional | ⚠️ Minimal perception | ⚠️ VLA limited or unreliable | Modules present but poorly integrated | **2/5** |
| **1** | ⚠️ ROS 2 minimal or incorrect | ❌ No simulation/hardware | ❌ No perception | ❌ No VLA | Modules not integrated | **0–1/5** |

### Integration Checklist:

- [ ] **Module 1 → Module 2**: ROS 2 commands control simulated/physical robot
- [ ] **Module 2 → Module 3**: Sensor data (camera, LiDAR) flows through ROS 2 topics
- [ ] **Module 3 → Module 4**: Perception output feeds LLM planning (e.g., "detected blue cube → LLM: move to object")
- [ ] **Module 4 → Module 1**: VLA output (actions) translated to ROS 2 commands
- [ ] **Feedback Loop**: Robot perceives result → Adjusts behavior if needed
- [ ] **Safety**: Emergency stop, joint limits, collision avoidance

---

## Final Grade Calculation

```
Final Grade = (Code × 0.30) + (Video × 0.30) + (Report × 0.20) + (Integration × 0.20)
```

### Example Calculation:

| Dimension | Score | Weight | Contribution |
|-----------|-------|--------|---------------|
| Code | 4/6 | 30% | 2.0 |
| Video | 5/6 | 30% | 2.5 |
| Report | 4/5 | 20% | 1.6 |
| Integration | 4/5 | 20% | 1.6 |
| **Total** | — | — | **7.7 / 10** |

**Letter Grade Conversion**:
- 9–10 = A
- 8–9 = B+
- 7–8 = B
- 6–7 = C+
- 5–6 = C
- Below 5 = Below expectations

---

## Exemplary Projects (Score 5)

### Example 1: Simulation-Only Path
- **Code**: ROS 2 nodes for perception, VLA, planning; clean architecture; 10+ voice commands
- **Video**: 7-minute demo showing Gazebo robot responding to complex commands with obstacle avoidance
- **Report**: Detailed design, SLAM integration analysis, sim-to-real transfer discussion
- **Integration**: All modules seamlessly connected; feedback loops working

### Example 2: Jetson Hardware Path
- **Code**: ROS 2 + Isaac hardware-accelerated perception; real camera feed processing
- **Video**: 5-minute demo with real Jetson running; voice commands with real sensor feedback
- **Report**: Hardware integration challenges; performance analysis (latency, power)
- **Integration**: Full stack from voice to robot actuation on real hardware

### Example 3: Physical Robot Path (Lab-Based)
- **Code**: ROS 2 full-stack; safety controllers; multi-modal perception
- **Video**: Physical robot responding to 4+ voice commands; walking, grasping, obstacle avoidance
- **Report**: Deep dive into sim-to-real transfer; lessons learned deploying to real hardware
- **Integration**: All modules working on physical platform; real-world constraints addressed

---

## Tips for High Scores

### Code (30%):
- Use ROS 2 launch files and parameters (not hardcoded values)
- Include error handling (network timeouts, unrecognized commands)
- Write docstrings and comments
- Organize code into modules (don't put everything in one file)

### Video (30%):
- Keep it 5–8 minutes (not too long)
- Show system components (node graph, camera feed, LLM response)
- Explain what's happening (narration is key)
- Include at least one edge case (what happens when command fails?)

### Report (20%):
- Show your thinking (design tradeoffs, alternatives considered)
- Include diagrams (even hand-drawn ROS 2 graphs are fine)
- Analyze your results (why did it work? what was hard?)
- Discuss lessons learned

### Integration (20%):
- Ensure all modules contribute (don't "fake" using them)
- Show feedback loops (perception informs action)
- Demonstrate autonomy (robot doesn't just repeat commands)

---

## Common Mistakes to Avoid

❌ **Don't**: Hardcode all robot commands; use ROS 2 abstractions
❌ **Don't**: Make a video with no narration; explain what you're doing
❌ **Don't**: Skip edge case handling; systems fail gracefully
❌ **Don't**: Ignore integration; each module should feed into the next
❌ **Don't**: Write a report that's just code listings; analyze and discuss

✅ **Do**: Show your system architecture
✅ **Do**: Test with multiple commands (prove it's not one hard-coded demo)
✅ **Do**: Explain your design choices and tradeoffs
✅ **Do**: Demonstrate error handling (what happens when things go wrong?)
✅ **Do**: Show all four modules working together

---

## Questions Before You Submit?

- **Code question**: Check ROS 2 best practices, ask in forums
- **Video question**: See example video structure above
- **Report question**: Refer to the outline and exemplary projects
- **Grading question**: Ask instructor for clarification

**Remember**: The capstone is about **integration and demonstration**, not perfection. A well-integrated system with 3–4 working commands and a clear explanation of what you built will score well.

Good luck! 🤖
