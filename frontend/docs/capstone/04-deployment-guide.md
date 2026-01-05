---
id: capstone-deployment-guide
title: "Capstone Deployment Guide"
sidebar_position: 4
sidebar_label: "Deployment Guide"
description: "Step-by-step instructions for capstone project submission, GitHub repository structure, video format, and report template"
keywords: [capstone, deployment, submission, GitHub, video, report, template]
---

# Capstone Deployment Guide

This guide walks you through submitting your capstone project. Follow these steps to ensure your work meets all requirements and is ready for evaluation.

---

## Part 1: GitHub Repository Setup

### Step 1: Create a Public Repository

1. Go to [GitHub.com](https://github.com)
2. Create a **new public repository** named:
   ```
   capstone-voice-controlled-robot
   # Or a more specific name:
   # capstone-[your-name]-[path]
   # Example: capstone-alice-jetson-robot
   ```
3. Initialize with:
   - [ ] README.md
   - [ ] .gitignore
   - [ ] License (optional, but MIT is standard)

### Step 2: Structure Your Repository

Follow this exact structure:

```
capstone-voice-controlled-robot/
│
├── README.md                          # Project overview (see template below)
├── SUBMISSION.md                      # Submission checklist
├── requirements.txt                   # Python dependencies
├── .gitignore                         # Exclude __pycache__, .env, etc.
│
├── docs/
│   ├── DESIGN.md                      # Design document
│   ├── ARCHITECTURE.md                # System architecture with diagrams
│   ├── TESTING.md                     # Test results and metrics
│   └── LESSONS_LEARNED.md             # Reflections on the project
│
├── launch/
│   ├── capstone.launch.xml            # Main launch file
│   ├── sim.launch.xml                 # Simulation-only (if applicable)
│   └── hardware.launch.xml            # Hardware-specific (if applicable)
│
├── src/
│   ├── voice_input_node.py
│   ├── planner_node.py
│   ├── robot_controller_node.py
│   ├── perception_node.py
│   ├── safety_manager_node.py
│   ├── vla_pipeline.py
│   └── utils.py
│
├── config/
│   ├── params.yaml                    # All configurable parameters
│   └── safety_limits.yaml
│
├── urdf/
│   └── robot.urdf                     # Robot description (if using custom URDF)
│
├── gazebo_worlds/
│   └── capstone_world.sdf             # Simulation world (if using Gazebo)
│
├── tests/
│   ├── test_voice_input.py
│   ├── test_planner.py
│   ├── test_integration.py
│   └── test_results.txt               # Results from running tests
│
├── video/
│   ├── capstone_demo.mp4              # Main demo video
│   └── README.md                      # Video notes and timestamps
│
├── report/
│   ├── CAPSTONE_REPORT.md             # Full technical report (or PDF)
│   ├── RESULTS.csv                    # Test data/metrics
│   └── figures/                       # Diagrams and screenshots
│       ├── system_architecture.png
│       ├── ros2_graph.png
│       └── test_results.png
│
└── reference/
    ├── tutorials_used.md              # Links to tutorials you referenced
    └── external_resources.md          # Papers, documentation you consulted
```

### Step 3: Create README.md

Use this template:

```markdown
# Voice-Controlled Humanoid Robot Capstone Project

## Project Overview

**Objective**: Build an end-to-end voice-controlled robot system integrating ROS 2, simulation/hardware, perception, and AI planning.

**Success Criteria**:
- ✅ Robot responds to 3+ natural language voice commands
- ✅ Demonstrates visual/audio feedback
- ✅ Executes multi-step sequences (if applicable)
- ✅ Handles edge cases gracefully

## Hardware Path

- [ ] Simulation-Only (Gazebo)
- [ ] Jetson Edge Hardware (RealSense + Jetson Orin Nano)
- [ ] Physical Robot (Unitree / Boston Dynamics / other)

## Quick Start

### Prerequisites
- Ubuntu 22.04 or WSL 2
- ROS 2 Humble: [Install Guide](https://docs.ros.org/en/humble/Installation.html)
- Python 3.10+
- [Optionalfor Simulation] Gazebo or Isaac Sim
- [Optional for Hardware] Jetson Orin Nano with JetPack 5.x

### Installation

```bash
# Clone repository
git clone https://github.com/[your-username]/capstone-voice-controlled-robot.git
cd capstone-voice-controlled-robot

# Install dependencies
pip install -r requirements.txt

# [Optional] Install ROS 2 dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Running the System

```bash
# Terminal 1: Launch ROS 2 system
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch capstone capstone.launch.xml

# Terminal 2: Speak commands
# Microphone will listen for voice input
# Examples:
# - "Walk forward"
# - "Turn left"
# - "Stop"
```

### Expected Output
```
[planner_node]: Planning action for "Walk forward"
[robot_controller]: Publishing cmd_vel: linear.x=1.0
[perception_node]: Camera feed active, detecting objects
[safety_manager]: System ready for commands
```

### Testing

```bash
# Run integration tests
python -m pytest tests/test_integration.py -v

# Run individual node tests
python -m pytest tests/test_voice_input.py
python -m pytest tests/test_planner.py
```

## Project Structure

- `src/`: ROS 2 node implementations
- `launch/`: ROS 2 launch files
- `config/`: Configuration parameters
- `tests/`: Integration and unit tests
- `video/`: Demo video (see below)
- `report/`: Technical report

## Key Modules

### Voice Input (`voice_input_node.py`)
- Captures microphone input
- Transcribes using Whisper ASR
- Publishes to `/voice_input` topic

### Planning (`planner_node.py`)
- Subscribes to `/voice_input`
- Sends prompt to LLM (ChatGPT/Claude/local model)
- Parses action and publishes to `/robot_action`

### Robot Controller (`robot_controller_node.py`)
- Subscribes to `/robot_action`
- Converts action to ROS 2 commands (`/cmd_vel`, `/joint_commands`)
- Publishes to robot actuators

### Perception (`perception_node.py`)
- Camera feed processing
- Object detection
- Publishes detected objects to `/detections`

### Safety Manager (`safety_manager_node.py`)
- Monitors joint limits, velocities
- Emergency stop functionality
- Publishes safety status

## Testing Results

See `tests/test_results.txt` for detailed metrics:
- Command success rate: X%
- Average latency: Y seconds
- Test coverage: Z%

## Demo Video

**Location**: `video/capstone_demo.mp4` (5–8 minutes)

**Contents**:
- System overview and architecture
- 3+ working voice commands
- Integration of all modules
- Edge case handling
- Narration explaining each step

**Timestamps**:
- [0:00] Intro and system overview
- [0:45] Live demo of commands
- [5:00] Technical deep-dive (optional)

## Technical Report

Full technical report in `report/CAPSTONE_REPORT.md`

Includes:
- Problem statement and design
- Architecture diagrams
- Implementation details
- Test results and analysis
- Challenges and solutions
- Lessons learned

## Modules Integrated

- ✅ **Module 1 (ROS 2)**: Node communication, pub/sub, launch files
- ✅ **Module 2 (Simulation/Hardware)**: [Gazebo/Jetson/Robot platform]
- ✅ **Module 3 (Perception)**: Object detection, sensor fusion
- ✅ **Module 4 (VLA)**: Voice input, LLM planning, action execution

## Known Limitations

- LLM latency: ~1.5s (can be improved with model caching)
- Voice recognition works best in quiet environments
- [Hardware-specific limitations if applicable]

## Future Improvements

- [ ] Add multi-modal feedback (LED indicators, haptic feedback)
- [ ] Implement adaptive gait planning
- [ ] Deploy on physical hardware
- [ ] Fine-tune LLM prompts for task-specific commands
- [ ] Add persistent state tracking

## References & Credits

- ROS 2 Documentation: https://docs.ros.org/
- Gazebo Tutorials: https://gazebosim.org/
- [Example project]: https://github.com/[reference-project]
- [Papers/research you referenced]

## Author

[Your Name]
[Date]
[Course: Physical AI & Humanoid Robotics, Week 13]

## License

[MIT License / Your Chosen License]

---

## Submission Checklist

- [ ] All code in `src/` is clean and documented
- [ ] Launch files work without errors
- [ ] Tests pass: `pytest tests/`
- [ ] README updated with your specifics
- [ ] Video uploaded to `video/` folder or as GitHub release
- [ ] Technical report in `report/CAPSTONE_REPORT.md`
- [ ] Repository is public
- [ ] All dependencies in `requirements.txt`
- [ ] .gitignore excludes large files and secrets
```

---

## Part 2: Create Your Technical Report

### Report Structure (Use Markdown or PDF)

**Filename**: `report/CAPSTONE_REPORT.md` or `report/CAPSTONE_REPORT.pdf`

**Use this outline**:

```markdown
# Capstone Project: Voice-Controlled Humanoid Robot

## 1. Introduction (1 page)

### Motivation
Why did you build this? Real-world applications?

### Problem Statement
"Build an integrated robot system that..."

### Objectives
What will you demonstrate?

## 2. System Design (2 pages)

### Architecture Overview
Include ASCII diagram or PNG image:
```
    Microphone
        ↓
    Voice Input Node (Whisper)
        ↓
    Planner Node (LLM)
        ↓
    Robot Controller Node
        ↓
    Robot Actuators
        ↓
    Perception (Feedback)
```

### Component Descriptions
- Voice input: Whisper API or local model
- Planning: GPT-4, Claude, or quantized LLM
- Control: ROS 2 pub/sub pattern
- Perception: Camera + object detection

### Design Choices & Tradeoffs
- Why Whisper instead of [alternative]?
- Why this LLM model?
- Why ROS 2 instead of custom framework?

## 3. Implementation (2–3 pages)

### Module Integration
- **Module 1 (ROS 2)**: Node graph, topics, services
- **Module 2 (Simulation/Hardware)**: How configured?
- **Module 3 (Perception)**: Detection pipeline details
- **Module 4 (VLA)**: Voice→LLM→action flow

### Key Implementation Details
```python
# Example: Voice command processing
command = "Walk forward 2 meters"
action = llm_planner.plan(command)
# Output: {"action": "walk", "distance": 2.0}
robot_controller.execute(action)
```

## 4. Testing & Results (2 pages)

### Test Plan
- Command 1: [Description] → Expected [X] → Actual [X] ✅
- Command 2: [Description] → Expected [X] → Actual [X] ✅
- Edge case: [Unrecognized command] → Handles gracefully ✅

### Metrics
- Success rate: X%
- Latency (voice → action): X seconds
- Robot motion accuracy: X%
- Failure modes: [List any failures and why]

### Results Summary
[Table or chart of results]

## 5. Challenges & Solutions (1 page)

### Challenge 1: [Issue]
**Impact**: [Why it matters]
**Solution**: [How you fixed it]
**Result**: [Outcome]

### Challenge 2: [Issue]
...

## 6. Module Integration Summary (1 page)

How each module contributed:
- Module 1 made possible: [X]
- Module 2 enabled: [X]
- Module 3 provided: [X]
- Module 4 completed: [X]

All four modules working together for: Voice→Robot→Action

## 7. Lessons Learned (1 page)

### Technical Insights
- What did you learn about ROS 2?
- What surprised you about robotics?
- What was hardest?

### Design Decisions
- What worked well?
- What would you do differently?

## 8. Conclusion (1 page)

### Summary
Recap what you built in 1–2 sentences.

### Future Work
- Improvement 1: [How and why]
- Improvement 2: [How and why]
- Research direction: [Topic you'd explore]

### Reflection
How has this 13-week course changed your understanding of robotics and AI?

---

## References

- [ROS 2 Docs](https://docs.ros.org/)
- [Whisper Paper](https://arxiv.org/abs/2212.04356)
- [Your Simulator Docs]
- [External papers/resources]
```

---

## Part 3: Create Your Demo Video

### Video Requirements

**Format**: MP4 (H.264 codec)
**Resolution**: 1080p (1920×1080) minimum
**Duration**: 5–8 minutes
**Audio**: Clear narration + system sounds

### Video Structure

```
[0:00–0:15] Title & Intro
  "Capstone Project: Voice-Controlled Robot"
  Show yourself or system overview

[0:15–0:45] System Architecture
  Diagram or screenshot of ROS 2 node graph
  Brief explanation (30 seconds)

[0:45–2:15] Live Demo: Command 1
  Voice input: "Robot, walk forward"
  Show: Transcription, LLM planning, robot movement
  Explain: What's happening at each step

[2:15–3:45] Live Demo: Command 2
  Another command with clear execution

[3:45–5:15] Live Demo: Command 3
  Third command (or additional variant)

[5:15–6:15] Edge Case or Advanced Feature
  Unrecognized command + graceful handling
  OR multi-step command sequence
  OR sensor feedback integration

[6:15–7:00] Technical Deep-Dive (30 seconds)
  Code walkthrough OR performance metrics

[7:00–7:30] Closing
  Recap of achievements
  Modules integrated
  Future directions

[7:30–8:00] Credits
  Tools used, references, team members
```

### Narration Script Template

```
[0:00] "Hi, I'm [Name]. This is my capstone project: a voice-controlled robot integrating all four modules of the Physical AI course."

[0:30] "The system architecture has four main components: voice input processing using Whisper, AI planning using a language model, ROS 2 communication, and finally robot execution."

[0:50] "Let me show you the system in action. Here's the first command..."

[1:00] *Speak into microphone*
"Robot, walk forward one meter"

[1:05] "The voice is captured and transcribed to text: 'Walk forward one meter'. The LLM planner converts this to a robot action: move forward 1 meter. Finally, the robot controller executes this command using ROS 2."

[1:30] *Robot moves forward* "Success! The robot walked forward."

[2:00] "Command 2: Turn left 90 degrees."

...continue for remaining commands...

[6:00] "Let me quickly show the code. Here's the main planner node that integrates voice to action..."

[6:30] "The entire system achieves an average latency of 1.5 seconds from voice to robot action, with a 90% success rate on recognized commands."

[7:00] "In conclusion, this project demonstrates the integration of ROS 2 middleware (Module 1), simulation and hardware control (Module 2), perception pipelines (Module 3), and vision-language-action systems (Module 4)."

[7:30] "Thank you for watching. The code is available on GitHub at [link]."
```

### Recording Tips

- **Audio**: Use good microphone; minimize background noise
- **Lighting**: Ensure screen/robot clearly visible
- **Screen recording**: 1920×1080 at 30 FPS minimum
- **Narration**: Speak clearly; avoid filler words ("um", "uh")
- **Pacing**: Not too fast (viewers need time to understand); not too slow

### Editing Software (Free Options)

- **OpenShot**: Free, Linux-friendly
- **OBS Studio**: Free, powerful
- **DaVinci Resolve**: Free version available
- **iMovie** (Mac) / **Windows Photos** (Windows)

### Upload Location

1. **Upload to YouTube** (unlisted or public)
2. **OR add to GitHub Releases**:
   ```bash
   git tag -a v1.0 -m "Capstone submission"
   git push origin v1.0
   # Then upload video as release asset
   ```
3. **OR include in `video/` folder** (if smaller than 100 MB)

---

## Part 4: Final Submission Checklist

Before submitting, verify:

### Code
- [ ] All code in `src/` is clean and well-documented
- [ ] Launch files (`.launch.xml`) exist and work
- [ ] `requirements.txt` includes all dependencies
- [ ] `.gitignore` excludes `__pycache__`, `.env`, logs
- [ ] No hardcoded secrets (API keys, passwords)
- [ ] Tests pass: `pytest tests/ -v`

### Documentation
- [ ] `README.md` is complete and clear
- [ ] `docs/ARCHITECTURE.md` explains system design
- [ ] `docs/TESTING.md` shows test results
- [ ] `report/CAPSTONE_REPORT.md` is 8–12 pages

### Video
- [ ] `video/capstone_demo.mp4` exists (5–8 minutes)
- [ ] Covers 3+ working commands
- [ ] Clear narration
- [ ] Includes edge case handling

### Repository
- [ ] Public on GitHub
- [ ] All files committed and pushed
- [ ] No large binary files (>10 MB) except video
- [ ] Proper .gitignore in place

### Submission Details
- [ ] Create `SUBMISSION.md` in repo root:

```markdown
# Capstone Project Submission

## Project Information
- **Student**: [Your Name]
- **Course**: Physical AI & Humanoid Robotics (Week 13)
- **Hardware Path**: [Simulation / Jetson / Physical Robot]
- **Repository**: [GitHub URL]
- **Video**: [YouTube link or GitHub release]

## Submission Date
[Date]

## Checklist
- [x] Code complete and tested
- [x] README.md updated
- [x] Technical report written
- [x] Demo video created
- [x] Repository public
- [x] All links working

## Quick Links
- Main Demo: [video/capstone_demo.mp4](video/capstone_demo.mp4)
- Full Report: [report/CAPSTONE_REPORT.md](report/CAPSTONE_REPORT.md)
- Architecture: [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
```

---

## Part 5: Submission Instructions

### For Online Course (GitHub-Based)

1. **Share your repository link**: Send GitHub URL to instructors
2. **Include in submission**:
   - GitHub repository URL
   - Video link (YouTube or GitHub release)
   - Brief summary of your capstone (1–2 paragraphs)

3. **Email template**:
   ```
   Subject: Capstone Project Submission - [Your Name]

   Dear Instructor,

   I'm submitting my Physical AI capstone project:

   Repository: https://github.com/[username]/capstone-voice-controlled-robot
   Video: https://youtu.be/[video-id] (or GitHub release)
   Hardware Path: [Simulation / Jetson / Physical]

   Project Summary:
   [2–3 sentences about what you built]

   Key Features:
   - ✅ [Feature 1]
   - ✅ [Feature 2]
   - ✅ [Feature 3]

   Thank you,
   [Your Name]
   ```

### For In-Person Course (Lab-Based)

1. **Demo appointment**: Schedule 15-minute presentation
2. **What to show**:
   - Live system running (simulation or hardware)
   - Video demo playing
   - Code walkthrough (5 minutes)
   - Q&A (5 minutes)

3. **Bring**:
   - Laptop with code ready
   - USB with backup code (if hardware needed)
   - Printed report (optional)

---

## Grading Timeline

| Date | Milestone |
|------|-----------|
| **Week 13 (Mon)** | Final coding push |
| **Week 13 (Wed)** | Video & report due |
| **Week 13 (Fri)** | All submissions collected |
| **Week 14–15** | Grading completed |
| **Week 16** | Grades released |

---

## Common Submission Mistakes to Avoid

❌ **Don't**: Leave code uncommented or disorganized
✅ **Do**: Add docstrings and clear variable names

❌ **Don't**: Skip the technical report
✅ **Do**: Write a thorough 8–12 page report

❌ **Don't**: Make a 2-minute or 15-minute video
✅ **Do**: Keep demo to 5–8 minutes (as required)

❌ **Don't**: Include hardcoded API keys or secrets
✅ **Do**: Use `.env` files and .gitignore

❌ **Don't**: Make the repository private
✅ **Do**: Make it public for grading

---

## Questions Before Submitting?

- **Code question**: Check the example projects (Chapter 3)
- **Grading question**: Review rubrics (Chapter 2)
- **Video question**: See video requirements above
- **Report question**: Use the outline template above
- **General**: Post in course forums or attend office hours

---

## You're Ready! 🚀

You've built a voice-controlled robot integrating:
- ✅ ROS 2 communication
- ✅ Simulation/hardware control
- ✅ Perception and autonomy
- ✅ AI-based planning

**Now submit it with confidence.**

The grading rubrics are clear. The examples are detailed. You know exactly what to include.

**Good luck!** 🤖

---

## Final Reminder

This capstone project represents 13 weeks of learning:
- 5 weeks of ROS 2 fundamentals
- 2 weeks of simulation practice
- 3 weeks of advanced perception
- 3 weeks of VLA integration

**Your capstone should demonstrate all of it.**

Commit to excellence. Submit with pride.

---

## Support

Need help?
- **Code issues**: Post in GitHub issues
- **Video problems**: Office hours or email
- **Grading questions**: Contact instructor directly
- **Technical problems**: Help forums are your friend

We're rooting for you! 🎉
