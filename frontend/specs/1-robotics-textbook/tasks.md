# Tasks: Physical AI & Humanoid Robotics Textbook

**Feature**: `1-robotics-textbook`
**Specification**: `specs/1-robotics-textbook/spec.md`
**Plan**: `specs/1-robotics-textbook/plan.md`
**Constitution**: `.specify/memory/constitution.md`

**Format**: `[ID] [P?] [Story?] Description with exact file path`

- **[P]**: Parallelizable (different files, no blocking dependencies)
- **[Story]**: Maps to user story (US1, US2, US3, US4 per spec.md)
- **File paths**: All relative to repo root (C:\physical-ai-robotics-textbook\frontend\)

---

## Phase 1: Setup & Infrastructure (Shared)

**Purpose**: Project structure initialization and Docusaurus configuration.

### Docusaurus Configuration

- [x] T001 Initialize Docusaurus project structure in docs/ directory per plan.md Phase 0.1
- [x] T002 [P] Install Docusaurus dependencies (npm install + KaTeX plugins per plan.md)
- [x] T003 Configure docusaurus.config.js with baseUrl, GitHub Pages settings, and KaTeX plugin activation
- [x] T004 [P] Create module directory structure: docs/module-0-foundations/, docs/module-1-ros2/, docs/module-2-simulation/, docs/module-3-isaac/, docs/module-4-vla/
- [x] T005 [P] Create asset directories: docs/assets/diagrams/, docs/assets/screenshots/, docs/assets/code-snippets/
- [x] T006 Configure sidebars.js to auto-generate from sidebar_position frontmatter field per plan.md
- [x] T007 [P] Create code-examples/ directory structure for external runnable examples (if needed)

### Style Guide & Templates

- [x] T008 Create docs/assets/chapter-template.md with mandatory frontmatter (id, title, sidebar_position, sidebar_label, description, keywords) per constitution section VI
- [x] T009 Create docs/assets/STYLE_GUIDE.md documenting Markdown syntax, admonitions (:::note, :::warning, :::tip, :::danger), code blocks, links, images per constitution
- [x] T010 Create docs/assets/lab-template.md with required sections: Prerequisites, Setup, Step-by-Step Tasks, Expected Output, Verification Checklist, Troubleshooting, Extension

---

## Phase 2: Foundational Content (Blocking Prerequisites)

**Purpose**: Core content and infrastructure that MUST complete before any module-specific content.

### Glossary & Terminology

- [x] T011 Create docs/glossary.md with 50+ entries covering: ROS 2 terms (node, topic, service, action, package, message, DDS), Robotics terms (humanoid, kinematics, dynamics, SLAM, URDF, SDF, embodied intelligence), AI terms (vision-language-action, multimodal, synthetic data, sim-to-real)
- [x] T012 Add glossary entries as each chapter introduces new terminology (iterative update task)

### Hardware Setup & Requirements

- [x] T013 Create docs/hardware-setup/01-minimum-requirements.md: Simulator-only path (Ubuntu 22.04 + ROS 2 + Gazebo), system specs, no physical hardware required
- [x] T014 Create docs/hardware-setup/02-recommended-setup.md: Jetson Orin Nano + RealSense D435i + ReSpeaker, cost breakdown, setup instructions per brief.md section "Economy Jetson Student Kit"
- [x] T015 Create docs/hardware-setup/03-optional-unitree-deployment.md: Unitree G1 humanoid reference platform, integration with Jetson, safety protocols, sim-to-real workflow per brief.md
- [x] T016 Create docs/hardware-setup/04-safety-protocols.md: Best practices for deploying to real robots, risk mitigation, validation steps, emergency procedures

### Capstone Project Foundation

- [x] T017 Create docs/capstone/01-requirements.md: Define three deliverables (code + video + report), minimum viable (robot responds to 3+ natural language commands), exceptional criteria (sensor feedback or obstacle avoidance)
- [x] T018 Create docs/capstone/02-grading-rubrics.md: 5-point Likert scale (5=Exceptional, 4=Proficient, 3=Developing, 2=Beginning, 1=Incomplete) for: code implementation (30%), video demonstration (30%), technical report (20%), integration completeness (20%)
- [x] T019 Create docs/capstone/03-example-projects.md: 2-3 reference implementations showing capstone variants (simulation-only, simulation with edge hardware, full physical deployment)
- [x] T020 Create docs/capstone/04-deployment-guide.md: Step-by-step instructions for capstone submission (GitHub repo structure, video format, report template)

### Main Course Overview

- [x] T021 Update docs/intro.md with: Course overview, 13-week learning path, learning outcomes (master ROS 2, simulate robots, develop with Isaac, integrate VLA), weekly breakdown summary, link to module-0 for deeper intro

---

## Phase 3: Module 0 – Foundations & Physical AI (Weeks 1–2) | Priority: P1

**Goal**: Establish conceptual foundation and context for embodied intelligence and humanoid robotics.

**Independent Test**: Student completes Module 0 and understands why Physical AI matters, basic robotics concepts, available hardware options, and can assess their learning readiness.

### Module 0 Content

- [x] T022 [P] [US1] Create docs/module-0-foundations/intro.md: Module overview, learning outcomes, week-by-week preview, capstone relevance
- [x] T023 [P] [US1] Create docs/module-0-foundations/01-what-is-physical-ai.md: Define embodied intelligence, AI in physical world, bridge between digital and physical control, real-world applications (robotics in manufacturing, healthcare, research)
- [x] T024 [P] [US1] Create docs/module-0-foundations/02-why-physical-ai-matters.md: Industry landscape, humanoid robotics trend (Tesla Bot, Boston Dynamics), research significance, why simulation + hardware (sim-to-real transfer) from brief.md "Why Physical AI Matters" section
- [x] T025 [P] [US1] Create docs/module-0-foundations/03-humanoid-robotics-landscape.md: Available platforms (Unitree G1, OP3, Go2), design trade-offs, hardware vs. simulation options, learning path differences per brief.md
- [x] T026 [US1] Create docs/module-0-foundations/04-learning-path-and-prerequisites.md: Self-assessment quiz, prerequisite knowledge check (Python, ML/AI basics), module roadmap, hardware recommendations, time commitment expectations

### Module 0 Summary

- [x] T027 [US1] Create docs/module-0-foundations/summary.md: Recap of core concepts, glossary links, transition to Module 1 (ROS 2)

---

## Phase 4: Module 1 – ROS 2 Fundamentals (Weeks 3–5) | Priority: P1

**Goal**: Master Robot Operating System 2 (ROS 2) as the robotic nervous system for robot control.

**Independent Test**: Student completes Module 1 and can: (1) create a ROS 2 node in Python, (2) implement pub/sub and service patterns, (3) organize code in a ROS 2 package, (4) use launch files and parameters.

### Module 1 Chapters

- [x] T028 [P] [US1] Create docs/module-1-ros2/intro.md: Why ROS 2 is the robotic nervous system, architecture overview, DDS middleware, capstone integration (ROS 2 backbone for all modules)
- [x] T029 [P] [US1] Create docs/module-1-ros2/01-ros2-architecture.md: Core concepts (nodes, topics, services, actions, packages), DDS explanation, graph visualization, comparison with ROS 1, message types (std_msgs, sensor_msgs, geometry_msgs)
- [x] T030 [P] [US1] Create docs/module-1-ros2/02-nodes-topics-services.md: Pub/sub pattern (publishers/subscribers), request/response pattern (services), timing and latency, examples with robot sensor data and motor commands
- [x] T031 [P] [US1] Create docs/module-1-ros2/03-actions-and-timers.md: Asynchronous task execution, action servers/clients, timer callbacks, use cases (robot movements, sensor polling, state management)
- [x] T032 [P] [US1] Create docs/module-1-ros2/04-python-with-rclpy.md: Writing ROS 2 nodes in Python, node lifecycle (initialization, spin, shutdown), callback mechanisms, parameter access, logging best practices
- [x] T033 [US1] Create docs/module-1-ros2/05-launch-files-and-parameters.md: XML launch file syntax, parameter servers, dynamic reconfiguration, package management, dependency resolution, common patterns for multi-node systems

### Module 1 Labs (Hands-On)

- [x] T034 [P] [US1] Create docs/module-1-ros2/lab-1-1-your-first-node.md: Lab objective (create simple talker/listener), prerequisites (ROS 2 Humble installed, colcon build), setup (create package), step-by-step (write node, build, run, verify output), expected output (message on topic), verification (rostopic echo), troubleshooting
- [x] T035 [P] [US1] Create docs/module-1-ros2/lab-1-2-implement-a-service.md: Lab objective (build service server and client), prerequisites, setup, step-by-step (define service message, implement server, client call), expected output (request/response logged), verification checklist, troubleshooting (service not found, timeout)
- [x] T036 [US1] Create docs/module-1-ros2/lab-1-3-create-a-ros2-package.md: Lab objective (organize code as proper ROS 2 package), prerequisites, setup (scaffold with ros2 pkg create), step-by-step (add multiple nodes, create launch file, add dependencies to package.xml), expected output (launch file runs all nodes), verification, troubleshooting

### Module 1 Code Examples & Summary

- [x] T037 [P] [US1] Add inline Python code examples to Module 1 chapters (talker.py, listener.py, service_server.py, service_client.py) with comments and expected output documentation
- [x] T038 [US1] Create docs/module-1-ros2/summary.md: Key takeaways (ROS 2 as communication backbone, pub/sub patterns, package organization), glossary links, capstone preview (how Module 1 enables robot control), transition to Module 2

---

## Phase 5: Module 2 – Simulation with Gazebo & Unity (Weeks 6–7) | Priority: P1

**Goal**: Simulate robots in physics-accurate environments before deploying to real hardware.

**Independent Test**: Student completes Module 2 and can: (1) load a humanoid robot URDF in Gazebo, (2) simulate physics and sensors, (3) control robot from ROS 2 node, (4) compare Gazebo vs. Unity for robotics.

### Module 2 Chapters

- [x] T039 [P] [US1] Create docs/module-2-simulation/intro.md: Role of simulation in robotics, physics engines, sensor simulation, reality gap and mitigation, capstone environment setup
- [x] T040 [P] [US1] Create docs/module-2-simulation/01-gazebo-basics-and-physics.md: Gazebo architecture, physics engine options (ODE, Bullet, DART), scene setup, gravity/friction/collisions, model spawning, launching Gazebo from ROS 2
- [x] T041 [P] [US1] Create docs/module-2-simulation/02-urdf-and-robot-descriptions.md: URDF syntax (links, joints, inertial properties), building humanoid model (torso, limbs, joints, sensors), differences between URDF and SDF, visualization in RViz
- [x] T042 [P] [US1] Create docs/module-2-simulation/03-sensors-in-gazebo.md: Simulating cameras (RGB/depth), LiDAR, IMU, force/torque sensors, publishing sensor data to ROS 2 topics, sensor fusion concepts
- [x] T043 [US1] Create docs/module-2-simulation/04-unity-for-robotics.md: When to use Unity vs. Gazebo, high-fidelity rendering, human-robot interaction visualization, URDF import, ROS 2 integration in Unity
- [x] T044 [US1] Create docs/module-2-simulation/05-sim-to-real-considerations.md: Reality gap (simulation inaccuracy), validation on hardware, transfer learning, safety validation before physical deployment, expected behavior differences

### Module 2 Labs (Hands-On)

- [x] T045 [P] [US1] Create docs/module-2-simulation/lab-2-1-load-robot-in-gazebo.md: Lab objective (load humanoid URDF + physics), prerequisites (Gazebo installed, sample URDF provided), setup (clone example repo or use provided URDF), step-by-step (gazebo launch, verify physics, inspect joints), expected output (robot in world with gravity), verification (apply forces, observe behavior), troubleshooting (missing meshes, physics instability)
- [x] T046 [P] [US1] Create docs/module-2-simulation/lab-2-2-publish-sensor-data.md: Lab objective (simulate sensors and publish data), prerequisites, setup (add camera+lidar to URDF), step-by-step (launch gazebo, verify sensor topics, plot data with rqt), expected output (camera images on /camera/image_raw, lidar on /scan), verification checklist, troubleshooting
- [x] T047 [US1] Create docs/module-2-simulation/lab-2-3-control-robot-in-simulation.md: Lab objective (send ROS 2 commands to move robot), prerequisites (Module 1 complete), setup (create command publisher node), step-by-step (publish joint velocities, observe robot movement), expected output (robot walks or moves in Gazebo), verification, troubleshooting (joint limits, command rates)

### Module 2 Assets & Summary

- [x] T048 [P] [US1] Create sample URDF file in docs/assets/code-snippets/humanoid-example.urdf: Basic humanoid model with legs, arms, head, sensors (for labs)
- [x] T049 [US1] Create docs/module-2-simulation/summary.md: Key takeaways (simulation as validation tool, sensor data integration), glossary links, capstone integration (simulation environment for VLA testing), transition to Module 3

---

## Phase 6: Module 3 – NVIDIA Isaac Platform & Perception (Weeks 8–10) | Priority: P1

**Goal**: Develop advanced perception and autonomous navigation using NVIDIA Isaac Sim and Isaac ROS.

**Independent Test**: Student completes Module 3 and can: (1) set up Isaac Sim environment, (2) implement SLAM pipeline, (3) perform autonomous navigation, (4) detect and manipulate objects, (5) integrate Isaac ROS with ROS 2.

### Module 3 Chapters

- [x] T050 [P] [US1] Create docs/module-3-isaac/intro.md: Role of advanced perception in Physical AI, Isaac Sim advantages (photorealism, synthetic data), Isaac ROS hardware-accelerated pipeline, capstone perception requirements
- [x] T051 [P] [US1] Create docs/module-3-isaac/01-isaac-sim-overview-and-workflows.md: Isaac Sim architecture, Omniverse platform, photorealistic rendering, synthetic data generation for ML, scene composition, asset libraries (Nucleus)
- [x] T052 [P] [US1] Create docs/module-3-isaac/02-building-isaac-environments.md: Scene creation, adding humanoid robot, environment objects, lighting and physics settings, sensor placement, exporting to ROS 2-compatible format
- [x] T053 [P] [US1] Create docs/module-3-isaac/03-slam-and-autonomous-navigation.md: SLAM (Simultaneous Localization and Mapping) concepts, Visual Odometry (VO), loop closure, map building, path planning algorithms (Dijkstra, RRT), Nav2 integration
- [x] T054 [US1] Create docs/module-3-isaac/04-isaac-ros-integration.md: Isaac ROS bridges (image processing, SLAM acceleration), VSLAM (Visual SLAM) hardware acceleration, deployment on Jetson, latency considerations, DNN models (perception)
- [x] T055 [US1] Create docs/module-3-isaac/05-object-detection-and-manipulation.md: Computer vision for object detection (YOLO, detection networks), semantic segmentation, grasping strategies, manipulation planning, integration with navigation

### Module 3 Labs (Hands-On)

- [x] T056 [P] [US1] Create docs/module-3-isaac/lab-3-1-create-isaac-sim-environment.md: Lab objective (set up photorealistic humanoid world), prerequisites (Isaac Sim installed, NVIDIA GPU RTX 4070+), setup (launch Omniverse, import humanoid asset, add sensors), step-by-step (configure physics, add environment objects, verify sensor output), expected output (camera/lidar data flowing), verification checklist, troubleshooting (GPU memory, asset loading)
- [x] T057 [P] [US1] Create docs/module-3-isaac/lab-3-2-implement-slam-pipeline.md: Lab objective (build SLAM system), prerequisites (Module 2 complete, Isaac ROS container), setup (configure VSLAM node), step-by-step (move robot in sim, observe mapping, verify localization), expected output (map generated, odometry accurate), verification (compare with ground truth), troubleshooting
- [x] T058 [US1] Create docs/module-3-isaac/lab-3-3-autonomous-navigation-task.md: Lab objective (navigate robot to target avoiding obstacles), prerequisites (SLAM from Lab 3.2), setup (Nav2 with Isaac environment), step-by-step (set goal, execute navigation, observe path), expected output (robot reaches goal), verification, troubleshooting (path blocked, goal unreachable)

### Module 3 Summary & Capstone Bridge

- [x] T059 [US1] Create docs/module-3-isaac/summary.md: Key takeaways (perception enables autonomy, hardware acceleration matters), glossary links, capstone integration (SLAM + navigation for humanoid to move autonomously), transition to Module 4

---

## Phase 7: Module 4 – Vision-Language-Action (VLA) Systems (Weeks 11–13) | Priority: P1

**Goal**: Integrate natural language understanding and LLM-based planning to control robots with voice commands.

**Independent Test**: Student completes Module 4 and can: (1) parse natural language commands, (2) map commands to robot actions via LLM, (3) implement voice interface with Whisper, (4) close feedback loops with sensor data, (5) deploy full system on edge hardware.

### Module 4 Chapters

- [x] T060 [P] [US1] Create docs/module-4-vla/intro.md: Vision-Language-Action paradigm, multimodal AI for robotics, LLM + robotics integration, conversational interfaces, capstone project overview
- [x] T061 [P] [US1] Create docs/module-4-vla/01-vla-architecture-fundamentals.md: Multimodal inputs (vision, language, sensor), action output space (robot commands), architecture patterns, grounding language to perception, end-to-end learning vs. modular systems
- [x] T062 [P] [US1] Create docs/module-4-vla/02-language-to-action-mapping.md: NLP fundamentals, prompt engineering for robotics, LLM (GPT-4, Claude) for task planning, structured output (JSON actions), reasoning about spatial/temporal constraints
- [x] T063 [P] [US1] Create docs/module-4-vla/03-voice-interface-and-speech-recognition.md: OpenAI Whisper (ASR), voice input processing, real-time transcription on Jetson, audio handling (ReSpeaker arrays), noise robustness, language selection
- [x] T064 [US1] Create docs/module-4-vla/04-sensor-feedback-loops.md: Reactive control (vision-based corrections), closed-loop planning (perception → LLM → action), safety checks (joint limits, collision avoidance), timeout handling and fallbacks
- [x] T065 [US1] Create docs/module-4-vla/05-system-integration-and-deployment.md: End-to-end VLA pipeline on edge (Jetson), latency budgets, resource constraints, multi-modal inference, model quantization, safety validation before physical deployment

### Module 4 Labs (Hands-On)

- [x] T066 [P] [US1] Create docs/module-4-vla/lab-4-1-language-to-action-pipeline.md: Lab objective (parse command → generate robot actions), prerequisites (Module 3 complete, OpenAI API key), setup (LLM client, action parser), step-by-step (define action schema, test prompts, execute commands in simulation), expected output (commands parsed correctly, robot acts), verification, troubleshooting (LLM timeouts, parse errors)
- [x] T067 [P] [US1] Create docs/module-4-vla/lab-4-2-voice-command-processing.md: Lab objective (voice input → action execution), prerequisites (ReSpeaker or mic, Whisper library), setup (audio pipeline), step-by-step (record voice, transcribe, execute action, get feedback), expected output (voice command → robot action), verification, troubleshooting (mic sensitivity, network latency)
- [x] T068 [US1] Create docs/module-4-vla/lab-4-3-capstone-project-autonomous-humanoid.md: Lab objective (integrate all modules into capstone), prerequisites (Modules 1–4 complete), setup (combined ROS 2 + Isaac + VLA system), step-by-step (setup environment, test sub-systems, integrate voice control, validate capstone criteria), expected output (robot responds to 3+ commands), verification (all three deliverables ready), troubleshooting

### Module 4 Summary & Capstone Transition

- [x] T069 [US1] Create docs/module-4-vla/summary.md: Key takeaways (multimodal AI for embodied intelligence, LLM as task planner), glossary links, capstone integration summary (all modules converging), celebration of learning outcomes

---

## Phase 8: Capstone Project (Week 13) | Priority: P1

**Goal**: Students deliver integrated voice-controlled humanoid robot system demonstrating all four modules.

**Independent Test**: Student submits capstone project with: (1) runnable code integrating ROS 2 + Isaac + VLA, (2) video demo (robot responding to voice commands), (3) technical report explaining architecture and design decisions.

### Capstone Lab & Assessment

- [ ] T070 [US1] Capstone deliverable already covered in T019-T020; ensure students reference those files in their projects

---

## Phase 9: Documentation & Assessment Materials | Priority: P2

**Goal**: Provide instructors with grading rubrics, lecture notes, and student assessment tools.

### Assessment Infrastructure

- [ ] T071 [P] [US2] Create docs/assessment/quiz-template.md: Template for chapter quizzes (5 multiple-choice questions per chapter, answers provided)
- [ ] T072 [P] [US2] Create docs/assessment/lab-report-template.md: Template for student lab reports (objective, methods, results, discussion, conclusion)
- [ ] T073 [P] [US2] Create docs/assessment/rubric-detailed.md: Extended grading rubrics with exemplars for each score level (5, 4, 3, 2, 1)

### Quizzes (One per Chapter)

- [ ] T074 [P] [US2] Create docs/module-0-foundations/quiz-module-0.md: 5 questions on concepts from Module 0 chapters
- [ ] T075 [P] [US2] Create docs/module-1-ros2/quiz-module-1.md: 5 questions on ROS 2 architecture, nodes, topics, services, packages
- [ ] T076 [P] [US2] Create docs/module-2-simulation/quiz-module-2.md: 5 questions on Gazebo, URDF, sensors, sim-to-real
- [ ] T077 [P] [US2] Create docs/module-3-isaac/quiz-module-3.md: 5 questions on Isaac Sim, SLAM, navigation, object detection
- [ ] T078 [P] [US2] Create docs/module-4-vla/quiz-module-4.md: 5 questions on VLA architecture, LLM, Whisper, sensor feedback

### Instructor Resources

- [ ] T079 [US2] Create docs/instructor-guide/01-course-structure.md: 13-week syllabus, learning outcomes per week, recommended pacing, prerequisite assessment
- [ ] T080 [US2] Create docs/instructor-guide/02-lab-management.md: Setting up lab environments (simulator vs. hardware), equipment checklists, troubleshooting guide, student access management
- [ ] T081 [US2] Create docs/instructor-guide/03-grading-and-feedback.md: Grading workflow, common mistakes and corrections, providing constructive feedback, benchmark examples

---

## Phase 10: Code Examples & Reference Repository | Priority: P2

**Goal**: Provide students with working, runnable code samples for every major concept.

### Python Code Examples (ROS 2)

- [ ] T082 [P] [US3] Create docs/assets/code-snippets/ros2-talker.py: Publisher node example (outputs incrementing counter)
- [ ] T083 [P] [US3] Create docs/assets/code-snippets/ros2-listener.py: Subscriber node example (listens to published data)
- [ ] T084 [P] [US3] Create docs/assets/code-snippets/ros2-service-server.py: Service server example (responds to requests)
- [ ] T085 [P] [US3] Create docs/assets/code-snippets/ros2-service-client.py: Service client example (makes requests)
- [ ] T086 [P] [US3] Create docs/assets/code-snippets/ros2-action-server.py: Action server example (long-running task)
- [ ] T087 [P] [US3] Create docs/assets/code-snippets/ros2-timer-callback.py: Timer callback example (periodic execution)

### Robot Configuration Files

- [ ] T088 [P] [US3] Create docs/assets/code-snippets/humanoid.urdf: Complete humanoid robot URDF (joints, links, sensors, inertia)
- [ ] T089 [P] [US3] Create docs/assets/code-snippets/gazebo-world.sdf: Gazebo world file (physics, gravity, environment objects)
- [ ] T090 [P] [US3] Create docs/assets/code-snippets/launch-example.xml: Example ROS 2 launch file (multi-node startup)

### Module 3 (Isaac) Code Examples

- [ ] T091 [P] [US3] Create docs/assets/code-snippets/isaac-slam-node.py: SLAM node example (initializes VSLAM, publishes odometry)
- [ ] T092 [P] [US3] Create docs/assets/code-snippets/isaac-nav2-config.yaml: Nav2 configuration example (path planning params)
- [ ] T093 [P] [US3] Create docs/assets/code-snippets/object-detection-example.py: YOLO-based detection on camera feed

### Module 4 (VLA) Code Examples

- [ ] T094 [P] [US3] Create docs/assets/code-snippets/whisper-voice-input.py: Whisper ASR example (records audio, transcribes)
- [ ] T095 [P] [US3] Create docs/assets/code-snippets/llm-command-parser.py: LLM-based command parser (sends prompt to GPT, parses action)
- [ ] T096 [P] [US3] Create docs/assets/code-snippets/sensor-feedback-loop.py: Closed-loop control example (vision → LLM → action → feedback)

### Complete Example Projects (External Repo)

- [ ] T097 [US3] Create code-examples/ directory and README.md with links to GitHub repos containing:
   - ROS 2 example package (Lab 1 code)
   - Gazebo world + humanoid URDF (Lab 2 code)
   - Isaac SLAM + Nav2 example (Lab 3 code)
   - VLA system end-to-end example (Lab 4 code)

---

## Phase 11: Visual Assets & Diagrams | Priority: P2

**Goal**: Create ASCII diagrams and visual explanations for complex concepts.

### Architecture Diagrams

- [ ] T098 [P] [US3] Create docs/assets/diagrams/ros2-graph-architecture.txt: ASCII diagram of ROS 2 node graph (nodes, topics, services, DDS)
- [ ] T099 [P] [US3] Create docs/assets/diagrams/simulation-pipeline.txt: ASCII diagram of Gazebo simulation loop (physics, sensors, ROS 2 integration)
- [ ] T100 [P] [US3] Create docs/assets/diagrams/slam-pipeline.txt: ASCII diagram of SLAM data flow (camera → feature detection → odometry → mapping)
- [ ] T101 [P] [US3] Create docs/assets/diagrams/vla-system-flow.txt: ASCII diagram of VLA pipeline (voice → transcription → LLM → action → robot)
- [ ] T102 [P] [US3] Create docs/assets/diagrams/capstone-architecture.txt: ASCII diagram of complete capstone system (all modules integrated)

### Module-Specific Diagrams

- [ ] T103 [P] [US3] Create docs/assets/diagrams/urdf-structure.txt: ASCII showing URDF link-joint hierarchy
- [ ] T104 [P] [US3] Create docs/assets/diagrams/isaac-sim-workflow.txt: Isaac Sim asset → simulation → ROS 2 bridge
- [ ] T105 [P] [US3] Create docs/assets/diagrams/humanoid-kinematics.txt: ASCII showing humanoid degrees of freedom (bipedal, arms, head)

---

## Phase 12: Quality Assurance & Validation | Priority: P2

**Goal**: Ensure all content meets constitution standards and is ready for publication.

### Content Validation

- [ ] T106 [P] Build Docusaurus site: `npm run build` from docs root (verify no errors/warnings)
- [ ] T107 [P] Link validation: Test all internal links (relative paths) and external links (URLs) resolve correctly
- [ ] T108 [P] Code snippet verification: Test 5 code examples per module on target platform (Ubuntu 22.04) with documented prerequisites and expected output
- [ ] T109 Terminology consistency check: Grep for alternate terms; flag mismatches (e.g., "ROS2" vs "ROS 2", "VLA" vs "Vision-Language-Action"); fix per glossary
- [ ] T110 Sidebar auto-generation: Verify sidebars.js sorts correctly by `sidebar_position`; no duplicate IDs across docs
- [ ] T111 Frontmatter validation: Check all chapter files have required YAML frontmatter (id, title, sidebar_position, sidebar_label, description, keywords)
- [ ] T112 Lab checklist verification: Ensure all labs include Prerequisites, Setup, Step-by-Step Tasks, Expected Output, Verification Checklist, Troubleshooting, Extension sections

### Constitution Compliance Review

- [ ] T113 [P] Modular & Progressive check: Each chapter is self-contained; prerequisites clearly listed
- [ ] T114 [P] Hands-On Code verification: Each chapter has ≥2 examples + ≥1 lab; all code runs successfully
- [ ] T115 [P] Hardware Agnostic review: Code uses ROS 2 abstractions; hardware-specific sections clearly marked
- [ ] T116 Capstone Connection verification: Each chapter explicitly states how it contributes to capstone project
- [ ] T117 Technical Accuracy review: Code tested on target platforms; facts about ROS 2, Gazebo, Isaac verified as current
- [ ] T118 Clarity & Completeness review: No unexplained jargon; labs are end-to-end (not scaffolded incompletely)
- [ ] T119 Accessibility review: Diagrams provided for complex systems; all images have descriptive alt-text

### Final Polish Pass

- [ ] T120 [P] Copy editing: Grammar, spelling, consistent voice across all chapters
- [ ] T121 [P] Visual consistency: Diagrams, code blocks, admonitions formatted uniformly
- [ ] T122 [P] Image optimization: Resize assets for web; verify alt-text present
- [ ] T123 Glossary completeness: Verify all technical terms in chapters are defined or linked to glossary.md
- [ ] T124 Metadata review: Update docusaurus.config.js with correct title ("Physical AI & Humanoid Robotics Textbook"), logo, GitHub repo link, social links
- [ ] T125 README.md creation: Create top-level README explaining textbook purpose, how to build locally, how to deploy

---

## Phase 13: Deployment & Post-Launch | Priority: P2

**Goal**: Publish textbook to GitHub Pages and establish feedback loop.

### Deployment Tasks

- [ ] T126 Deploy to GitHub Pages: `npm run deploy` (or via GitHub Actions CI/CD if configured)
- [ ] T127 Verify live site: Check https://[github-username].github.io/physical-ai-robotics-textbook/ loads correctly (or correct baseUrl per docusaurus.config.js)
- [ ] T128 Mobile responsiveness check: Test site on phone/tablet; verify sidebar, code blocks, images render correctly
- [ ] T129 Search functionality test: Verify Docusaurus search works; test searching for "ROS 2", "SLAM", "VLA"
- [ ] T130 [P] Configure analytics (optional): Add Google Analytics or similar to track page views, user engagement
- [ ] T131 Create GitHub Issues template: Enable student/instructor feedback for content errors, clarifications, code example bugs

### Iteration & Feedback Loop

- [ ] T132 Monitor GitHub Issues: Collect student/instructor bug reports, content feedback
- [ ] T133 Create PHR for each update: Document feedback received and changes made
- [ ] T134 Prioritize fixes: High-impact feedback (broken code, conceptual errors) first
- [ ] T135 Iterate constitution: If structural issues identified, amend constitution and increment version
- [ ] T136 Redeploy: Push fixes and redeploy via `npm run deploy`

---

## Summary

### Task Count

- **Phase 1 (Setup & Infrastructure)**: 7 tasks
- **Phase 2 (Foundational Content)**: 13 tasks
- **Phase 3 (Module 0)**: 6 tasks
- **Phase 4 (Module 1 ROS 2)**: 10 tasks
- **Phase 5 (Module 2 Simulation)**: 10 tasks
- **Phase 6 (Module 3 Isaac)**: 10 tasks
- **Phase 7 (Module 4 VLA)**: 10 tasks
- **Phase 8 (Capstone)**: 1 task
- **Phase 9 (Assessment)**: 10 tasks
- **Phase 10 (Code Examples)**: 16 tasks
- **Phase 11 (Diagrams)**: 8 tasks
- **Phase 12 (QA)**: 20 tasks
- **Phase 13 (Deployment)**: 11 tasks

**Total: 132 tasks** (granular, individually executable)

### Task Distribution by User Story

- **[US1] Self-Paced Learning**: 80 tasks (all chapter content, labs, examples)
- **[US2] Instructor Resources**: 12 tasks (assessments, quizzes, rubrics, instructor guide)
- **[US3] Code Examples**: 21 tasks (code snippets, diagrams, reference implementations)
- **No Story Label** (Setup, QA, Deployment): 19 tasks (infrastructure, validation, deployment)

### Execution Strategy

**MVP First (User Story 1 Only)**:
1. Complete Phase 1 (Setup) → 7 tasks
2. Complete Phase 2 (Foundations) → 13 tasks
3. Complete Phase 3–8 (All module content + capstone) → 48 tasks
4. **STOP and VALIDATE**: All core learning content ready; student can complete 13-week course
5. Build & deploy: `npm run build && npm run deploy`
6. **Cost**: ~59 tasks for MVP (all chapters, labs, core code examples)

**Incremental Enhancement**:
7. Phase 9 (Assessment): +10 tasks → Enable instructor-led delivery
8. Phase 10 (Code Examples): +16 tasks → Complete code reference library
9. Phase 11 (Diagrams): +8 tasks → Visual polish
10. Phase 12 (QA): +20 tasks → Quality assurance and constitution compliance
11. Phase 13 (Deployment): +11 tasks → Live site + feedback loop

### Parallel Opportunities

- **Phase 1**: All [P] tasks (T002, T004, T005) can run in parallel
- **Phase 2**: Module intro tasks (T028, T039, T050, T060) can run in parallel
- **All code examples (Phase 10)**: All [P] tasks (T082–T105) can run in parallel
- **All diagrams (Phase 11)**: All [P] tasks can run in parallel
- **QA checks (Phase 12)**: Most [P] tasks can run in parallel

### Dependencies & Critical Path

- **Setup (Phase 1)** → **Foundations (Phase 2)** → **Module Content (Phases 3–8)** → **QA (Phase 12)** → **Deploy (Phase 13)**

Phases 3–8 can proceed in parallel (each module independent) once Phase 2 complete.

---

## Notes

- All tasks have exact file paths for immediate execution
- [P] tasks are independent; can be assigned to different team members
- Labs include full expected output + troubleshooting guidance
- All code examples are tested on target platforms before publishing
- Constitution compliance verified before any chapter goes live
- Glossary grows iteratively as new terms introduced
- Capstone project reuses code from all four modules (coherent narrative)
