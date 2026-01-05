# Implementation Plan: Physical AI & Humanoid Robotics Textbook

**Feature**: 1-robotics-textbook
**Specification**: `specs/1-robotics-textbook/spec.md`
**Constitution**: `.specify/memory/constitution.md`
**Created**: 2025-12-10
**Status**: PLANNING

---

## Executive Summary

This plan outlines the complete execution strategy for generating a 13-week Physical AI & Humanoid Robotics textbook using Docusaurus v2+ and Markdown. The textbook bridges digital AI systems with physical robot control through four progressive modules (ROS 2 → Simulation → Isaac → VLA), culminating in a capstone project.

**Delivery Model**: Self-paced or instructor-led; simulator-first with optional physical hardware.
**Output Format**: Markdown docs → Docusaurus site → GitHub Pages
**Quality Gate**: All content complete, tested, and aligned with constitution before publication.

---

## Phase 0: Preparation & Bootstrap

### Step 0.1: Docusaurus Project Setup

**Objective**: Initialize a Docusaurus v2 site with correct folder structure and configuration.

**Tasks**:

1. Bootstrap Docusaurus project:
   already initiallized
2. Install dependencies:
   ```bash
   npm install
   npm install @docusaurus/plugin-google-analytics  # optional
   npm install katex remark-math rehype-katex        # for math notation
   ```
3. Create directory structure:
   ```
   docs/
   ├── intro.md
   ├── module-0-foundations/
   ├── module-1-ros2/
   ├── module-2-simulation/
   ├── module-3-isaac/
   ├── module-4-vla/
   ├── capstone/
   ├── hardware-setup/
   ├── glossary.md
   └── assets/
       ├── diagrams/
       └── screenshots/
   ```
4. Configure `docusaurus.config.js`:
   - Enable KaTeX for math notation
   - Configure sidebar auto-generation from frontmatter
5. Configure `sidebars.js`:
   - Auto-generate sidebar from `sidebar_position` frontmatter field
   - Organize by module structure

**Output**: Initialized Docusaurus project with folder structure ready for content.

### Step 0.2: Establish Template & Style Guide

**Objective**: Define canonical chapter template and formatting standards.

**Deliverables**:

1. **Chapter Template** (must follow constitution):

   ```markdown
   ---
   id: module-X-chapter-Y
   title: "Chapter Title"
   sidebar_position: N
   sidebar_label: "Short Title"
   description: "1-sentence summary"
   keywords: [keyword1, keyword2, ...]
   ---

   ## Introduction

   [Why this matters; context in course]

   ## Learning Outcomes

   - Outcome 1
   - Outcome 2

   ## Core Concepts

   ### Concept 1

   [Theory + architecture]

   ### Concept 2

   [Theory + examples]

   ## Examples

   ### Example 1

   [Code + explanation]

   ### Example 2

   [Code + explanation]

   ## Hands-On Lab: [Lab Title]

   - Prerequisites
   - Setup
   - Step-by-Step Tasks
   - Expected Output
   - Verification Checklist
   - Troubleshooting
   - Extension Challenge

   ## Summary & Next Steps

   [Key takeaways; bridge to next chapter]

   ## Further Reading

   [Links to docs, papers, resources]

   ## Code Repository

   [Links to example code]
   ```

2. **Style Guide** documenting:

   - Markdown syntax rules (per constitution)
   - Admonition usage (:::note, :::warning, :::tip, :::danger)
   - Code block formatting (language tags required)
   - Link conventions (relative vs. absolute)
   - Image naming and placement
   - Terminology standards (glossary alignment)

3. **Glossary Template** with entries for:
   - ROS 2 terms (node, topic, service, action, package, message)
   - Robotics terms (humanoid, kinematics, dynamics, SLAM, URDF, SDF)
   - AI terms (embodied intelligence, vision-language-action, multimodal)

**Output**: Template files, style guide, and glossary starter in `docs/assets/`.

---

## Phase 1: Content Architecture & Breakdown

### Step 1.1: Define Module-Level Architecture

**Objective**: Create detailed chapter breakdowns for each module, ensuring sequential dependencies and capstone integration.

**Module 0: Foundations (Weeks 1–2)**

| Chapter | Topic                         | Learning Outcomes                           | Capstone Contribution |
| ------- | ----------------------------- | ------------------------------------------- | --------------------- |
| 0.1     | Course Overview & Physical AI | Understand embodied intelligence concept    | Context & motivation  |
| 0.2     | Why Physical AI Matters       | Real-world applications, research landscape | Industry relevance    |
| 0.3     | Humanoid Robotics Landscape   | Available platforms, design trade-offs      | Hardware options      |
| 0.4     | Learning Path & Prerequisites | Self-assessment, module structure           | Study guide           |

**Module 1: ROS 2 Fundamentals (Weeks 3–5)**

| Chapter | Topic                              | Learning Outcomes                              | Capstone Contribution      |
| ------- | ---------------------------------- | ---------------------------------------------- | -------------------------- |
| 1.1     | ROS 2 Architecture Overview        | Understand DDS middleware, node graph          | ROS 2 control framework    |
| 1.2     | Nodes, Topics, Services            | Publish/subscribe, request/response patterns   | Communication backbone     |
| 1.3     | Actions & Timers                   | Asynchronous task execution, timing control    | Robot action requests      |
| 1.4     | Python with rclpy                  | Write ROS 2 nodes in Python                    | Implementation language    |
| 1.5     | Launch Files & Parameters          | Package configuration, dynamic reconfiguration | Deployment automation      |
| Lab 1.1 | Build a ROS 2 Publisher/Subscriber | Execute first ROS 2 node                       | Foundation code            |
| Lab 1.2 | Implement a Service                | Request/response communication                 | Service client for robot   |
| Lab 1.3 | Create a ROS 2 Package             | Project organization                           | Package structure template |

**Module 2: Simulation with Gazebo & Unity (Weeks 6–7)**

| Chapter | Topic                       | Learning Outcomes                      | Capstone Contribution     |
| ------- | --------------------------- | -------------------------------------- | ------------------------- |
| 2.1     | Gazebo Basics & Physics     | Simulation engine, physics models      | Simulation environment    |
| 2.2     | URDF & Robot Descriptions   | Unified Robot Description Format       | Robot model definition    |
| 2.3     | Sensors in Gazebo           | Camera, LiDAR, IMU simulation          | Sensor data pipeline      |
| 2.4     | Unity for Robotics          | High-fidelity rendering, visualization | Visualization alternative |
| 2.5     | Sim-to-Real Considerations  | Reality gap, validation strategies     | Hardware validation       |
| Lab 2.1 | Load a Robot in Gazebo      | Configure URDF + physics               | Simulation ready          |
| Lab 2.2 | Publish Sensor Data         | Simulate camera/lidar output           | Sensor integration        |
| Lab 2.3 | Control Robot in Simulation | Send commands from ROS 2 node          | Simulated control loop    |

**Module 3: NVIDIA Isaac Platform (Weeks 8–10)**

| Chapter | Topic                           | Learning Outcomes                          | Capstone Contribution |
| ------- | ------------------------------- | ------------------------------------------ | --------------------- |
| 3.1     | Isaac Sim Overview & Workflows  | Photorealistic simulation, synthetic data  | Training environment  |
| 3.2     | Building Isaac Environments     | Scene creation, asset management           | Capstone world        |
| 3.3     | SLAM & Autonomous Navigation    | Visual localization, path planning         | Autonomous movement   |
| 3.4     | Isaac ROS Integration           | Hardware-accelerated perception            | Perception pipeline   |
| 3.5     | Object Detection & Manipulation | Computer vision, grasping                  | Object interaction    |
| Lab 3.1 | Create Isaac Sim Environment    | Configure humanoid in photorealistic world | Capstone environment  |
| Lab 3.2 | Implement SLAM Pipeline         | Visual odometry, mapping                   | Localization          |
| Lab 3.3 | Autonomous Navigation Task      | Point-to-point navigation with obstacles   | Navigation controller |

**Module 4: Vision-Language-Action (VLA) Systems (Weeks 11–13)**

| Chapter | Topic                                 | Learning Outcomes                        | Capstone Contribution  |
| ------- | ------------------------------------- | ---------------------------------------- | ---------------------- |
| 4.1     | VLA Architecture Fundamentals         | Multimodal AI, vision-language models    | AI architecture        |
| 4.2     | Language-to-Action Mapping            | NLP → robot actions, LLM planning        | Command interpretation |
| 4.3     | Voice Interface & Speech Recognition  | Whisper integration, audio processing    | Voice input            |
| 4.4     | Sensor Feedback Loops                 | Real-time perception → action adaptation | Reactive control       |
| 4.5     | System Integration & Deployment       | End-to-end pipeline assembly             | Full integration       |
| Lab 4.1 | Language-to-Action Pipeline           | NLP model → ROS 2 actions                | Capstone action engine |
| Lab 4.2 | Voice Command Processing              | Speech recognition + action execution    | Voice control          |
| Lab 4.3 | Capstone Project: Autonomous Humanoid | Integrated voice-controlled robot        | Capstone deliverable   |

**Capstone Chapter**

| Chapter    | Focus                                                        |
| ---------- | ------------------------------------------------------------ |
| Capstone.1 | Project Requirements & Deliverables (code + video + report)  |
| Capstone.2 | Grading Rubrics & Evaluation Criteria (5-point Likert scale) |
| Capstone.3 | Example Projects & Reference Implementations                 |
| Capstone.4 | Deployment Guide (simulation or physical hardware)           |

**Hardware Setup Chapter**

| Chapter    | Focus                                            |
| ---------- | ------------------------------------------------ |
| Hardware.1 | Minimum Requirements (simulator-only)            |
| Hardware.2 | Recommended Setup (Jetson Orin Nano + RealSense) |
| Hardware.3 | Optional: Unitree G1 Physical Deployment         |
| Hardware.4 | Safety Protocols & Best Practices                |

**Output**: Detailed module breakdowns with chapter titles, learning outcomes, and capstone mappings.

### Step 1.2: Define Content Dependencies & Week-by-Week Mapping

**Objective**: Create a Gantt-like timeline showing which chapters/labs map to each week, identifying prerequisites.

**Dependency Graph**:

```
Week 1:   Module 0.1-0.2 (no code)
Week 2:   Module 0.3-0.4 + Hardware.1-2 (setup)
Week 3:   Module 1.1-1.2 + Lab 1.1 (ROS basics)
Week 4:   Module 1.3-1.4 + Lab 1.2-1.3 (Python + packages)
Week 5:   Module 1.5 + Review (launch files, capstone connection)
       ↓
Week 6:   Module 2.1-2.2 + Lab 2.1 (Gazebo + URDF)
Week 7:   Module 2.3-2.5 + Lab 2.2-2.3 (sensors + sim-to-real)
       ↓
Week 8:   Module 3.1-3.2 + Lab 3.1 (Isaac setup)
Week 9:   Module 3.3-3.4 + Lab 3.2 (SLAM + nav)
Week 10:  Module 3.5 + Review (perception, capstone pipeline)
       ↓
Week 11:  Module 4.1-4.2 + Lab 4.1 (VLA + language mapping)
Week 12:  Module 4.3-4.4 + Lab 4.2 (voice + feedback)
Week 13:  Capstone.1-4 + Lab 4.3 (integration + submission)
```

**Output**: Week-by-week content calendar with prerequisites identified.

---

## Phase 2: File Generation Strategy

### Step 2.1: File Naming & Organization Convention

**Objective**: Define consistent naming conventions to ensure Docusaurus sidebar auto-generation works correctly.


**Key Rules**:

- File names: lowercase, hyphen-separated, descriptive
- IDs: match file path with colons (e.g., `m1-nodes-topics`)
- `sidebar_position`: sequential integer per module (auto-sorts sidebar)
- Labs prefixed with `lab-` and numbered
- No duplicate IDs across the entire docs/ tree

**Output**: Directory structure + naming convention guide.

### Step 2.2: Content Generation Order (Priority-Based)

**Objective**: Define which files to generate first to minimize blocking dependencies.

**Generation Order** (Wave 1–4):

**Wave 1: Foundations & Structure** (Start immediately)

1. `docs/intro.md` — Course overview, learning outcomes
2. `docs/module-0-foundations/intro.md` — Module intro template
3. `docs/module-1-ros2/intro.md` — Module intro template
4. [Repeat for modules 2–4]
5. `docs/glossary.md` — Glossary with all terms (grows iteratively)
6. `docs/hardware-setup/01-minimum-requirements.md` — Simulator-only path
7. `docs/capstone/01-requirements.md` — Capstone definition

**Wave 2: Module 0 & Module 1 (ROS 2)** (Week 1–2)

1. Module 0 chapters (0.1–0.4): Conceptual, no code
2. Module 1 chapters (1.1–1.5): Theory + examples
3. Module 1 labs (Lab 1.1–1.3): Step-by-step instructions

**Wave 3: Modules 2–3 (Simulation & Isaac)** (Week 3–4)

1. Module 2 chapters (2.1–2.5): Build on Module 1
2. Module 2 labs (Lab 2.1–2.3): Hands-on simulation
3. Module 3 chapters (3.1–3.5): Advanced perception
4. Module 3 labs (Lab 3.1–3.3): Isaac + SLAM

**Wave 4: Module 4 & Capstone** (Week 5–6)

1. Module 4 chapters (4.1–4.5): VLA integration
2. Module 4 labs (Lab 4.1–4.3): Voice + capstone
3. Capstone chapters (Capstone.1–4): Full integration
4. Hardware & deployment guides (Hardware.3–4)

**Blocking Dependencies**:

- Module 1 labs must exist before Module 2 chapters reference them
- Module 3 Isaac chapters must exist before VLA integration
- Glossary must be updated as new terms introduced

**Output**: Generation priority order + blocking dependency map.

### Step 2.3: Code Sample & Asset Strategy

**Objective**: Plan where runnable code examples are stored and referenced.

**Code Storage**:

1. **Inline Code** (in Markdown, fenced blocks):

   - Short snippets (<20 lines) or teaching examples
   - Syntax-highlighted, language-tagged
   - Example:
     ```python
     import rclpy
     from rclpy.node import Node
     # ... code ...
     ```

2. **External Code Repositories** (GitHub or similar):

   - Complete, runnable examples (full projects)
   - Lab setup scripts and URDF/SDF files

3. **Assets** (diagrams, screenshots):
   - Store in `docs/assets/` with descriptive names
   - Reference with relative paths
   - Example: `![ROS 2 Node Graph](../assets/diagrams/ros2-graph.png)`

**Code Testing Strategy**:

- Before publication, run all code examples on target platforms (Ubuntu 22.04, macOS, Windows + WSL)
- Document prerequisites (Python 3.10+, ROS 2 Humble, Gazebo version, etc.)
- Capture expected output for verification

**Output**: Code repository structure + asset naming convention.

---

## Phase 3: Writing & Quality Assurance

### Step 3.1: Constitution Alignment Checklist

**Objective**: Ensure every chapter adheres to the constitution before publication.

**Pre-Publication Checklist** (every chapter must pass):

- [ ] **Modular & Progressive**: Chapter is self-contained; prerequisites listed
- [ ] **Hands-On Code**: At least 2 examples + 1 lab; all code is runnable
- [ ] **Docusaurus Markdown**: Frontmatter valid, no raw HTML (except JSX), KaTeX for math
- [ ] **Terminology**: All new terms defined or linked to glossary; consistent naming
- [ ] **Hardware Agnostic**: Code uses ROS 2 abstractions; hardware-specific sections marked
- [ ] **Capstone Connection**: Clearly states how this chapter contributes to capstone
- [ ] **Lab Requirements**: Prerequisites, setup, step-by-step, expected output, troubleshooting
- [ ] **Technical Accuracy**: Code tested; facts verified; links valid
- [ ] **Clarity & Completeness**: No unexplained jargon; no "left as exercise"
- [ ] **Accessibility**: Diagrams for complex systems; captions on all images

**Review Process**:

1. Author submits chapter draft
2. Reviewer checks against constitution
3. If fails: author revises; loop until PASS
4. Mark as "Ready for Docusaurus build"

**Output**: Checklist artifact + review workflow.

### Step 3.2: Consistency & Integration Testing

**Objective**: Verify that chapters link correctly, terminology is consistent, and capstone integrates all modules.

**Tests**:

1. **Docusaurus Build**: `npm run build` succeeds with no warnings
2. **Link Validation**: All internal links (relative paths) resolve; no 404s
3. **Terminology Consistency**: Grep for alternate terms; flag mismatches
4. **Code Snippet Verification**: Sample running 3-5 code examples per module
5. **Capstone Integration**: Verify capstone.md lists reusable outputs from modules 1–4
6. **Sidebar Generation**: `sidebars.js` auto-sorts by `sidebar_position`; no duplicates

**Output**: Test report + fix log.

### Step 3.3: Final Polish Pass

**Objective**: Ensure textbook is publication-ready (tone, formatting, visual consistency).

**Tasks**:

1. Copy editing: Grammar, spelling, consistent voice
2. Visual consistency: Diagrams, code blocks, admonitions formatted uniformly
3. Link audit: All links tested (internal and external)
4. Image optimization: Resize for web; add alt-text
5. Glossary completeness: All technical terms defined
6. Metadata: Update `docusaurus.config.js` with correct title, logo, social links

**Output**: Publication-ready markdown + configuration.

---

## Phase 4: Deployment & Iteration

### Step 4.1: Docusaurus Build & GitHub Pages Deployment

**Objective**: Generate static site and publish to GitHub Pages.

**Commands**:

```bash
# Build static site
npm run build

# Test local build
npm run serve

# Deploy to gh-pages branch
npm run deploy

# (Or via GitHub Actions CI/CD)
```

### Step 4.2: Post-Launch Monitoring & Iteration

**Objective**: Collect feedback and plan updates.

**Feedback Channels**:

- GitHub Issues (bug reports, content errors)
- Student surveys (clarity, pacing, code examples)
- Instructor feedback (assessment rubrics, labs)

**Iteration Plan**:

1. Prioritize bugs and high-impact feedback
2. Create PHR for each update
3. Update constitution if principles need adjustment
4. Increment version in constitution
5. Redeploy via `npm run deploy`

**Output**: Feedback loop + iteration cadence.

---

## Quality Standards

### Technical Accuracy

- **Requirement**: All code examples run on target platforms (Ubuntu 22.04, macOS, Windows + WSL)
- **Validation**: Before commit, test on each platform; document version requirements
- **Scope**: ROS 2, Gazebo, Isaac versions as of Q1 2025

### Clarity & Completeness

- **Requirement**: No unexplained jargon; every concept explained or linked to glossary
- **Validation**: First-reader test (non-expert reviews chapter)
- **Standard**: Labs are end-to-end with expected output; no scaffolded incompleteness

### Consistency & Accessibility

- **Requirement**: Uniform formatting, terminology, and visual style across all 13 weeks
- **Validation**: Automated checks (Docusaurus linter) + manual review
- **Standard**: Every complex system has a diagram; all images have captions and alt-text

### Capstone Integration

- **Requirement**: Each module's output feeds into capstone; capstone is a coherent whole
- **Validation**: Capstone chapter explicitly lists module dependencies and reusable code
- **Standard**: No disconnected "toy" examples; all labs build toward voice-controlled humanoid

---

## Success Criteria

**Content Completeness**:

- ✅ All 21 weeks of content (Modules 0–4 + Capstone + Hardware) complete and published
- ✅ 20+ chapters with learning outcomes, examples, and labs
- ✅ 10+ hands-on labs with step-by-step instructions and expected output
- ✅ Comprehensive glossary with 50+ defined terms

**Quality & Consistency**:

- ✅ 100% of chapters pass constitution checklist
- ✅ Docusaurus build succeeds with zero warnings
- ✅ All code examples tested and documented (prerequisites, expected output)
- ✅ All internal links valid; all external links verified

**Usability**:

- ✅ Sidebar auto-generates from frontmatter; navigation is clear
- ✅ First example runnable in <30 minutes (simulator)
- ✅ Students report 4/5+ satisfaction with content clarity
- ✅ 80%+ of students complete capstone and deploy robot control system

**Deployment**:

- ✅ Live site on GitHub Pages with correct baseUrl
- ✅ Mobile-responsive design (Docusaurus default)
- ✅ Fast page load times (<3 seconds on 4G)
- ✅ Searchable (Docusaurus default search enabled)

---

## Risk Mitigation

| Risk                      | Impact                     | Mitigation                                                                     |
| ------------------------- | -------------------------- | ------------------------------------------------------------------------------ |
| ROS 2 / Isaac API changes | Code examples break        | Version-pin dependencies; provide migration guides; update on breaking changes |
| Inconsistent terminology  | Student confusion          | Glossary + automated checks; flag mismatches in review                         |
| Low student completion    | Course failure             | Early pilots; adjust pacing; survey feedback                                   |
| Hardware unavailability   | Real robot testing blocked | Simulation-first path; optional hardware tiers                                 |

---

## Next Steps

1. **Step 4.1**: Bootstrap Docusaurus project and initialize folder structure
2. **Step 4.2**: Establish chapter template and style guide
3. **Step 1.1**: Define detailed module architecture and chapter breakdowns
4. **Wave 1**: Generate foundation chapters (intros, glossary, hardware, capstone)
5. **Wave 2–4**: Sequentially generate module content with integrated testing

**Timeline**: 6–8 weeks to publication (depending on team size and writing pace)

---

## Appendix: File Generation Checklist

**Pre-Generation**:

- [ ] Constitution reviewed and approved
- [ ] Chapter template finalized
- [ ] Style guide documented
- [ ] Docusaurus configured (baseUrl, sidebar plugin)
- [ ] Code repository structure created (if external)

**Per-Chapter**:

- [ ] Frontmatter: id, title, sidebar_position, description, keywords
- [ ] Learning outcomes defined
- [ ] Core concepts outlined
- [ ] 2+ worked examples written and tested
- [ ] Lab exercise with all required sections
- [ ] Links to glossary and further reading
- [ ] Diagrams/screenshots created
- [ ] Constitution checklist passed
- [ ] Code tested on target platforms
- [ ] Merged to main branch

**Pre-Deployment**:

- [ ] Docusaurus build succeeds
- [ ] All links tested (internal + external)
- [ ] Sidebar auto-generates correctly
- [ ] Final polish pass (grammar, formatting)
- [ ] GitHub Pages configured
- [ ] Deploy command verified

**Post-Deployment**:

- [ ] Live site verified
- [ ] Mobile responsiveness checked
- [ ] Search functionality tested
- [ ] Analytics configured (if desired)
- [ ] Feedback channels opened (GitHub Issues, surveys)
