# ADR-003: Content Architecture - Modular Progressive Design with Capstone Integration

**Status**: Accepted
**Date**: 2025-12-10
**Decision Cluster**: Pedagogical Structure & Learning Progression
**References**: plan.md (Phase 1), spec.md (Module Coverage, Capstone Integration), constitution.md (I. Modular Progressive Learning, VI. Capstone Integration)

---

## Context

The textbook teaches Physical AI through four sequential technology modules (ROS 2 → Simulation → Isaac → VLA), culminating in a capstone project. However, several design questions required resolution:

1. **Module dependencies**: Should modules be strictly sequential, or can students self-select?
2. **Capstone coherence**: How do we prevent capstone from feeling disconnected from modules?
3. **Content scope**: How many chapters per module? How long should each chapter be?
4. **Code example strategy**: Should examples evolve incrementally across modules, or be standalone?
5. **Assessment alignment**: Do all modules contribute equally to capstone grading?

**Constraints**:
- 13-week quarter constraint (~10–15 hours/week student workload)
- Mix of self-paced and instructor-led delivery
- Simulation-first, hardware-optional accessibility
- 80% completion target (ambitious for a technical capstone)

**Stakeholder goals**:
- Students: Clear progression from basics → advanced; no prerequisites assumed beyond Python + basic ML/AI
- Instructors: Flexibility to adjust pacing per cohort; clear learning outcomes for assessment
- Architects: Maintainability as frameworks (ROS 2, Gazebo, Isaac) evolve

---

## Decision

**Adopt a modular progressive architecture with incremental capstone building**:

### 1. Module Structure (5 modules, 21 weeks)

| Module | Weeks | Focus | Capstone Role | Dependencies |
|--------|-------|-------|---------------|--------------|
| **0: Foundations** | 1–2 | Why Physical AI matters; humanoid landscape | Motivation & context | None (prerequisites only) |
| **1: ROS 2** | 3–5 | Robotic control framework | Build control architecture | Module 0 (optional review) |
| **2: Gazebo & Unity** | 6–7 | Simulation & physics | Deploy framework in virtual world | Module 1 (required) |
| **3: Isaac** | 8–10 | Perception & autonomy | Add sensing & navigation | Module 2 (required) |
| **4: VLA** | 11–13 | AI + language + action | Integrate voice commands & planning | Module 3 (required) |

**Strict progression**: Module 1 → 2 → 3 → 4. Each module builds on the previous; skipping is explicitly discouraged.

### 2. Chapter Design (4–5 chapters per module)

**Each chapter follows this template** (per constitution):
1. **Introduction**: Why this topic matters for Physical AI
2. **Learning Outcomes**: 3–5 concrete, measurable outcomes
3. **Core Concepts**: 3–5 major sections (theory + architecture)
4. **Examples**: ≥2 worked examples with code + explanation
5. **Hands-On Lab**: 1 lab per chapter (see below)
6. **Summary & Next Steps**: Key takeaways; bridge to next chapter
7. **Further Reading**: Links to official docs + papers

**Length**: 2,500–4,000 words per chapter (10–15 min read + 30–45 min hands-on lab)

### 3. Incremental Capstone Building

**Each module contributes a reusable deliverable toward the capstone**:

- **Module 1 Output**: `robot_controller.py` (ROS 2 package with pub/sub nodes)
- **Module 2 Output**: `urdf/humanoid.urdf` + Gazebo world file (simulated environment)
- **Module 3 Output**: `slam_node.py` + `nav_planner.py` (perception & navigation)
- **Module 4 Output**: `vla_interface.py` (voice-to-action bridge)

**Capstone assembles all outputs**: Voice command → VLA module → navigation → robot control → simulated movement

**Explicit connection**: Each module chapter explicitly states: "Your `robot_controller.py` from Module 1 is reused in Module 2 as the control layer." This prevents student perception of disconnected toy examples.

### 4. Lab Exercise Progression

**Labs evolve across modules, building on previous work**:

| Module | Lab Focus | Builds On | Capstone Connection |
|--------|-----------|-----------|---------------------|
| 1.1 | Pub/Sub node | None (first code) | Foundation for all communication |
| 1.2 | Service client | Pub/Sub (1.1) | Service for robot actions |
| 1.3 | ROS package | Services (1.2) | Project structure for all modules |
| 2.1 | Gazebo + URDF | ROS package (1.3) | Load humanoid model |
| 2.2 | Sensor sim | Gazebo (2.1) | Publish sensor topics |
| 2.3 | Control loop | Sensor sim (2.2) + Pub/Sub (1.1) | Closed-loop control in simulation |
| 3.1 | Isaac environment | Gazebo (2.1) | Advanced simulation |
| 3.2 | SLAM pipeline | Sensor sim (2.2) | Localization |
| 3.3 | Navigation | SLAM (3.2) | Path planning |
| 4.1 | VLA pipeline | Navigation (3.3) + ROS (1.2) | Command interpretation |
| 4.2 | Voice interface | VLA (4.1) | Voice input |
| 4.3 | **Capstone** | All above | Assemble & deploy |

**No standalone silos**: Every lab reuses prior outputs. Capstone assembly is nearly automatic if students complete all labs.

### 5. Assessment Alignment

**Grading weights** (per clarification Q1):
- Quizzes (formative): 20% — Check conceptual understanding (per module)
- Labs (formative + summative): 40% — Hands-on competency (per chapter)
- Capstone (summative): 40% — Integrated demonstration (Week 13)

**Each module contributes equally to capstone**: Module 1's ROS code, Module 2's simulation, Module 3's perception, Module 4's AI integration all required for capstone to function.

### 6. Flexibility Within Structure

**Strict progression, flexible pacing**:
- Instructors can adjust week-by-week schedule (compress Module 1 to 2.5 weeks if cohort is advanced)
- Self-paced students can take breaks between modules
- Lab retakes encouraged (re-submit if failed initially)
- **But**: Module order is fixed; Module 3 always comes after Module 2

**Optional depth**:
- "Extension challenges" in each lab for students who want to go deeper
- Further reading links to advanced topics (not required, for curious students)
- Capstone "exceptional" tier for students adding novel features (sensor fusion, advanced planning, etc.)

---

## Rationale

### Why Modular?

- **Conceptual clarity**: Each module (ROS 2, Simulation, Isaac, VLA) is a distinct technology; module boundaries align with industry skill domains
- **Reusability**: Students (and instructors) can extract individual modules for other courses (e.g., "Advanced ROS 2" course uses Module 1 + extensions)
- **Maintenance**: Framework updates (ROS 2.1, Isaac v2, new VLA models) can be scoped to single module; others less affected

### Why Progressive?

- **Cognitive load management**: Start simple (ROS 2 pub/sub), add complexity gradually (simulation, then perception, then AI)
- **Confidence building**: Early wins (Module 1 labs) motivate students to persist through harder topics (Isaac perception)
- **Prerequisite clarity**: Module 2 requires ROS mastery from Module 1; not assumed knowledge

### Why Capstone Integration?

- **Coherent narrative**: Capstone is not "one more project"; it's the natural outcome of four modules working together
- **Intrinsic motivation**: Students see how modules connect; less "why am I learning this?" frustration
- **Practical relevance**: Capstone mirrors real robotics: control system → sim validation → perception → task planning
- **Grading fairness**: All modules weighted equally; no single module can tank a grade if others are strong

### Tradeoffs

| Aspect | Upside | Downside |
|--------|--------|----------|
| **Strict progression** | Clear prerequisites; no confusion | Less flexibility; can't skip "boring" Module 1 |
| **Incremental capstone** | Coherent narrative; reuses code | Complex scaffolding; more instructor grading |
| **Modular chapters** | Reusable; maintainable | Boundaries might feel artificial to some; some duplication |
| **4–5 chapters per module** | Manageable scope; regular assessments | Risk of superficiality if chapters too compressed |

### Risks & Mitigations

- **Risk**: Module boundaries feel artificial (e.g., Isaac Sim in Module 3 uses concepts from Module 2, causing rework)
  - **Mitigation**: Careful chapter design to minimize rework; explicit "prerequisites from Module X" in each chapter
- **Risk**: Capstone assembly too complex; students struggle to integrate modules
  - **Mitigation**: Week 13 includes assembly guide; example integration code provided; TAs available for debugging
- **Risk**: 80% completion target unrealistic if progression too strict
  - **Mitigation**: Early intervention (surveys Week 4); adjust pacing; optional drop-in tutoring; simulation-only path accessible to all

---

## Consequences

### Positive
- **Student clarity**: "I'm learning ROS 2 this week, and it's the foundation for controlling robots in any environment"
- **Instructor control**: Clear rubrics per module; straightforward to identify struggling students and intervene
- **Curriculum evolution**: When ROS 2.1 released, Module 1 updates independently; Modules 2–4 less affected
- **Capstone coherence**: Capstone demo isn't a surprise; students see it coming from Week 1
- **80% completion realistic**: Modular structure allows students to succeed incrementally; fewer "all-or-nothing" failures
- **Team teaching**: Modules can be taught by different instructors (ROS expert, Simulation expert, etc.)

### Negative
- **Initial authoring overhead**: Designing incremental capstone requires planning all modules upfront (higher coordination cost)
- **Student pacing flexibility**: "Can I skip Module 0?" answer is no, even if student knows why Physical AI matters
- **Code reuse constraints**: Students who want to "start fresh" in Module 2 can't; must build on Module 1 output
- **Grading complexity**: Labs build on prior labs; if Module 1 lab failed, Module 2 lab might also fail (cascading issues)

### Long-term Sustainability
- **Framework updates**: When ROS 2 major version released (e.g., 2025's ROS 3.0 hypothetically), Module 1 updated; schedule shift if needed, but structure remains
- **Capstone persistence**: Voice-to-action is timeless concept; specific VLA models (GPT-4, Llama, etc.) swap out; architecture persists

---

## Implementation

### Immediate (Phase 1: Content Architecture)
1. Define exact chapters per module (see plan.md, Step 1.1)
2. Map incremental capstone outputs: Module 1 → Module 4
3. Design lab progression chain (see above)
4. Identify prerequisite chain: "Module 2, Chapter 2.1 requires ROS 2 nodes (Module 1.1)"

### Phase 2 (File Generation)
1. Create chapter templates with "prerequisite" section
2. Add "capstone contribution" section to every chapter
3. Create assembly guide for Week 13 capstone

### Phase 3 (Authoring)
1. Author modules in order (0 → 1 → 2 → 3 → 4)
2. Test lab progression: Run 1.1, then 1.2, then 1.3, verify 1.3 output works in Module 2
3. Validate capstone assembly: All Module 1–4 outputs combine to run full capstone (voice to robot movement)

---

## Related Decisions

- **ADR-001**: Docusaurus (platform to publish modular structure)
- **ADR-002**: Hardware tiers (affects how modules deploy—simulation, Jetson, or physical—but structure unchanged)

---

## Sign-Off

**Decision Maker**: Architect (on behalf of project team)
**Consensus**: Aligns with specification (capstone clarification) and constitution (I, VI)

---

**Next Action**: Phase 1, Step 1.1: Define exact chapters per module with capstone contribution mapping
