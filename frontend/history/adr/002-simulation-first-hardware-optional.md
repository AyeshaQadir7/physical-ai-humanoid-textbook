# ADR-002: Hardware Accessibility Strategy - Simulation-First with Optional Physical Tiers

**Status**: Accepted
**Date**: 2025-12-10
**Decision Cluster**: Hardware Requirements & Learning Path Flexibility
**References**: plan.md (Phase 1, Step 1.1; Phase 2, Step 2.3), spec.md (Clarifications), constitution.md (V. Hardware/Simulation Agnostic)

---

## Context

Physical AI education requires embodied experimentation—students must control robots to understand how digital AI translates to physical actions. However:

- **Hardware costs are prohibitive**: A full humanoid robot (Unitree G1, ~$16k) is inaccessible to most students
- **Unequal access creates inequity**: Only privileged institutions could afford dedicated robot labs
- **Simulation has matured**: Gazebo and NVIDIA Isaac Sim provide physics-realistic environments for learning control
- **Capstone requires proof**: Students must demonstrate robot control, but it doesn't mandate physical hardware

**Student personas span broad backgrounds and budgets**:
- Online students with laptops only
- On-campus students with lab access
- International students with shipping/customs barriers
- Institutions with 1-2 shared robots for classroom demos

The 13-week course must remain accessible to all, without compromising pedagogical integrity.

---

## Decision

**Adopt a three-tier hardware strategy with simulation as the primary learning path**:

### Tier 1: Minimum (Simulator-Only) - $0
- **Hardware**: Any laptop/desktop with GPU or CPU-only fallback
- **Software**: Ubuntu 22.04 (or Windows + WSL) + ROS 2 Humble + Gazebo
- **Capability**: Complete modules 1–4; capstone in full simulation
- **Target**: Online students, bandwidth-constrained regions, budget-limited institutions
- **Guarantee**: 100% of curriculum completable; no hardware barrier

### Tier 2: Recommended (Edge AI Kit) - ~$700
- **Hardware**: NVIDIA Jetson Orin Nano (8GB) + Intel RealSense D435i camera + ReSpeaker USB mic
- **Software**: JetPack OS + ROS 2 + Isaac ROS perception stack
- **Capability**: All of Tier 1, plus hands-on perception (SLAM, object detection) on real edge device
- **Target**: On-campus students, institutions with lab funding
- **Advantage**: Real latency/resource constraints; bridges sim-to-real

### Tier 3: Optional (Physical Robot) - ~$16,000+
- **Hardware**: Unitree G1 humanoid robot (or alternative, e.g., Boston Dynamics Spot for quadrupeds)
- **Software**: Robot OS + ROS 2 integration + proprietary SDKs
- **Capability**: All of Tier 2, plus physical deployment; capstone on real robot
- **Target**: Research institutions, companies, advanced students
- **Caveat**: Not required for course completion; exemplar only

### Code & Example Strategy

**All code examples use ROS 2 abstractions** (independent of hardware):
- Pub/sub patterns for sensor data (works equally on sim or physical)
- Action servers for robot control (abstraction layer, no hardware lock-in)
- Hardware-specific sections clearly marked: "Requires Jetson", "Physical deployment only", "Simulation-only"
- Example:
  ```python
  # This works identically on Gazebo or Unitree G1 via ROS 2 abstraction
  self.cmd_pub.publish(Twist(linear=Vector3(x=1.0)))
  ```

---

## Rationale

### Why Simulation-First?

- **Accessibility**: Students can start immediately; no hardware setup barrier
- **Safety**: Learning to control robots in simulation prevents crashes and injuries
- **Iteration velocity**: Simulation is faster—reset environment instantly, run experiments repeatedly
- **Reproducibility**: Exact same physics conditions every time (unlike physical hardware with wear, calibration drift)
- **Cost-benefit**: Gazebo is free; NVIDIA Isaac is free for developers; vastly cheaper than purchasing hardware

### Why Optional Hardware Tiers?

- **Meet students where they are**: Online-only students don't need Jetson; on-campus labs can invest selectively
- **Progressive complexity**: Tier 1 (simulator) → Tier 2 (edge AI latency, real sensors) → Tier 3 (physical dynamics) is a natural progression
- **Institutional flexibility**: Small institutions start with Tier 1; scale to Tier 2/3 as course matures
- **Capstone still meaningful**: Tier 1 capstone (simulation) demonstrates voice-controlled avatar humanoid; Tier 3 capstone (physical) demonstrates same on real robot

### Why Unitree G1 as Reference Platform (Not Mandate)?

- **Industry relevance**: G1 is commercially available, well-supported in robotics labs (Stanford, MIT, others)
- **ROS 2 compatible**: SDK available; lowers integration barrier vs. proprietary closed systems
- **Realistic**: Humanoid form factor (2 arms, 2 legs) matches learning goals
- **Alternative agnostic**: All code remains platform-agnostic; example could swap to Boston Dynamics Spot or academic humanoid without code change

### Tradeoffs

| Aspect | Upside | Downside |
|--------|--------|----------|
| **Simulation-first** | Accessible, safe, fast iteration | Gaps in sim-to-real (latency, wear, calibration) |
| **Optional hardware** | Reduces barrier to entry | Some students miss tactile embodiment; may feel artificial |
| **Jetson Tier 2** | Bridges gap; real edge constraints | $700 upfront; some students can't afford |
| **Unitree G1 reference** | Industry-relevant, documented | Expensive; not all labs can afford |

### Risk Mitigation

- **Sim-to-real gap**: Module 2.5 (Sim-to-Real Considerations) teaches validation strategies; Tier 2 (Jetson) bridges the gap
- **Inaccessibility of Tier 3**: Capstone is 100% completable in Tier 1; Tier 3 is exemplar, not requirement
- **Perception learning without real sensors**: Tier 2 (Jetson + RealSense) solves this; also cheaper than full robot

---

## Consequences

### Positive
- **80% completion rate achievable**: No student turned away due to hardware cost
- **Scalable**: Institutions can start small (Tier 1) and expand (Tier 2/3) based on budget/demand
- **Capstone remains credible**: Demonstrating robot control (simulated or physical) proves learning; medium is flexible
- **Pragmatic**: Reflects reality—most roboticists start with simulation; physical robots reserved for validation
- **Teacher/researcher enablement**: Instructors can teach on laptops; deploy real robots later if institutional resources allow

### Negative
- **Student perception**: Some students may feel they're "cheating" with simulation (need messaging that sim is legitimate)
- **Latency differences**: Tier 1 students won't experience real control delays; Tier 2/3 students will (but documented in curriculum)
- **Hardware setup complexity**: Tier 2/3 setup guides add authoring burden
- **Fragmentation**: Labs may not share code across tiers (mitigation: strict ROS 2 abstraction rules in constitution)

### Long-term Sustainability
- **Hardware evolves**: In 2 years, better/cheaper edge boards emerge; curriculum remains viable by keeping ROS 2 abstractions front-and-center
- **Simulator maintenance**: Gazebo updates regularly; Isaac Sim supported by NVIDIA; reasonable maintenance cadence

---

## Implementation

### Curriculum Design

1. **Week 2**: Present all three tiers in Hardware Setup chapter; students self-select
2. **Module 1–4**: All code works on Tier 1 (simulator); hardware-specific notes for Tier 2/3
3. **Module 3 (Isaac)**: Intro to edge AI concepts; Tier 2 students follow with real Jetson
4. **Module 4 (VLA)**: Voice-to-action works in all tiers; Tier 3 students get physical demo video
5. **Capstone**: Grading rubric accepts simulator or physical; equivalent credit either way

### Code Examples

**Template for hardware-aware examples**:
```markdown
## Hands-On Lab: [Topic]

### Prerequisites
- Requires: Ubuntu 22.04 + ROS 2 + Gazebo (all tiers)
- Optional: Jetson Orin Nano for physical sensor data (Tier 2+)

### Lab 1.1: ROS 2 Publisher/Subscriber

#### Option A: Simulation (All students)
```bash
# Gazebo is free; everyone starts here
gazebo
# [steps...]
```

#### Option B: Physical (Tier 2+)
```bash
# If you have a Jetson:
ssh jetson@jetson-host
# [steps...]
```

### Capstone Grading
- Simulator capstone: Robot navigates 10x10m maze, responds to 3 voice commands (video + code + report)
- Physical capstone: Same task on real hardware (equivalent rubric score)

---

## Related Decisions

- **ADR-001**: Docusaurus + GitHub Pages (publishing; independent of hardware strategy)
- **ADR-003**: Modular Progressive Architecture (pedagogy; supports multi-tier hardware via ROS 2 abstraction)

---

## Sign-Off

**Decision Maker**: Architect (on behalf of project team)
**Consensus**: Aligns with specification (hardware clarifications); supports accessibility goals

---

**Next Action**: Phase 0 includes Tier 1 setup guide; Tier 2/3 guides authored in Phase 2
