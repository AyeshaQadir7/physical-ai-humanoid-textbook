# Physical AI & Humanoid Robotics Textbook Constitution

## Governance Principles (Root Constitution)

This document inherits from the root constitution (`.specify/memory/constitution.md`). Key governance principles include:

1. **Book as Authoritative Source of Truth** - Textbook is the definitive knowledge base
2. **Retrieval-Augmented Generation (RAG) Mandate** - Chatbot operates exclusively via RAG
3. **Traceability and Explainability** - All answers traceable to source material
4. **Strict Scope Boundaries** - Chatbot bounded by textbook content
5. **Technical Accuracy and Verification** - Content current, version-pinned, tested
6. **AI-Native, Chunkable Content Structure** - Content optimized for human reading and AI processing
7. **Spec-Driven Development Governance** - Specifications authoritative; implementation follows specs

See root constitution for technology stack, RAG standards, chatbot operational requirements, and governance processes.

---

## Content & Authoring Principles

### I. Modular, Progressive Learning

Every chapter and module is self-contained yet sequentially progressive. Content must:

- Start with foundational concepts applicable to learners with basic AI/CS knowledge
- Use \source-material\brief.md for book content overview
- Build incrementally toward advanced topics (embodied intelligence, VLA systems, capstone integration)
- Follow a consistent narrative: Introduction → Concepts → Examples → Hands-on → Summary
- Enable independent study of any module without skipping essential prerequisites from earlier modules

**Rationale:** Modularity supports both sequential learning and selective deep-dives. Progressive difficulty ensures learners gain confidence before tackling complex robotics systems.

### II. Hands-On Learning with Real Code

Theory must be paired with executable examples and labs. Every concept requires:

- Code snippets (Python/YAML/SDF/URDF) demonstrating the principle
- Runnable examples targeting either simulation (Gazebo/Isaac) or physical hardware (Jetson Orin, RealSense)
- Explicit prerequisites and setup instructions for each lab
- Clear success criteria ("when you run this, you should see...") to validate learning

**Rationale:** Physical AI demands embodied experimentation. Reading about ROS 2 topics without publishing to one leaves gaps. Code+theory prevents confusion about practice vs. theory.

### III. Docusaurus Markdown Compatibility (MUST)

All content is authored in Markdown compatible with Docusaurus v2+. Requirements:

- Standard Markdown syntax only (no raw HTML except for Docusaurus JSX components)
- Frontmatter: `id`, `title`, `sidebar_position`, `sidebar_label` (required)
- Admonitions: `:::note`, `:::warning`, `:::tip`, `:::danger` for emphasis
- Code blocks: language-tagged fenced blocks (e.g., `\`\`\`python`, `\`\`\`bash`)
- Links: relative paths within the docs structure (e.g., `../module-1/basics.md`)
- Images: stored in `docs/assets/` with descriptive alt-text; paths relative to the file
- No inline LaTeX (use code blocks for equations); KaTeX enabled for math notation

**Rationale:** Docusaurus is the deployment target. Strict adherence ensures zero friction during site generation and GitHub Pages publishing.

### IV. Consistent Terminology & Glossary

All technical terms MUST align with a project-wide glossary. Rules:

- First usage of a term in each document must link to the glossary or include an inline definition
- Use consistent naming: "ROS 2 node" (not "ROS2 process"), "vision-language-action model" (not "VLA"), "unified robot description format" (not "URDF description")
- Establish canonical spellings and hyphenation (e.g., "humanoid" not "human-like", "sim-to-real" not "sim to real")
- Abbreviations introduced after full term: "Robot Operating System 2 (ROS 2)" on first use

**Rationale:** Learners easily confuse terminology. Consistency across chapters reduces cognitive load and supports glossary-based review and retention.

### V. Hardware/Simulation Agnostic Where Possible

Content must support multiple execution paths:

- Simulation-first labs use Gazebo or Isaac Sim (available to all students)
- Physical labs document equivalents for Jetson Orin, RealSense, Unitree Go2/G1
- Code must clearly indicate: "Runs in simulation only", "Requires Jetson", "Tested on Go2 only", etc.
- Fallbacks: if NVIDIA Isaac is unavailable, Gazebo + ROS 2 alone must suffice for Modules 1-3

**Rationale:** Students have different access to hardware. Making content adaptable to simulation or budget-conscious options broadens accessibility while respecting real-world constraints.

### VI. Capstone Integration (Coherence Guarantee)

Every module (1-4) contributes a deliverable toward the Capstone Project: "Autonomous Humanoid with Voice-to-Action". Rules:

- Module 1 (ROS 2): Build a robotic control framework
- Module 2 (Simulation): Deploy the framework in Gazebo/Isaac
- Module 3 (Isaac): Add AI perception (SLAM, nav, object detection)
- Module 4 (VLA): Integrate voice commands and LLM planning
- Capstone chapter explicitly lists which outputs from Modules 1-4 are reused
- All code examples incrementally build toward the capstone (no disconnected toys)

**Rationale:** A cohesive capstone reinforces the core thesis: "From digital AI to embodied intelligence." Fragmented examples confuse learners about real-world integration.

## Content Structure & Standards

### Chapter Template (MANDATORY)

Every chapter MUST follow this structure:

1. **Frontmatter** (YAML): `id`, `title`, `sidebar_position`, `description`, `keywords`
2. **Introduction** (1–2 paragraphs): Why this topic matters for Physical AI; context within the broader course
3. **Learning Outcomes** (bulleted list): What the reader will know/do after this chapter
4. **Core Concepts** (3–5 major sections): Theory, architecture, design decisions
5. **Examples** (at least 2 worked examples): Pseudo-code or full code with explanations
6. **Hands-On Labs** (at least 1 lab): Step-by-step instructions, expected output, troubleshooting
7. **Summary & Next Steps** (1–2 paragraphs): Key takeaways; how this topic bridges to the next chapter
8. **Further Reading** (optional): Links to official documentation, papers, external resources
9. **Code Repository** (if applicable): Links to example repos or GitHub gists for all runnable code

### Content Quality Standards

- **Technical Accuracy**: All code runs (tested on target platform). All facts about ROS 2, Gazebo, Isaac are current (as of publication date)
- **Clarity**: No unexplained jargon; assumptions about reader knowledge stated upfront
- **Completeness**: No "left as exercise to the reader" without guidance; labs are end-to-end (not scaffolded incompletely)
- **Accessibility**: Visuals (diagrams, architecture charts) for every complex system; captions for all images

### Lab Requirements

Each hands-on lab MUST include:

- **Title & Learning Goal**: e.g., "Lab 2.1: Publish Temperature Data on a ROS 2 Topic"
- **Prerequisites**: Required knowledge, tools (ROS 2 installed? Gazebo running?), estimated time
- **Setup Instructions**: Reproducible steps (e.g., clone a repo, install dependencies, build a package)
- **Step-by-Step Tasks**: Numbered, action-oriented (not conceptual)
- **Expected Output**: Screenshots, console output, or behavioral description (e.g., "the robot should move forward 1 meter")
- **Verification Checklist**: How the student knows they succeeded
- **Troubleshooting**: Common failures (missing packages, port conflicts, syntax errors) and fixes
- **Extension** (optional): Advanced challenge that deepens understanding or connects to the next module

## Module Coverage & Mapping

| Module | Topic                                            | Weeks | Key Deliverables                               |
| ------ | ------------------------------------------------ | ----- | ---------------------------------------------- |
| 0      | Foundations (Physical AI, Embodied Intelligence) | 1–2   | Conceptual understanding; no code yet          |
| 1      | ROS 2 Fundamentals                               | 3–5   | Python ROS 2 package; pub/sub and services     |
| 2      | Simulation (Gazebo & Unity)                      | 6–7   | URDF model; simulated sensors and physics      |
| 3      | NVIDIA Isaac & Perception                        | 8–10  | SLAM pipeline; autonomous navigation           |
| 4      | Vision-Language-Action (VLA)                     | 11–13 | Voice-to-action pipeline; capstone integration |

**Guarantee**: Content for all weeks (1–13) must be complete before publication. No "coming soon" placeholders.

## Docusaurus Deployment Standards

### Markdown Repository Structure

```
docs/
├── intro.md                              (Course overview, learning outcomes)
├── module-0-foundations/
│   ├── intro.md
│   ├── what-is-physical-ai.md
│   ├── embodied-intelligence.md
│   ├── humanoid-landscape.md
│   └── glossary.md
├── module-1-ros2/
│   ├── intro.md
│   ├── architecture-overview.md
│   ├── nodes-topics-services.md
│   ├── python-rclpy.md
│   ├── urdf-basics.md
│   ├── lab-1-1-your-first-node.md
│   └── ...
├── module-2-simulation/
├── module-3-isaac/
├── module-4-vla/
├── capstone/
├── assets/
│   ├── diagrams/
│   ├── screenshots/
│   └── code-snippets/
└── glossary.md
```

### Docusaurus Configuration Requirements

- `docusaurus.config.js`: Must define correct `baseUrl` for GitHub Pages
- `sidebars.js`: Auto-generates sidebar from `sidebar_position` in Markdown frontmatter
- All chapter IDs must be globally unique (no duplicate `id` fields across docs)
- Deployment: GitHub Pages via `gh-pages` branch (automated via `yarn deploy`)

### Link & Asset Conventions

- Internal links: relative paths (e.g., `../module-1/basics.md`)
- External links: full HTTPS URLs (e.g., `https://docs.ros.org/...`)
- Images: stored in `docs/assets/` with descriptive names and alt-text
- Code files: either inline in Markdown or linked to `code-examples/` directory with full path

## Governance & Amendment Process

### Compliance Verification

- Every spec, plan, and task generated for a chapter MUST reference this constitution
- Every chapter draft MUST pass a pre-publication review checklist:
  - [ ] Frontmatter complete and valid YAML
  - [ ] Markdown renders without errors in Docusaurus
  - [ ] All code examples run and produce expected output
  - [ ] All links (internal and external) are valid
  - [ ] Terminology aligns with project glossary
  - [ ] Labs include setup, steps, verification, and troubleshooting
  - [ ] Capstone relevance clearly stated

### Version Control

- CONSTITUTION_VERSION uses MAJOR.MINOR.PATCH (semantic versioning)
- MAJOR: Removal or fundamental redefinition of a principle (rare)
- MINOR: New principles, new sections, or significant expansions (e.g., adding "V. Hardware Agnostic")
- PATCH: Clarifications, typo fixes, examples, link updates
- LAST_AMENDED_DATE updates on any change; RATIFICATION_DATE never changes (original adoption)

### Amendment Workflow

1. Identify principle, section, or standard requiring change (explain why in commit message)
2. Edit this file; update LAST_AMENDED_DATE and increment version
3. Validate: Run consistency checklist (see step 4 in constitution command above)
4. Create a PHR or ADR documenting the rationale
5. Commit with message: `docs: amend constitution to vX.Y.Z (principle additions + governance update)`

### Runtime Guidance

For day-to-day authoring decisions not covered here, refer to:

- **Spec Template** (`.specify/templates/spec-template.md`): Governs feature spec structure
- **Plan Template** (`.specify/templates/plan-template.md`): Governs chapter plan and design
- **Tasks Template** (`.specify/templates/tasks-template.md`): Governs lab and content tasks
- **PHR Template** (`.specify/templates/phr-template.prompt.md`): Records decisions for traceability

This constitution supersedes all default practices. Any deviation requires explicit justification and amendment.

**Version**: 1.0.0 | **Ratified**: 2025-12-10 | **Last Amended**: 2025-12-10
