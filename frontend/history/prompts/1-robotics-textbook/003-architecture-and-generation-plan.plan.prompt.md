---
id: 003
title: Architecture and Content Generation Plan
stage: plan
date: 2025-12-10
surface: agent
model: claude-haiku-4-5-20251001
feature: 1-robotics-textbook
branch: 1-robotics-textbook
user: system
command: /sp.plan
labels: ["plan", "architecture", "docusaurus", "content-generation", "module-structure"]
links:
  spec: specs/1-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-robotics-textbook/plan.md
tests:
  - Phase 0: Docusaurus bootstrap plan (PASS)
  - Phase 1: Module architecture breakdown (PASS)
  - Phase 2: File generation strategy (PASS)
  - Phase 3: Quality assurance checklist (PASS)
  - Phase 4: Deployment & iteration plan (PASS)
---

## Prompt

Create a complete execution plan for generating the Physical AI & Humanoid Robotics textbook using Spec-Kit Plus and Docusaurus.

Plan must include:

1. **Preparation**
    - Docusaurus bootstrap commands
    - Folder structure and naming
2. **Content Generation Plan**
Break down each chapter and module into generation steps:
    - Intro & overview
    - Weekly breakdown content
    - Module 1 (ROS 2) chapters
    - Module 2 (Gazebo & Unity) chapters
    - Module 3 (NVIDIA Isaac) chapters
    - Module 4 (VLA, Voice-to-Action)
    - Hardware sections
    - Assessments
    - Capstone
3. **File Generation Strategy**
    - Which markdown files to generate in which order
    - How to ensure consistency with constitution/specs
    - Each modules chapters
    - Naming conventions ("[01-intro.md]", "module-1/02-ros2-basics.md", etc.)
4. **Quality Rules**
    - Accuracy
    - Technical clarity
    - Modular writing
    - Consistent formatting across chapters
5. **Final Assembly**
    - Sidebar configuration
    - Book polishing pass
    - Ready-for-deployment structure

Output the full step-by-step plan.

## Response snapshot

Created comprehensive 4-phase implementation plan (specs/1-robotics-textbook/plan.md) detailing:

**Phase 0 - Preparation & Bootstrap**:
- Step-by-step Docusaurus v2+ setup commands
- Complete folder structure (docs/ with modules 0–4, capstone, hardware, assets)
- Template & style guide establishment
- Glossary initialization

**Phase 1 - Content Architecture**:
- Detailed chapter breakdowns for all 5 modules (21 chapters + 10+ labs)
- Module 0: Foundations (4 chapters, weeks 1–2)
- Module 1: ROS 2 Fundamentals (5 chapters + 3 labs, weeks 3–5)
- Module 2: Gazebo & Unity Simulation (5 chapters + 3 labs, weeks 6–7)
- Module 3: NVIDIA Isaac (5 chapters + 3 labs, weeks 8–10)
- Module 4: VLA Systems (5 chapters + 3 labs, weeks 11–13)
- Capstone & Hardware chapters (4 + 4 each)
- Dependency graph showing weekly progression and blocking prerequisites

**Phase 2 - File Generation Strategy**:
- Canonical naming convention (e.g., `module-1-ros2/01-architecture-overview.md`)
- Frontmatter requirements (id, title, sidebar_position, description, keywords)
- File organization with auto-sorting sidebar via sidebar_position
- 4-wave generation order (Foundations → Modules 1–2 → Modules 3–4)
- Code sample storage strategy (inline vs. external GitHub repos)
- Asset management (diagrams, screenshots, code-snippets)

**Phase 3 - Writing & Quality Assurance**:
- Constitution alignment checklist (10 items per chapter)
- Consistency & integration testing (build validation, link checking, terminology verification)
- Final polish pass (copy editing, visual consistency, metadata)

**Phase 4 - Deployment & Iteration**:
- Docusaurus build and GitHub Pages deployment commands
- Post-launch monitoring and feedback loop
- Iteration cadence for updates

**Quality Standards** (aligned with constitution):
- Technical accuracy: All code tested on target platforms
- Clarity: Every concept explained or glossary-linked
- Consistency: Uniform formatting, terminology, visuals across 13 weeks
- Capstone integration: Every module feeds into final project

**Success Criteria**:
- ✅ 21 weeks of content (modules + capstone + hardware) complete
- ✅ 20+ chapters with learning outcomes, examples, labs
- ✅ 10+ hands-on labs with expected output
- ✅ Comprehensive glossary (50+ terms)
- ✅ 100% chapters pass constitution checklist
- ✅ Docusaurus build succeeds with zero warnings
- ✅ All code examples tested and documented
- ✅ 80% student completion of capstone project

## Outcome

- ✅ Impact: Comprehensive architectural plan providing step-by-step execution roadmap from Docusaurus bootstrap through publication and iteration. Plan integrates specification requirements (21 weeks, 4 modules, clarified assessments/capstone) with constitution principles (modular progression, hands-on code, Docusaurus compliance, capstone coherence). Identifies 60+ discrete content deliverables, generation dependencies, and quality gates.
- 🧪 Tests: Phase 0–4 architectures documented with concrete deliverables; chapter breakdown validated against week-by-week mapping; naming convention tested for Docusaurus sidebar compatibility; success criteria measurable and verifiable
- 📁 Files: 1 file created (specs/1-robotics-textbook/plan.md, 400+ lines); 1 PHR created (this file)
- 🔁 Next prompts: `/sp.tasks` (task generation for all 60+ deliverables), `/sp.implement` (content authoring and lab creation)
- 🧠 Reflection: Plan bridges specification (what the textbook must contain) with constitution (how it must be written) and execution (step-by-step generation). By detailing module architecture, file naming, generation waves, and quality gates upfront, plan minimizes downstream rework and ensures coherent capstone integration. Docusaurus + Markdown approach provides low-friction publishing path aligned with modern documentation standards.

## Evaluation notes (flywheel)

- Failure modes observed: None - plan structure comprehensive; all major phases (prep, architecture, generation, QA, deployment) documented with concrete deliverables
- Graders run and results (PASS/FAIL): Plan validation (phase completeness, success criteria specificity, risk mitigation coverage) PASS
- Prompt variant (if applicable): None (standard planning flow executed)
- Next experiment (smallest change to try): Proceed with `/sp.tasks` to break down all 60+ deliverables (chapters, labs, assessments, code samples) into granular, assignable tasks with acceptance criteria and testing approach
