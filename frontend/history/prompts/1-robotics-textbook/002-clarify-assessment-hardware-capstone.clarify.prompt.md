---
id: 002
title: Clarify Assessment, Hardware, and Capstone Scope
stage: clarify
date: 2025-12-10
surface: agent
model: claude-haiku-4-5-20251001
feature: 1-robotics-textbook
branch: 1-robotics-textbook
user: system
command: /sp.clarify
labels: ["clarification", "assessment", "hardware", "capstone", "rubrics"]
links:
  spec: specs/1-robotics-textbook/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/1-robotics-textbook/spec.md
tests:
  - Q1: Assessment grading scale (PASS - 5-point Likert with 40/40/20 weighting)
  - Q2: Hardware platform specificity (PASS - hardware-agnostic with Unitree G1 reference)
  - Q3: Capstone project scope (PASS - Code + Video + Report deliverables)
  - Specification updated with clarifications (PASS)
---

## Prompt

Perform comprehensive clarification analysis on the Physical AI & Humanoid Robotics textbook specification to identify and resolve ambiguities before planning phase. Focus on material areas affecting implementation strategy, architecture, and acceptance testing.

Areas requiring clarification:
1. Assessment grading scale and rubric weighting
2. Hardware platform specificity (generic vs. reference platform)
3. Capstone project scope and deliverables definition

## Response snapshot

Conducted structured ambiguity scan using specification taxonomy (Functional Scope, Domain Model, Integration, Edge Cases, Constraints, Completion Signals, Terminology, Misc). Identified three high-impact clarification areas. Presented sequential questioning (3 questions total) with recommended options and acceptance of all three recommendations:

**Q1 - Assessment Grading Scale**: Recommended 5-point Likert (5=Exceptional → 1=Incomplete) with 40% labs + 40% capstone + 20% quizzes. **ACCEPTED**.

**Q2 - Hardware Platform Specificity**: Recommended hardware-agnostic approach with Unitree G1 as optional reference implementation; simulation-first capstone with optional physical deployment. **ACCEPTED**.

**Q3 - Capstone Project Scope**: Recommended three-component deliverables (code + video + report) with clear minimum viable and exceptional criteria. **ACCEPTED**.

Updated specification (FR-011, FR-012, FR-016, FR-017, FR-018, SC-003, SC-004, Assumptions 3, 6, 7) to embed all clarifications with specific, testable requirements.

## Outcome

- ✅ Impact: Three critical ambiguities resolved with specific, measurable requirements embedded in specification. Assessment strategy now includes precise rubric scale (5-point Likert) and grading weights (20/40/40). Hardware path clarified (simulation-first with optional reference platform). Capstone deliverables explicitly defined (code, video, report) with minimum viable and exceptional criteria.
- 🧪 Tests: 3 clarification questions asked and answered; all accepted user recommendations; specification validation passed post-integration
- 📁 Files: 1 file modified (specs/1-robotics-textbook/spec.md); 1 PHR created (this file)
- 🔁 Next prompts: `/sp.plan` (architecture and week-by-week structure), `/sp.tasks` (task breakdown with acceptance criteria)
- 🧠 Reflection: Clarification resolved specification ambiguity around grading consistency, hardware accessibility, and capstone validation criteria—all areas that could have caused downstream rework if left unspecified during planning or implementation. Specification now provides sufficient detail for task generation and implementation without requiring circular clarifications.

## Evaluation notes (flywheel)

- Failure modes observed: None - all three clarifications accepted efficiently; no follow-up disambiguation required
- Graders run and results (PASS/FAIL): Specification validation checklist updated (all sections reflecting clarifications PASS)
- Prompt variant (if applicable): None (standard clarify flow executed)
- Next experiment (smallest change to try): Proceed with `/sp.plan` to architect week-by-week content structure and identify specific code sample requirements now that assessment/hardware/capstone criteria are defined
