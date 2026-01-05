---
id: 004
title: Architectural Decisions - ADR Documentation
stage: adr
date: 2025-12-10
surface: agent
model: claude-haiku-4-5-20251001
feature: 1-robotics-textbook
branch: 1-robotics-textbook
user: system
command: /sp.adr
labels: ["adr", "architecture", "decisions", "publishing", "hardware", "pedagogy"]
links:
  spec: specs/1-robotics-textbook/spec.md
  plan: specs/1-robotics-textbook/plan.md
  ticket: null
  pr: null
files:
  - history/adr/001-docusaurus-markdown-github-pages.md
  - history/adr/002-simulation-first-hardware-optional.md
  - history/adr/003-modular-progressive-capstone-integrated.md
tests:
  - ADR-001: Documentation Stack (PASS - clustered: Docusaurus + Markdown + GitHub Pages)
  - ADR-002: Hardware Strategy (PASS - clustered: Simulation-first + 3-tier options)
  - ADR-003: Content Architecture (PASS - clustered: Modular + Progressive + Capstone-integrated)
  - Significance test: All 3 ADRs pass (impact on architecture, alternatives considered, revisit risk)
---

## Prompt

Analyze planning artifacts and document architecturally significant decisions as ADRs.

Extract decision clusters from plan.md (specifications, design patterns, technology choices). For each decision:
- Identify why chosen vs. alternatives
- Document tradeoffs and consequences
- Create Architecture Decision Record (ADR) for team alignment

Focus on decisions that:
- Impact how engineers implement the textbook
- Have notable tradeoffs (cost, accessibility, maintainability)
- Will likely be questioned or revisited

## Response snapshot

Analyzed plan.md (400+ lines) and spec.md against clarifications. Extracted 3 architecturally significant decision clusters:

**ADR-001: Documentation Stack (Docusaurus + Markdown + GitHub Pages)**
- Decision: Publish textbook as static Docusaurus site hosted on GitHub Pages
- Rationale: Optimized for technical documentation; Markdown is version-control-friendly; GitHub Pages is free and integrated
- Alternatives: WordPress (overkill), LaTeX+PDF (fragmented), Notion (not version-controlled), custom Next.js (overcomplicated)
- Consequences: Fast iteration, quality gates via Git, searchable; but limited interactivity, GitHub Pages limitations
- Status: Accepted

**ADR-002: Hardware Accessibility Strategy (Simulation-First, 3 Optional Tiers)**
- Decision: Tier 1 (Simulator-only, $0) → Tier 2 (Jetson Nano + RealSense, ~$700) → Tier 3 (Unitree G1, ~$16k)
- Rationale: Makes course accessible to all; simulation is legitimate learning path; hardware is optional, not required
- Alternatives: Mandate physical robots (inequitable), simulator-only (misses embodiment), custom hardware (expensive)
- Consequences: 80% completion achievable; scalable institutions; but student perception issues ("is sim cheating?"), fragmentation risk
- Status: Accepted

**ADR-003: Content Architecture (Modular Progressive Design with Capstone Integration)**
- Decision: 5 modules (Foundations, ROS 2, Simulation, Isaac, VLA) with strict progression; each module feeds output to capstone
- Rationale: Clear prerequisites; cognitive load management; capstone coherence; aligns with industry skill domains
- Alternatives: Non-sequential modules (confusing prerequisites), siloed examples (disconnected capstone)
- Consequences: 80% completion realistic; coherent narrative; but initial authoring overhead, less flexibility
- Status: Accepted

## Outcome

- ✅ Impact: 3 decision clusters documented as formal ADRs with full context, rationale, alternatives, consequences, and risk mitigations. ADRs align planning decisions with specification requirements (accessibility, capstone integration, modular design) and ensure team alignment before implementation.
- 🧪 Tests: All 3 ADRs pass significance test (impact on architecture, alternatives documented, revisit risk high); clustered (not atomic); consequences cover positive + negative outcomes
- 📁 Files: 3 ADRs created (001, 002, 003); 1 PHR created (this file)
- 🔁 Next prompts: `/sp.tasks` (task breakdown for all 60+ deliverables), `/sp.implement` (content authoring and lab creation)
- 🧠 Reflection: ADRs capture "why" behind planning decisions upfront. Docusaurus choice is low-risk (static site, portability); hardware strategy is inclusive (simulation-first with options); content architecture ensures capstone coherence. All three decisions are independent (can be evaluated separately) but mutually reinforcing (together solve publication, accessibility, and pedagogy challenges).

## Evaluation notes (flywheel)

- Failure modes observed: None - all 3 ADRs pass clustering check (no atomic/trivial decisions); all document alternatives and tradeoffs
- Graders run and results (PASS/FAIL): ADR significance grading PASS (all 3 impact architecture, have alternatives, revisit risk)
- Prompt variant (if applicable): None (standard ADR analysis flow executed)
- Next experiment (smallest change to try): Proceed with `/sp.tasks` to break down 60+ deliverables (chapters, labs, assessments, code samples) into granular, assignable tasks with acceptance criteria; ADRs provide architectural guardrails for task decomposition
