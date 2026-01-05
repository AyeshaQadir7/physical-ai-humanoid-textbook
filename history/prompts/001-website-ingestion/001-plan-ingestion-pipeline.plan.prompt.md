---
id: 001
title: Plan ingestion, embedding, and vector storage pipeline
stage: plan
date: 2025-12-25
surface: agent
model: claude-haiku-4.5
feature: 001-website-ingestion
branch: 001-website-ingestion
user: automated
command: /sp.plan --feature 001-website-ingestion
labels: ["architecture", "planning", "backend", "rag"]
links:
  spec: specs/001-website-ingestion/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-website-ingestion/plan.md
  - specs/001-website-ingestion/research.md
  - specs/001-website-ingestion/data-model.md
  - specs/001-website-ingestion/quickstart.md
  - specs/001-website-ingestion/contracts/openapi.yaml
tests:
  - (pending) Integration tests with live Cohere + Qdrant
  - (pending) Unit tests for each module
---

## Prompt

```text
Spec-1: Plan ingestion, embedding, and vector storage pipeline

Objective:
Set up a minimal backend pipeline to ingest a deployed Docusaurus book, generate embeddings, and store them in Qdrant Cloud.

Plan:
- Project setup
  - Create `backend/` directory
  - Initialize UV Python package inside `backend/`
  - Configure environment variables (Cohere API key, Qdrant URL, Qdrant API key, collection name)

- Backend structure
  - Create a single `main.py` file
  - Implement all ingestion, embedding, and storage logic in this file

- URL ingestion
  - Define base Vercel URL
  - Discover and collect all internal documentation URLs
  - Fetch and extract clean article text from each page

- Chunking
  - Chunk extracted text using fixed-size chunks with overlap
  - Attach metadata (URL, page title, section headers)

- Embedding
  - Generate embeddings using Cohere embedding model
  - Batch requests for efficiency and error handling

- Vector storage
  - Create or validate Qdrant collection
  - Store embeddings with metadata
  - Ensure idempotent inserts to avoid duplication

- Execution
  - Implement a single `main()` function
  - Run ingestion → chunking → embedding → Qdrant storage sequentially
```

## Response snapshot

### Phase 0: Research Complete

Conducted comprehensive research on all technical components:

1. **HTML Parsing** → BeautifulSoup 4 (static content extraction, 3-6x faster than Selenium/Playwright)
2. **Text Chunking** → LangChain RecursiveCharacterTextSplitter with tiktoken (token-aware, preserves semantics)
3. **Embedding Generation** → Cohere embed-english-v3.0 (1024D vectors, production API key required)
4. **Vector Storage** → Qdrant Cloud with content-hash deduplication (SHA256-based deterministic IDs)
5. **Configuration** → Environment variables + JSON checkpoint file for resume capability
6. **Error Handling** → Exponential backoff with jitter + structured logging

**Research Artifact**: `research.md` (7,800+ words with code examples, rationale, alternatives considered)

### Phase 1: Design Complete

Generated comprehensive design artifacts:

1. **Data Model** (`data-model.md`)
   - Page, Chunk, Embedding, Qdrant Point, Checkpoint, Ingestion Report
   - Entity relationships and validation rules
   - Example payloads and state transitions

2. **API Contracts** (`contracts/openapi.yaml`)
   - OpenAPI 3.0 specification
   - 7 endpoints: ingestion, collection management, search
   - Placeholder for Spec-2 (FastAPI integration)

3. **Quickstart Guide** (`quickstart.md`)
   - Step-by-step setup (Python 3.10+, venv, dependencies)
   - Configuration via `.env` file
   - Three execution modes (CLI, Python API, single file)
   - Example workflows and troubleshooting

4. **Updated Plan** (`plan.md`)
   - Technical context filled (Python 3.10+, BeautifulSoup, LangChain, Cohere, Qdrant)
   - Constitution check passed (no violations)
   - Project structure defined (modular backend/ directory)
   - Phase 0 & 1 completion documented
   - Success criteria mapped to implementation components

### Key Design Decisions

- **Token-based chunking**: 512 tokens per chunk with 50-token overlap (10%) ← optimal for educational content
- **Content-hash IDs**: SHA256(url + text) → UUID ← deterministic deduplication without extra queries
- **Cohere production key**: 1000 req/min vs 100 trial ← required for 50k+ chunks
- **Single-threaded CLI**: Sequential processing ← simplicity over parallelism (can optimize in Spec-2)
- **Checkpoint system**: JSON file with processed hashes ← enables resume without re-embedding

### Validation

✅ Constitution Check: PASS (no violations; all principles maintained)
✅ All NEEDS CLARIFICATION resolved (technology stack complete)
✅ Technical context filled (dependencies, constraints, performance goals documented)
✅ Project structure designed (backend/ layout with modular components)
✅ Phase 0 & 1 complete (no blockers for Phase 2 implementation)

## Outcome

- ✅ Impact: Architecture and design finalized; ready for Phase 2 implementation (task generation + coding)
- 🧪 Tests: Integration tests planned for live Cohere + Qdrant; unit tests per-module
- 📁 Files: 5 new design artifacts created (research.md, data-model.md, quickstart.md, openapi.yaml, updated plan.md)
- 🔁 Next prompts: Run `/sp.tasks` to generate implementation tasks from design artifacts
- 🧠 Reflection: Comprehensive design enables straightforward implementation; all decisions documented with rationale and alternatives

## Evaluation notes (flywheel)

- Failure modes observed: None; all research tasks completed successfully with comprehensive findings
- Graders run and results (PASS/FAIL): Constitution check = PASS; Phase 0 research = PASS; Phase 1 design = PASS
- Prompt variant (if applicable): N/A (single planning workflow)
- Next experiment: Implement Phase 2 using generated tasks; validate against spec acceptance criteria (SC-001 through SC-012)
