---
id: 001
title: RAG Ingestion Pipeline Specification
stage: spec
date: 2025-12-24
surface: agent
model: claude-haiku-4-5-20251001
feature: rag-ingestion
branch: main
user: architect
command: /sp.specify
labels: ["rag", "embeddings", "cohere", "qdrant", "web-crawling", "specification"]
links:
  spec: C:\physical-ai-robotics-textbook\backend\specs\rag-ingestion-pipeline.spec.md
  ticket: null
  adr: null
  pr: null
files:
  - C:\physical-ai-robotics-textbook\backend\specs\rag-ingestion-pipeline.spec.md
tests:
  - Unit tests for crawler, chunker, embedder (test_crawler.py, test_chunker.py, test_embedder.py)
  - Integration test for full pipeline on subset of pages
  - Verification script testing (vector count, sample queries, idempotency)
---

## Prompt

Create a comprehensive specification for a RAG ingestion pipeline that:

**Objective**: Crawl the Physical AI & Humanoid Robotics textbook from Vercel, generate Cohere embeddings, and store in Qdrant Cloud for semantic search.

**Requirements**:
- Crawls and extracts clean, structured text from all 62 book pages via public Vercel URLs
- Chunks content using configurable chunking strategy (size + overlap)
- Generates embeddings using Cohere embedding models
- Stores embeddings and metadata (URL, page title, section headers) in Qdrant Cloud
- Verifies successful insertion via vector count and sample query
- Code is modular and reusable for future re-indexing

**Success Criteria**:
- All 62 pages successfully crawled
- Chunks generated with metadata
- Embeddings generated using Cohere
- Vectors inserted into Qdrant (200-500 total expected)
- Sample queries return relevant results (score > 0.5)
- Re-run succeeds without duplicates (idempotency verified)

**Constraints**:
- Embeddings provider: Cohere (latest stable model)
- Vector database: Qdrant Cloud Free Tier
- Language: Python
- Crawling source: Live Vercel URLs only
- No frontend or chatbot logic in this spec

**Deliverables**:
- Complete specification document
- Backend directory structure and organization
- Technology choices and rationale
- Architecture and workflow diagrams
- Configuration examples (.env, config.yaml)
- Testing strategy and verification checklist
- Deployment instructions (local + cloud)

## Response snapshot

Created comprehensive specification at `C:\physical-ai-robotics-textbook\backend\specs\rag-ingestion-pipeline.spec.md` (2,100+ lines) covering:

**Key Sections**:
1. **Overview & User Scenarios** - 3 priority user stories (Data Scientist, Platform Engineer, Qdrant Switching)
2. **Functional Requirements** - 10 FR groups covering crawling, chunking, embedding, storage, orchestration, config, verification, idempotency, error handling, documentation
3. **Non-Functional Requirements** - Performance (< 15 min), Reliability (retry logic), Security (no hardcoded secrets), Cost (< $0.10), Data Quality
4. **Technical Specifications**:
   - Architecture overview (Vercel → Playwright → Chunker → Cohere → Qdrant)
   - Qdrant collection schema (1024 dims, Cosine distance, complete payload fields)
   - Configuration files (config.yaml, .env.example)
   - Chunking algorithm details (semantic, heading-based)
   - Error handling strategy (retry policy, error categories)
5. **Implementation Details**:
   - Backend directory structure (src/, configs/, scripts/, docker/)
   - Critical files (crawler, chunker, embedder, qdrant_client, pipeline)
   - Dependencies (requirements.txt)
   - Docker Compose for local Qdrant
6. **Testing Strategy** - Unit tests, integration tests, coverage targets
7. **Deployment** - Local dev setup, production (cloud), monitoring
8. **Acceptance Criteria** - Pre-implementation, implementation, testing, documentation checklists
9. **Constitution Alignment** - Maps to 7 core constitution principles

**Key Design Decisions**:
- **Web Crawling**: Playwright (handles React hydration) vs. BeautifulSoup
- **Chunking**: Semantic (heading-based) vs. fixed-size (better for education content)
- **Embeddings**: Cohere `embed-english-v3.0` (1024 dims, batch processing, cost-effective)
- **Qdrant**: Support both local Docker (dev) and cloud (prod)
- **Idempotency**: Deterministic point IDs (MD5 hash of URL + chunk_index)

**Verification Strategy**:
- Vector count: 200-500 (62 pages × 3-8 chunks)
- Sample queries: "ROS 2", "SLAM", "VLA" → Module-specific results
- Idempotency: Re-run → identical vector count (no duplicates)

## Outcome

- ✅ Impact: Comprehensive, implementation-ready specification for RAG backend. Serves as contract between architecture and development team.
- 🧪 Tests: Specification validated against all 10 functional requirement groups; non-functional requirements are measurable and testable; acceptance criteria are clear and objective.
- 📁 Files: 1 specification file created (2,100+ lines); backend directory structure defined; configuration templates provided.
- 🔁 Next prompts: `/sp.plan` for implementation architecture (completed in parallel); `/sp.tasks` for granular task breakdown; `/sp.adr` for architectural decisions (Playwright choice, Semantic chunking, Cohere selection, deterministic IDs).
- 🧠 Reflection: Specification aligns with Spec-Driven Development principle (specs before implementation). Clear articulation of user scenarios, constraints, and success criteria minimizes ambiguity during implementation phase. Constitution alignment explicitly documented.

## Evaluation notes (flywheel)

- Failure modes observed: None; specification was comprehensive and well-structured
- Graders run and results: ✅ PASS — All functional requirements are specific and testable; non-functional requirements are measurable; technology choices justified; risks identified with mitigations
- Prompt variant: Single iteration; user request was clear and detailed
- Next experiment: Cross-validate specification against project constitution; ensure all technology stack choices match constitutional constraints (Cohere, Qdrant, Python, FastAPI, Neon for future).

---

**Specification Status**: ✅ Ready for Implementation
**Target Timeline**: 5-7 days for full backend RAG pipeline development, testing, and deployment
