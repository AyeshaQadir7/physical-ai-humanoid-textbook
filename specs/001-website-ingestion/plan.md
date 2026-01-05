# Implementation Plan: Website Ingestion & Vector Storage Pipeline

**Branch**: `001-website-ingestion` | **Date**: 2025-12-25 | **Spec**: `specs/001-website-ingestion/spec.md`
**Input**: Feature specification from `/specs/001-website-ingestion/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implement a minimal Python backend pipeline to ingest a live Docusaurus textbook from Vercel, extract clean article text, chunk it into semantically-meaningful segments, generate embeddings via Cohere API, and store vectors + metadata in Qdrant Cloud. The pipeline is single-threaded, CLI-driven, and supports re-runs without duplication.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: requests (HTML fetching), BeautifulSoup4 (HTML parsing), cohere (embeddings), qdrant-client (vector storage), python-dotenv (env config)
**Storage**: Qdrant Cloud (vector + metadata); local JSON checkpoint file for resume capability
**Testing**: pytest for unit tests; integration tests against live Qdrant collection
**Target Platform**: Linux/macOS/Windows CLI (standalone script)
**Project Type**: Single Python CLI application
**Performance Goals**: Ingest 50-page book (~50k chunks) in <30 minutes end-to-end
**Constraints**: ≥99% embedding success rate; idempotent inserts (no duplicate vectors); handle API rate limits gracefully
**Scale/Scope**: Support textbook URLs with 10-100+ pages; checkpoint/resume on failure

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

**Constitution Principles Alignment** (from `.specify/memory/constitution.md`):

| Principle | Compliance | Notes |
|-----------|-----------|-------|
| **I. Book as Authoritative Source** | ✅ PASS | Ingestion extracts directly from textbook; no external knowledge |
| **II. RAG Mandate** | ✅ PASS | Embeddings + Qdrant provide RAG foundation for future chatbot |
| **III. Traceability** | ✅ PASS | Metadata includes URL, title, headers for source attribution |
| **IV. Strict Scope Boundaries** | ✅ PASS | Ingests only textbook content (Vercel-hosted); no external crawling |
| **V. Technical Accuracy** | ✅ PASS | Extracts live published content; schema supports metadata |
| **VI. AI-Native Content Structure** | ✅ PASS | Chunking preserves hierarchy; metadata enables structured retrieval |
| **VII. Spec-Driven Development** | ✅ PASS | Implementation follows this spec; changes require spec amendment |
| **Tech Stack** | ✅ PASS | Python backend + Qdrant align with constitution (FastAPI-ready for future) |
| **Code Standards** | ✅ PASS | No hardcoded secrets; environment variables for API keys/URLs |

**Gate Status**: ✅ **PASS** - No violations. Proceed to Phase 0 research.

## Project Structure

### Documentation (this feature)

```text
specs/001-website-ingestion/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command) [TBD]
├── data-model.md        # Phase 1 output (/sp.plan command) [TBD]
├── quickstart.md        # Phase 1 output (/sp.plan command) [TBD]
├── contracts/           # Phase 1 output (/sp.plan command) [TBD]
│   └── openapi.yaml     # API contracts for future FastAPI wrapper
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

**Structure Decision**: **Option 1 - Single Python CLI Project**

This feature is a standalone CLI application (not yet a web API), so a lightweight single-project structure is appropriate. Once the CLI is stable, it can be wrapped in FastAPI for Spec-2 (RAG Chatbot feature).

```text
backend/                          # NEW: Backend root directory
├── ingestion/                    # Ingestion pipeline package
│   ├── __init__.py
│   ├── main.py                   # Entry point: ingestion orchestration
│   ├── crawler.py                # URL crawling and HTML extraction
│   ├── chunker.py                # Text chunking logic
│   ├── embedder.py               # Cohere embedding generation
│   ├── qdrant_storage.py         # Qdrant insert/verification
│   ├── checkpoint.py             # Resume checkpoint management
│   └── config.py                 # Environment variable parsing
├── requirements.txt              # Python dependencies
├── setup.py                      # Package metadata
├── README.md                      # Setup and usage instructions
└── tests/
    ├── unit/                     # Unit tests for each module
    │   ├── test_chunker.py
    │   ├── test_embedder.py
    │   └── test_qdrant_storage.py
    └── integration/              # Integration tests with live services
        └── test_full_pipeline.py
```

**Rationale**:
- **Single package**: All ingestion logic in one `ingestion/` module for simplicity
- **Single entry point**: `main.py` orchestrates the full pipeline
- **Modular functions**: Each step (crawl → chunk → embed → store) in separate files
- **Test structure**: Unit tests for components; integration tests for full pipeline
- **Minimal**: No database, no API server (that's future Spec-2 with FastAPI)
- **Future-ready**: Easily wraps into FastAPI routes when needed

## Complexity Tracking

**Status**: ✅ No violations to track - Constitution Check passed with no justified exceptions needed.

---

## Phase 0 & 1 Completion

### ✅ Phase 0: Research Complete

All NEEDS CLARIFICATION items resolved:

1. **HTML Parsing** → BeautifulSoup 4 (static content extraction from Docusaurus)
2. **Text Chunking** → LangChain RecursiveCharacterTextSplitter with tiktoken (512 tokens, 10% overlap)
3. **Embedding Generation** → Cohere embed-english-v3.0 (1024D, production API key required)
4. **Vector Storage** → Qdrant Cloud with content-hash deduplication (SHA256-based IDs)
5. **Configuration** → Environment variables + JSON checkpoint file
6. **Error Handling** → Exponential backoff with jitter + structured logging

**Artifact**: `research.md` - Complete technology stack with rationale and alternatives considered

---

### ✅ Phase 1: Design Complete

#### 1. Data Model (`data-model.md`)

Core entities defined:
- **Page**: Single URL with extracted text and section headers
- **Chunk**: Text segment (512 tokens) with metadata (URL, title, headers)
- **Embedding**: Vector representation (1024-dimensional) via Cohere
- **Qdrant Point**: Combined vector + metadata payload for retrieval
- **Checkpoint**: Progress tracking for resume capability
- **Ingestion Report**: Final summary (stats, errors, verification)

#### 2. API Contracts (`contracts/openapi.yaml`)

OpenAPI 3.0 specification (placeholder for FastAPI integration):
- `POST /ingestion/run` - Start ingestion pipeline
- `GET /ingestion/{run_id}/status` - Check progress
- `GET /ingestion/{run_id}/report` - Get final report
- `GET /collections` - List all collections
- `GET /collections/{name}` - Collection statistics
- `DELETE /collections/{name}` - Delete collection
- `POST /search` - Vector similarity search

**Note**: Spec-1 (CLI-only) - FastAPI implemented in Spec-2 (RAG Chatbot)

#### 3. Quickstart Guide (`quickstart.md`)

Step-by-step setup and usage:
- Environment setup (Python 3.10+, virtual environment)
- Dependency installation (BeautifulSoup, LangChain, Cohere, Qdrant, etc.)
- Configuration via `.env` file
- Three execution modes (single file, CLI, Python API)
- Example workflow (test → verify → full book)
- Common tasks (monitor, resume, test, troubleshoot)
- Performance expectations and optimization



---

### Re-evaluation: Constitution Check (Post-Design)

**Gate**: Constitution principles still aligned after detailed design?

✅ **Result**: PASS - All principles maintained:

| Principle | Status | Notes |
|-----------|--------|-------|
| **Book as Authoritative Source** | ✅ | Ingest only from Vercel textbook URL |
| **RAG Mandate** | ✅ | Qdrant vectors enable future RAG chatbot |
| **Traceability** | ✅ | Metadata tracks URL, headers, chunk position |
| **Scope Boundaries** | ✅ | No external crawling; textbook content only |
| **Technical Accuracy** | ✅ | Live content extraction preserves structure |
| **AI-Native Structure** | ✅ | Token-based chunking + semantic embeddings |
| **Spec-Driven Development** | ✅ | Implementation follows detailed spec |
| **Tech Stack** | ✅ | Python + Qdrant align with constitution |
| **Code Standards** | ✅ | Environment variables for secrets; no hardcoding |

---

## Next Steps: Phase 2 (Implementation)

### Artifacts to Generate

1. **`tasks.md`** - Dependency-ordered implementation tasks (via `/sp.tasks` command)
2. **Modular Python implementation**:
   - `crawler.py`: URL discovery + text extraction
   - `chunker.py`: Token-based splitting
   - `embedder.py`: Cohere batch embedding
   - `qdrant_storage.py`: Vector insertion + verification
   - `checkpoint.py`: Progress tracking
   - `config.py`: Configuration loading
   - `main.py`: Orchestration
3. **Unit & Integration Tests**: pytest with live API testing
4. **README.md**: Detailed setup + troubleshooting guide

### Success Criteria (from Spec)

| Criterion | Acceptance | Implementation |
|-----------|-----------|-----------------|
| **SC-001** | 100% page coverage | Crawler discovers all internal links |
| **SC-002** | Structure preserved | Boilerplate removed; headers intact |
| **SC-003** | ±10% chunk tolerance | Validation in chunker.py |
| **SC-004** | ≥99% embedding success | Retry logic in embedder.py |
| **SC-005** | ≥99% insertion success | Upsert + deduplication in storage |
| **SC-006** | Vector count ±1% match | Verification in main.py report |
| **SC-007** | Search returns metadata | Qdrant query filtering validation |
| **SC-008** | No duplicate vectors | Content hash-based ID deduplication |
| **SC-009** | Complete without manual intervention | Full e2e automation |
| **SC-010** | Configuration via env vars | Config.py environment loading |
| **SC-011** | <30 min for 50 pages | Performance target (batch processing) |
| **SC-012** | Final report with stats | IngestionReport class generation |

---

## Architecture Decision Summary

### Key Decisions Made (ADR candidates if cross-team impact)

1. **Token-based chunking (512 tokens, 10% overlap)**
   - Decision: RecursiveCharacterTextSplitter + tiktoken
   - Trade-off: Semantic coherence vs. simplicity
   - Alternative: Character-based or sentence-based chunking

2. **Content-hash IDs for deduplication**
   - Decision: SHA256(url + text) → UUID
   - Trade-off: Deterministic deduplication vs. custom tracking
   - Alternative: Separate deduplication queries

3. **Checkpoint system for resume capability**
   - Decision: JSON file tracking processed hashes
   - Trade-off: Simple local state vs. distributed checkpointing
   - Alternative: Database-backed checkpointing (over-engineered for Spec-1)

4. **Cohere embed-english-v3.0 (production API key)**
   - Decision: 1024D embeddings with 1000 req/min rate limit
   - Trade-off: Quality vs. cost/speed
   - Alternative: Lighter model (384D) if performance critical

5. **Single-threaded CLI (no parallelism)**
   - Decision: Sequential URL processing → batched embedding → Qdrant insert
   - Trade-off: Simplicity vs. throughput
   - Alternative: Async/parallel URL crawling (future optimization)

**ADR Recommendation**: These are implementation-level trade-offs appropriate for detailed design review but not currently cross-team-blocking decisions. Escalate to ADR only if team concerns arise during Spec-2 (RAG Chatbot) integration planning.

---

## Summary Table: Phase 0 & 1 Artifacts

| Artifact | Location | Status | Key Content |
|----------|----------|--------|-------------|
| **research.md** | `specs/001-website-ingestion/research.md` | ✅ Complete | Technology stack, rationale, code patterns |
| **data-model.md** | `specs/001-website-ingestion/data-model.md` | ✅ Complete | Entity definitions, relationships, validation rules |
| **quickstart.md** | `specs/001-website-ingestion/quickstart.md` | ✅ Complete | Setup, configuration, usage examples, troubleshooting |
| **openapi.yaml** | `specs/001-website-ingestion/contracts/openapi.yaml` | ✅ Complete | REST API contracts (Spec-2 placeholder) |
| **plan.md** | `specs/001-website-ingestion/plan.md` | ✅ Complete | Architecture, structure, decisions (this file) |
| **spec.md** | `specs/001-website-ingestion/spec.md` | ✅ Complete | Requirements, acceptance criteria, edge cases |

**Ready for**: Phase 2 implementation via `/sp.tasks` command and `/sp.implement` workflow

---

## Validation Checklist

- ✅ Constitution aligned (no violations)
- ✅ Technical context filled (language, dependencies, platform, constraints)
- ✅ Project structure documented (backend/ directory layout)
- ✅ All research complete (technology decisions made)
- ✅ Data model comprehensive (entities, relationships, validation)
- ✅ API contracts defined (future FastAPI integration)
- ✅ Quickstart guide provided (setup + usage)
- ✅ Success criteria traceable (SC-001 through SC-012)
- ✅ No unresolved technical unknowns
- ✅ Ready for implementation phase

---

**Status**: 🟢 **PHASE 1 COMPLETE** - Ready to proceed to Phase 2 (Task Generation & Implementation)
