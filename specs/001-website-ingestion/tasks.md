# Tasks: Website Ingestion & Vector Storage Pipeline

**Feature**: `001-website-ingestion`
**Branch**: `001-website-ingestion`
**Status**: Phase-based implementation, MVP scope = US1 + US2 + US3
**Generated**: 2025-12-25

---

## Quick Reference

- **Total Tasks**: 23 (excluding Phase 4: Polish)
- **User Stories**: 4 (3 P1, 1 P2)
- **MVP Scope**: US1 + US2 + US3 (18 tasks)
- **Parallel Opportunities**: 9 tasks can run in parallel after dependencies complete

---

## Phase 1: Setup (Prerequisites)

All tasks must complete before user story implementation.

- [x] T001 Create `backend/` directory and Python package structure
- [x] T002 Create `requirements.txt` with all dependencies (BeautifulSoup4, LangChain, Cohere, Qdrant, pytest)
- [x] T003 Create `.env` template with placeholder API keys and default configuration
- [x] T004 Create `backend/__init__.py` and module package structure (ingestion/)
- [x] T005 Create `backend/ingestion/config.py` to load env vars with validation
- [x] T006 Verify Python 3.10+ environment and all dependencies install correctly

---

## Phase 2: Foundational (Blocking Prerequisites)

Tasks that all user stories depend on. Complete before US1.

- [x] T007 Implement `backend/ingestion/checkpoint.py`: CheckpointManager class for resume capability
- [x] T008 [P] Implement `backend/ingestion/chunker.py`: TextChunker class with token-based splitting (512 tokens, 10% overlap)
- [x] T009 [P] Implement `backend/ingestion/qdrant_storage.py`: QdrantManager class for collection setup, insertion, verification
- [x] T010 [P] Implement `backend/ingestion/embedder.py`: CohereEmbedder class with batching and retry logic
- [x] T011 Create `backend/tests/unit/` directory with pytest fixtures for mocking Cohere and Qdrant

---

## Phase 3a: User Story 1 - Crawl and Index Book Content (P1)

Extract, chunk, and prepare content for embedding generation.

- [x] T012 [US1] Implement `backend/ingestion/crawler.py`: URLCrawler class to discover and fetch URLs from Vercel
- [x] T013 [US1] Implement HTML parsing in crawler: extract `<article>` tag, remove boilerplate (nav, footer, script)
- [x] T014 [US1] Implement text extraction with UTF-8 encoding and special character handling in crawler
- [x] T015 [US1] Extract section headers from HTML hierarchy; attach as metadata to chunks in chunker
- [x] T016 [US1] Implement chunk deduplication in chunker: hash-based detection before embedding
- [x] T017 [US1] Create `backend/ingestion/main.py`: Orchestrator function (crawl → chunk → checkpoint)
- [x] T018 [US1] Implement CLI entry point: `python -m ingestion.main --base-url <url>`
- [x] T019 [US1] Write `backend/tests/unit/test_crawler.py`: Crawler extraction and error handling
- [x] T020 [US1] Write `backend/tests/unit/test_chunker.py`: Token count validation, metadata attachment
- [x] T021 [US1] Write `backend/tests/integration/test_crawl_and_chunk.py`: End-to-end test with live Vercel URL (sample page)

---

## Phase 3b: User Story 2 - Generate and Store Embeddings (P1)

Embed chunks and store vectors in Qdrant with idempotent inserts.

- [x] T022 [US2] Integrate CohereEmbedder in main.py: batch chunks, generate embeddings with retry logic
- [x] T023 [US2] Integrate QdrantManager in main.py: create/validate collection, upsert vectors with metadata
- [x] T024 [US2] Implement idempotent insertion logic: content-hash IDs prevent duplicate vectors
- [x] T025 [US2] Implement rate limit handling: exponential backoff (1s, 2s, 4s...) on TooManyRequestsError
- [x] T026 [US2] Add error logging: capture failed chunks with chunk_id and error details
- [x] T027 [US2] Implement checkpoint save after each batch to enable resume
- [x] T028 [US2] Write `backend/tests/unit/test_embedder.py`: Batching, retry logic, rate limit handling
- [x] T029 [US2] Write `backend/tests/unit/test_qdrant_storage.py`: Collection setup, upsert, deduplication
- [x] T030 [US2] Write `backend/tests/integration/test_embed_and_store.py`: End-to-end with live Cohere + Qdrant (10 chunks)

---

## Phase 3c: User Story 3 - Verify Indexing Success (P1)

Validate data integrity and report final statistics.

- [x] T031 [US3] Implement verification in main.py: check vector count vs chunk count (±1% tolerance)
- [x] T032 [US3] Implement metadata queryability check: verify URL and section headers are filterable in Qdrant
- [x] T033 [US3] Implement similarity search validation: sample 5 random chunks, verify metadata in results
- [x] T034 [US3] Create IngestionReport class: generate JSON report with stats, errors, verification results
- [x] T035 [US3] Log final report to console and file: JSON-formatted for machine parsing
- [x] T036 [US3] Write `backend/tests/unit/test_report_generation.py`: Report schema and stat calculations
- [x] T037 [US3] Write `backend/tests/integration/test_full_pipeline.py`: Complete ingestion + verification (1 page)

---

## Phase 3d: User Story 4 - Configure and Re-index (P2)

Support iterative updates with environment variable configuration.

- [x] T038 [US4] Extend config.py: CHUNK_SIZE, CHUNK_OVERLAP, BATCH_SIZE as env vars
- [x] T039 [US4] Implement skip logic in main.py: skip already-processed URLs (via checkpoint)
- [x] T040 [US4] Clear checkpoint on demand: CLI flag `--clear-checkpoint` for fresh runs
- [x] T041 [US4] Validate collection schema on startup: halt with error if mismatch detected
- [x] T042 [US4] Write `backend/tests/integration/test_reconfiguration.py`: Re-run with different chunking params

---

## Phase 4: Polish & Cross-Cutting (After MVP)

Only after US1 + US2 + US3 passing with live APIs.

- [x] T043 Create `backend/README.md`: Setup, configuration, usage, troubleshooting
- [x] T044 Add logging configuration: INFO level by default, DEBUG via `--log-level` flag
- [x] T045 Create `.gitignore`: exclude `.env`, `__pycache__`, `.pytest_cache`, `ingestion_checkpoint.json`
- [x] T046 Add type hints to all modules (Python 3.10+)
- [x] T047 Run full test suite with coverage: aim for ≥80% coverage
- [x] T048 Performance profile: measure crawl, chunk, embed, insert times for typical 50-page book

---

## Dependency Graph

```
Setup (T001-T006)
    ↓
Foundational (T007-T011)
    ├→ US1 (T012-T021) [independent implementation & testing]
    ├→ US2 (T022-T030) [depends on US1: needs chunks]
    ├→ US3 (T031-T037) [depends on US2: needs vectors in Qdrant]
    └→ US4 (T038-T042) [depends on US1: config reuse]
        ↓
Polish (T043-T048)
```

---

## Parallel Execution Opportunities

**After Phase 2 complete**, these tasks can run in parallel:

- T012 (Crawler) + T007 (Checkpoint) — independent modules
- T013, T014, T015, T016 — all crawler/chunker sub-tasks (single dependency on crawler.py)
- T019, T020 — unit tests for US1 components
- T022, T023 — once US1 chunks available

**After Phase 3a complete**:
- T022, T023 — start embedding/storage (depend on chunks)
- T028, T029 — unit tests for US2 components

**After Phase 3b complete**:
- T031-T037 — verification phase (all depend on US2 vectors)

---

## Independent Test Criteria (Per User Story)

### US1: Crawl and Index Book Content
**Scope**: Crawler + Chunker + Checkpoint
**Input**: Live Vercel URL (e.g., https://physical-ai-robotics.vercel.app)
**Test**: `pytest backend/tests/integration/test_crawl_and_chunk.py`
**Success**:
- ≥2 pages crawled
- ≥10 chunks created per page
- Chunks contain URL, title, headers metadata
- No boilerplate content (nav/footer removed)

### US2: Generate and Store Embeddings
**Scope**: Embedder + QdrantManager + Orchestration
**Input**: Pre-chunked content (10 chunks from US1)
**Test**: `pytest backend/tests/integration/test_embed_and_store.py`
**Success**:
- 10 embeddings generated (1024D vectors)
- 10 points inserted into Qdrant
- Metadata queryable by URL
- No duplicate vectors (idempotent)

### US3: Verify Indexing Success
**Scope**: Verification + Reporting
**Input**: Populated Qdrant collection (from US2)
**Test**: `pytest backend/tests/integration/test_full_pipeline.py`
**Success**:
- Vector count = chunk count
- Similarity search returns top-3 results with correct metadata
- Report generated with stats and verification results

### US4: Configure and Re-index
**Scope**: Configuration + Resume
**Input**: Env vars + existing checkpoint
**Test**: `pytest backend/tests/integration/test_reconfiguration.py`
**Success**:
- New chunks (different CHUNK_SIZE) embedded without re-processing old chunks
- Collection size increases; no duplicates

---

## MVP Scope & Execution Strategy

### MVP = US1 + US2 + US3 (18 tasks)
Complete this first to have a working end-to-end pipeline.

**Execution order**:
1. Setup (T001-T006): ~30 min
2. Foundational (T007-T011): ~2 hours
3. US1 (T012-T021): ~3 hours [includes tests]
4. US2 (T022-T030): ~2 hours [includes tests]
5. US3 (T031-T037): ~1.5 hours [includes tests]

**Total MVP time**: ~8.5 hours of coding + testing

**Success criteria**:
- ✅ Can run `python -m ingestion.main --base-url https://physical-ai-robotics.vercel.app`
- ✅ Produces ingestion report with vector count, errors, verification
- ✅ All US1 + US2 + US3 integration tests passing

### Post-MVP: US4 + Polish (5 tasks)
Add after MVP validated with live data.

---

## Quick Start for Implementation

```bash
# Setup
cd backend
python -m venv venv
source venv/bin/activate  # or venv\Scripts\activate on Windows
pip install -r requirements.txt

# Run Phase 1 tasks (T001-T006)
# Create directory structure and config

# Run Phase 2 tasks (T007-T011)
# Implement checkpoint, chunker, embedder, qdrant_storage classes

# Run Phase 3a (T012-T021)
# Implement crawler and orchestrator; test with live URL

# Verify with unit tests
pytest backend/tests/unit/ -v

# Verify with integration test
pytest backend/tests/integration/test_crawl_and_chunk.py -v
```

---

## File Structure After Task Completion

```
backend/
├── ingestion/
│   ├── __init__.py
│   ├── main.py                 # Orchestrator (T017)
│   ├── config.py               # Config loading (T005, T038)
│   ├── crawler.py              # URL crawling (T012)
│   ├── chunker.py              # Text chunking (T008)
│   ├── embedder.py             # Cohere embeddings (T010)
│   ├── qdrant_storage.py       # Qdrant management (T009)
│   └── checkpoint.py           # Resume capability (T007)
├── tests/
│   ├── unit/
│   │   ├── test_crawler.py     # T019
│   │   ├── test_chunker.py     # T020
│   │   ├── test_embedder.py    # T028
│   │   ├── test_qdrant_storage.py # T029
│   │   └── test_report_generation.py # T036
│   └── integration/
│       ├── test_crawl_and_chunk.py # T021
│       ├── test_embed_and_store.py # T030
│       ├── test_full_pipeline.py   # T037
│       └── test_reconfiguration.py # T042
├── requirements.txt            # T002
├── .env                        # T003
├── README.md                   # T043
└── .gitignore                  # T045
```

---

## Notes

- **No intermediate files** created until each phase complete
- **Tests are integration-heavy**: Most validation via live API tests (Cohere + Qdrant)
- **Checkpoint critical**: Enables safe re-runs and failure recovery
- **Content hash IDs**: SHA256(url + text) ensures idempotent Qdrant inserts
- **Configuration via env vars**: No hardcoded secrets (constitution requirement)
