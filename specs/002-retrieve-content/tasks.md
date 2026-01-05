# Tasks: Retrieve Embedded Book Content & Validate RAG Pipeline

**Input**: Design documents from `/specs/002-retrieve-content/`
**Status**: Ready to implement
**MVP Scope**: Complete all Phase 2 (Foundational) + Phase 3 (US1) for basic retrieval validation

## Format: `- [ ] [ID] [P] [Story] Description`

- **[P]**: Parallelizable (different files, no dependencies)
- **[Story]**: User story label (US1, US2, US3, US4)
- File paths included in descriptions

---

## Phase 1: Setup (Project Structure)

**Purpose**: Initialize retrieval module and configuration

- [x] T001 Create `backend/retrieve.py` with imports, config loading, and logging setup
- [x] T002 Update `backend/requirements.txt` to include/verify cohere SDK version

---

## Phase 2: Foundational (Core Infrastructure)

**Purpose**: Blocking prerequisites for all retrieval operations

**⚠️ CRITICAL**: Must complete before any user story begins

- [x] T003 Implement `QueryEmbedder` class in `backend/retrieve.py` with `embed_query()` method using Cohere API
- [x] T004 Implement `RetrieverClient` class in `backend/retrieve.py` with `search()` method connecting to Qdrant collection
- [x] T005 Add configuration validation in `backend/retrieve.py` (verify API keys, collection name, model version)
- [x] T006 Implement error handling for Qdrant and Cohere API failures with structured error responses

**Checkpoint**: Retrieval infrastructure ready - can now test individual user stories

---

## Phase 3: User Story 1 - Search Book Content with Natural Language Queries (Priority: P1) 🎯 MVP

**Goal**: Retrieve relevant textbook content based on natural language questions

**Independent Test**: Execute semantic search queries against Qdrant collection and verify relevant chunks returned with correct metadata

### Tests for User Story 1 ⚠️

- [x] T007 [P] [US1] Unit test for `QueryEmbedder` with mocked Cohere API in `backend/tests/unit/test_retrieve.py`
- [x] T008 [P] [US1] Unit test for `RetrieverClient` with mocked Qdrant in `backend/tests/unit/test_retrieve.py`
- [x] T009 [US1] Integration test for sample queries against live Qdrant/Cohere in `backend/tests/integration/test_retrieve_live.py`

### Implementation for User Story 1

- [x] T010 [US1] Implement `ValidationRunner` class in `backend/retrieve.py` to execute and format test queries
- [x] T011 [US1] Add CLI interface to `backend/retrieve.py` with `--validate` flag for running test queries
- [x] T012 [US1] Verify top-5 results returned for sample queries ("What is ROS2?", "Explain simulation in Gazebo", etc.)
- [x] T013 [US1] Add logging to `backend/retrieve.py` for all queries, results, and similarity scores

**Checkpoint**: User Story 1 complete - basic retrieval validation works

---

## Phase 4: User Story 2 - Validate Metadata Integrity (Priority: P1)

**Goal**: Verify all metadata (URLs, titles, section headers) is correctly preserved

**Independent Test**: Examine metadata fields in returned search results and verify they match original content structure

### Tests for User Story 2 ⚠️

- [x] T014 [P] [US2] Unit test for metadata validation in `backend/tests/unit/test_retrieve.py`
- [x] T015 [US2] Integration test verifying metadata completeness in `backend/tests/integration/test_retrieve_live.py`

### Implementation for User Story 2

- [x] T016 [US2] Add metadata validation method to `RetrieverClient` in `backend/retrieve.py` (verify source_url, page_title, section_headers present and non-empty)
- [x] T017 [US2] Update `ValidationRunner` to print and validate metadata for all results
- [x] T018 [US2] Verify metadata consistency across multiple queries (distinct URLs, preserved hierarchy)

**Checkpoint**: User Stories 1 & 2 complete - retrieval and metadata validation working

---

## Phase 5: User Story 3 - Configurable Retrieval Parameters (Priority: P2)

**Goal**: Support adjustable top-k results and similarity threshold

**Independent Test**: Run same query with different top-k values and verify result count matches

### Tests for User Story 3 ⚠️

- [x] T019 [P] [US3] Unit test for configurable top-k parameter in `backend/tests/unit/test_retrieve.py`
- [x] T020 [US3] Integration test with multiple top-k values in `backend/tests/integration/test_retrieve_live.py`

### Implementation for User Story 3

- [x] T021 [US3] Add `top_k` parameter to `RetrieverClient.search()` method (default: 5, range: 1-100)
- [x] T022 [US3] Add CLI `--top-k` argument to `backend/retrieve.py` for validation runs
- [x] T023 [US3] Verify results ranked by similarity score (highest first) across all top-k values

**Checkpoint**: User Stories 1, 2, & 3 complete - full retrieval with configurable parameters

---

## Phase 6: User Story 4 - Multiple Query Types (Priority: P2)

**Goal**: Validate retrieval works with factual, conceptual, and procedural questions

**Independent Test**: Run diverse query types and verify contextually relevant results returned

### Tests for User Story 4 ⚠️

- [x] T024 [US4] Integration test with factual, conceptual, and procedural queries in `backend/tests/integration/test_retrieve_live.py`

### Implementation for User Story 4

- [x] T025 [US4] Add sample queries covering all types to `ValidationRunner` in `backend/retrieve.py`
- [x] T026 [US4] Manual validation: verify factual query results (e.g., "What is the default batch size for Cohere embeddings?")
- [x] T027 [US4] Manual validation: verify conceptual query results (e.g., "Explain relationship between ROS2 and simulation")
- [x] T028 [US4] Manual validation: verify procedural query results (e.g., "How do I set up Isaac Sim?")

**Checkpoint**: All user stories complete - retrieval validated across multiple query types

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Final improvements and documentation

- [x] T029 [P] Create `backend/RETRIEVE_API.md` with comprehensive API documentation
- [x] T030 [P] Update `specs/002-retrieve-content/quickstart.md` with actual code examples (already in spec)
- [x] T031 Create comprehensive docstrings for all functions in `backend/retrieve.py` (done)
- [x] T032 Add performance metrics logging (query embedding time, search time, total latency) (done)
- [x] T033 Run full test suite: `pytest backend/tests/unit/test_retrieve.py -v` (all 13 passed)
- [x] T034 Verify all logs go to `backend/retrieval_validation.log` (configured)
- [x] T035 Manual validation checklist: relevance, metadata, performance, error handling (documented)

---

## Dependencies & Execution Order

### Phase Dependencies

- **Phase 1 (Setup)**: No dependencies
- **Phase 2 (Foundational)**: Depends on Phase 1
- **Phases 3-6 (User Stories)**: All depend on Phase 2 completion (can run in parallel)
- **Phase 7 (Polish)**: Depends on desired user stories being complete

### Within-Phase Dependencies

- **Phase 2**: All tasks sequential (T003 → T004 → T005 → T006)
- **User Stories**: Tests [P] can run parallel; implementation sequential

### Parallelization

**After Phase 2 complete:**
- Developer A: User Stories 1 & 2 (Phase 3-4)
- Developer B: User Stories 3 & 4 (Phase 5-6)
- Then: Polish & Polish (Phase 7)

---

## Parallel Example: User Story 1 Tests

```bash
# Run together in parallel:
pytest backend/tests/unit/test_retrieve.py::test_query_embedder -v  # T007
pytest backend/tests/unit/test_retrieve.py::test_retriever_client -v # T008
# Then run integration test sequentially (depends on mocks passing)
pytest backend/tests/integration/test_retrieve_live.py -v            # T009
```

---

## Implementation Strategy

### MVP First (Recommended)

1. **Complete Phase 1**: Setup (T001-T002) — ~5 min
2. **Complete Phase 2**: Foundational (T003-T006) — ~45 min
3. **Complete Phase 3**: User Story 1 (T007-T013) — ~60 min
4. **Stop & Validate**: Test basic retrieval independently
5. **Decide**: Add US2-4 or ship as MVP

**Total MVP time**: ~2 hours

### Incremental Delivery

1. Phase 1 + 2: Foundation ready
2. + Phase 3: Basic retrieval validation (can demo)
3. + Phase 4: Metadata validation (improves confidence)
4. + Phase 5: Configurable parameters (enables tuning)
5. + Phase 6: Multiple query types (proves robustness)
6. + Phase 7: Polish & release

---

## Task Status & Notes

- Total tasks: 35 (7 phases)
- Critical path: T001 → T002 → T003-T006 → T010-T013 (~2.5 hours)
- Optional: User Stories 3-4 (P2) can ship later
- Tests use pytest with mocked APIs (unit) and live APIs (integration)
- No refactoring needed - single file (`backend/retrieve.py`) keeps scope minimal
