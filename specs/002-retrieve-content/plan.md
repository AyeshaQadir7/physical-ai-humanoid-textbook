# Implementation Plan: Retrieve Embedded Book Content & Validate RAG Pipeline

**Branch**: `002-retrieve-content` | **Date**: 2025-12-25 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/002-retrieve-content/spec.md`

## Summary

Implement a validation layer for the RAG retrieval pipeline by querying existing embeddings stored in Qdrant Cloud. The system will embed natural language queries using Cohere's model (matching the ingestion pipeline), perform similarity searches against the 192-vector textbook collection, and return relevant chunks with complete metadata for manual validation of retrieval accuracy.

**Approach**: Single-file Python utility (`backend/retrieve.py`) that connects to the populated Qdrant collection from Spec 1 and demonstrates successful retrieval with sample queries. No API server, agent framework, or re-embedding required.

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: Cohere SDK, Qdrant Python client (existing from Spec 1), python-dotenv
**Storage**: Qdrant Cloud (existing collection from Spec 1, read-only access)
**Testing**: pytest with mocked Qdrant responses (unit), live Qdrant queries (integration)
**Target Platform**: Backend server / CLI utility
**Project Type**: Single Python module (retrieval utility)
**Performance Goals**: <2 seconds per query (end-to-end query embedding + search)
**Constraints**:
- Reuse existing Qdrant collection (no re-indexing)
- Reuse Cohere API key and model from Spec 1
- Vector dimensions: 1024 (Cohere embed-english-v3.0)
- Similarity metric: COSINE (matching Spec 1 collection)
**Scale/Scope**: Retrieval from 192-vector collection, support 1-100 configurable top-k

## Constitution Check

**Project Principles** (from `.specify/memory/constitution.md`):
- ✅ Minimal dependencies: Only Cohere, Qdrant, python-dotenv (no new frameworks)
- ✅ No hardcoded secrets: Uses .env configuration (consistent with Spec 1)
- ✅ Modular code: Single responsibility (query embedding + retrieval)
- ✅ Testable: Mocked unit tests + integration tests with live Qdrant
- ✅ Documented: Logging for all operations, clear function signatures
- ✅ Error handling: Graceful failures with informative messages
- ✅ Type hints: Python 3.10+ type annotations throughout

**Gates**: ✅ PASS
- No violations of project principles
- Follows established patterns from Spec 1 (config, logging, error handling)
- Minimal new surface area (single module with clear boundaries)

## Project Structure

### Documentation (this feature)

```text
specs/002-retrieve-content/
├── spec.md              # Feature specification (COMPLETE)
├── plan.md              # This file (COMPLETE)
├── data-model.md        # Data models and entities (Phase 1)
├── quickstart.md        # How to run retrieval tests (Phase 1)
├── research.md          # Research findings (Phase 0 - not needed)
├── contracts/           # API contracts (Phase 1 - optional)
└── checklists/
    └── requirements.md  # Quality validation (COMPLETE)
```

### Source Code (backend extension)

```text
backend/
├── retrieve.py                  # Main retrieval utility (NEW)
├── requirements.txt             # Dependencies (existing + cohere update)
├── .env                         # Configuration (existing)
├── ingestion/                   # Existing from Spec 1
│   ├── config.py               # Reuse Config class
│   ├── embedder.py             # Study Cohere embedding patterns
│   └── ... (other modules)
└── tests/
    ├── unit/
    │   └── test_retrieve.py     # Unit tests with mocks (NEW)
    └── integration/
        └── test_retrieve_live.py # Integration tests with live Qdrant (NEW)
```

**Structure Decision**: Single Python module (`backend/retrieve.py`) extending the existing ingestion backend. Reuses configuration, logging, and error handling patterns from Spec 1. Keeps project simple and maintainable without separate project structure.

## Key Design Decisions

### 1. **Reuse Existing Infrastructure**
- Config class from `backend/ingestion/config.py` for environment variables
- Cohere SDK pattern from `backend/ingestion/embedder.py` for query embedding
- Qdrant client from `backend/ingestion/qdrant_storage.py` for connection
- Logging setup from `backend/ingestion/main.py`

**Rationale**: Minimizes code duplication, maintains consistency across Spec 1 and Spec 2, reduces bugs from divergent patterns.

### 2. **Single File for Retrieval Logic**
- Single `retrieve.py` module containing:
  - `QueryEmbedder` class: Embed search queries (reuse Cohere logic)
  - `RetrieverClient` class: Query Qdrant collection and return results
  - `ValidationRunner` class: Execute test queries and print results
  - Helper functions for formatting/logging output

**Rationale**: Spec 2 scope is validation only (not production RAG API), so single file suffices. Easy to move to `qdrant_storage.py` later if needed for Spec 3.

### 3. **No API Server**
- Retrieval logic is a library (can be imported)
- Optional CLI interface for running validation tests
- No FastAPI/Flask/Django (per spec constraints)

**Rationale**: Focuses on testing and validation, not production serving. API layer can be added in future spec.

### 4. **Configurable Top-K**
- `RetrieverClient.search(query: str, top_k: int = 5)` method
- Validation with k=3, 5, 10 to demonstrate flexibility
- Config file can override default (not CLI-only)

**Rationale**: Supports both programmatic use (library) and configuration-based tuning.

### 5. **Comprehensive Logging**
- Log all queries, results, similarity scores, execution times
- Separate log file `retrieval_validation.log` for debugging
- DEBUG level available for detailed API call inspection

**Rationale**: Essential for validating retrieval accuracy and diagnosing issues.

## Data Model & Entities

### **Query** (Input)
```
{
  "text": "What is ROS2?",
  "embedding": [<1024 floats>],  # Generated by Cohere
  "model": "embed-english-v3.0"
}
```

### **SearchResult** (Retrieved Chunk)
```
{
  "chunk_id": "<uuid>",
  "text": "ROS2 (Robot Operating System 2) is...",
  "similarity_score": 0.87,
  "metadata": {
    "source_url": "https://physical-ai-robotics.vercel.app/docs/module-1-ros2/overview",
    "page_title": "ROS2 Overview",
    "section_headers": ["Module 1: ROS2", "Getting Started", "What is ROS2?"],
    "chunk_index": 0
  }
}
```

### **RetrievalResponse** (Output)
```
{
  "query": "What is ROS2?",
  "results": [<SearchResult>, ...],
  "total_results": 5,
  "execution_time_ms": 1234,
  "model_used": "embed-english-v3.0"
}
```

## Implementation Phases

### Phase 1: Setup & Configuration (1-2 hours)
1. Create `backend/retrieve.py` with imports and config loading
2. Implement `QueryEmbedder` class (embed queries via Cohere)
3. Implement `RetrieverClient` class (connect to Qdrant, execute search)
4. Test basic connectivity to Qdrant and Cohere APIs

### Phase 2: Validation & Testing (2-3 hours)
1. Implement `ValidationRunner` class (execute test queries, format output)
2. Create `test_retrieve.py` with unit tests (mocked Qdrant/Cohere)
3. Create `test_retrieve_live.py` with integration tests (live APIs)
4. Run validation against sample queries and manually verify relevance

### Phase 3: Documentation & Polish (1-2 hours)
1. Create `quickstart.md` with usage examples
2. Create `data-model.md` documenting entities
3. Add comprehensive docstrings to all functions
4. Update `backend/README.md` with retrieval section
5. Commit and prepare for code review

## Dependencies & Risks

### **Dependencies**
- ✅ Spec 1 completion (Qdrant collection must be populated with 192+ vectors)
- ✅ Cohere API access (same account as ingestion pipeline)
- ✅ Qdrant Cloud cluster (same cluster as ingestion)
- ✅ Python 3.10+ environment with existing dependencies

### **Risks & Mitigations**

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| Qdrant API changes between Spec 1 and Spec 2 | Low | High | Maintain version pinning in requirements.txt, test against live cluster |
| Cohere rate limiting during validation | Medium | Medium | Implement retry logic (reuse from embedder.py), batch queries |
| Low retrieval relevance (embeddings quality) | Medium | Medium | Verify with manual sampling, document in validation report |
| Metadata corruption during ingestion | Low | High | Validate sample chunk metadata matches expected structure |

## Success Metrics

- ✅ Code completes without errors against live Qdrant collection
- ✅ All sample queries return top-5 results with metadata
- ✅ Similarity scores are consistent (0.0-1.0 range, higher = more relevant)
- ✅ Retrieval latency <2 seconds per query
- ✅ Unit tests pass with mocked APIs (100% coverage of retrieval logic)
- ✅ Integration tests pass against live Qdrant (≥5 sample queries)
- ✅ Manual validation: ≥80% of results are semantically relevant to query
- ✅ Zero hardcoded secrets or API keys in code

## Not In Scope

- ❌ API server or REST endpoints
- ❌ Chatbot conversational logic
- ❌ Fine-tuning embedding model
- ❌ Caching query results
- ❌ Multiple embedding models
- ❌ Re-indexing Qdrant collection
- ❌ OpenAI Agents SDK or other agent frameworks
- ❌ Frontend UI or web interface
