# Implementation Plan: Build RAG Agent API with OpenAI Agents SDK

**Branch**: `003-rag-agent-api` | **Date**: 2025-12-26 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/003-rag-agent-api/spec.md`

---

## Summary

Implement a FastAPI-based RAG (Retrieval-Augmented Generation) agent service that answers natural language questions about the textbook by retrieving relevant content from Qdrant and using OpenAI Agents SDK to generate grounded responses. The agent will reuse retrieval patterns from Spec 2 and configuration management from Spec 1, while introducing OpenAI agent orchestration and FastAPI HTTP interface.

**Approach**: Single-file Python module (`backend/agent.py`) containing agent initialization, retrieval integration, and FastAPI endpoints. Reuses `RetrieverClient` from Spec 2 for semantic search. Configurable retrieval scope supports both full-collection and text-only query modes.

---

## Technical Context

**Language/Version**: Python 3.10+
**Primary Dependencies**: FastAPI, OpenAI Agents SDK, Cohere SDK, Qdrant Python client, pydantic (for request/response validation)
**Storage**: Qdrant Cloud (existing from Spec 1, read-only access); optional Neon PostgreSQL for conversation logs (out-of-scope for MVP)
**Testing**: pytest with mocked OpenAI/Cohere APIs (unit), live API tests (integration)
**Target Platform**: Backend REST API server (Linux, macOS, Windows + WSL2)
**Project Type**: Single Python backend module extending existing backend
**Performance Goals**: <5 seconds end-to-end response time (p95), <100ms overhead per request
**Constraints**: Responses must be grounded strictly in retrieved textbook content; no hallucinations tolerated
**Scale/Scope**: Single-threaded per request, FastAPI handles concurrency; target 10+ concurrent requests

---

## Constitution Check

**✅ PASS**: Plan aligns with all constitutional principles

- ✅ **Book as Authoritative Source**: Agent configured to answer ONLY from retrieved textbook content
- ✅ **RAG Mandate**: Every response retrieves relevant chunks before generation; confidence thresholds enforced
- ✅ **Traceability**: Responses cite retrieved chunks with metadata (source URLs, section headers)
- ✅ **Strict Scope Boundaries**: Configurable `retrieval_scope` supports text-only mode for strict boundaries; out-of-scope queries receive "not in textbook" responses
- ✅ **Technical Accuracy**: Reuses validated patterns from Specs 1-2; OpenAI API ensures accuracy
- ✅ **AI-Native Content**: Leverages chunked, hierarchical content structure from Spec 1
- ✅ **Spec-Driven Development**: Implementation follows spec exactly; no deviations without spec amendments

**Gates**: ✅ PASS
- No constitutional violations
- OpenAI Agents SDK and FastAPI align with mandated tech stack
- RAG integrity requirements explicitly coded in system prompt
- Logging design supports auditing and compliance

---

## Project Structure

### Documentation (this feature)

```text
specs/003-rag-agent-api/
├── spec.md                      # Feature specification (COMPLETE)
├── plan.md                       # This file
├── research.md                   # Phase 0: Research findings (TBD)
├── data-model.md                # Phase 1: Data schemas (TBD)
├── contracts/                    # Phase 1: API contracts (TBD)
│   └── chat-endpoint.openapi.yaml
├── quickstart.md                # Phase 1: Usage guide (TBD)
├── checklists/
│   └── requirements.md          # Quality validation (COMPLETE)
└── tasks.md                      # Phase 2 output (from /sp.tasks)
```

### Source Code (backend extension)

```text
backend/
├── agent.py                      # Main agent module (NEW - Spec 3)
│   ├── AgentConfig class
│   ├── GroundedAgent class (OpenAI agent orchestration)
│   ├── FastAPI app and /chat endpoint
│   └── Request/response schemas (Pydantic)
├── retrieve.py                   # Existing from Spec 2
├── requirements.txt              # Add: openai, fastapi, uvicorn
├── .env                          # Add: OPENAI_API_KEY
├── RETRIEVE_API.md              # Existing from Spec 2
└── tests/
    ├── unit/
    │   ├── test_retrieve.py     # Existing from Spec 2
    │   └── test_agent.py         # NEW: Agent unit tests
    └── integration/
        ├── test_retrieve_live.py # Existing from Spec 2
        └── test_agent_live.py    # NEW: Live OpenAI API tests
```

**Structure Decision**: Single-file design (`backend/agent.py`) keeps scope focused on agent orchestration. Reuses `RetrieverClient` from Spec 2 (imported as dependency) to avoid code duplication. FastAPI server exposes `/chat` endpoint with request validation via Pydantic schemas.

---

## Key Design Decisions

### 1. **Reuse Spec 2 Retrieval Client**
- Import `RetrieverClient` from `backend/retrieve.py`
- Avoid re-implementing Qdrant connection, Cohere embedding, error handling
- Agent uses `client.search()` to fetch context before generation

**Rationale**: Minimizes code duplication, leverages tested retrieval logic, maintains consistency

### 2. **OpenAI Agents SDK for Orchestration**
- Use OpenAI Agents SDK to initialize agent with system prompt
- System prompt explicitly restricts responses to retrieved chunks
- Agent formats retrieved chunks as context injection in prompt

**Rationale**: Agents SDK handles complex reasoning without custom prompt engineering; easier to enforce grounding

### 3. **Single File Agent Module**
- `backend/agent.py` contains:
  - `AgentConfig`: Environment variable loading
  - `GroundedAgent`: Wrapper around OpenAI agent
  - FastAPI app initialization and `/chat` endpoint
  - Request/response schemas (Pydantic)
  - Error handling and logging

**Rationale**: Spec 3 scope is MVP (agent + retrieval integration); simple structure keeps it maintainable

### 4. **Configurable Retrieval Scope**
- Request parameter: `retrieval_scope: "text_only" | "full_collection"`
- `text_only`: Process only provided `context_text` (P2 feature, MVP can skip)
- `full_collection`: Search entire Qdrant collection (default)

**Rationale**: Enables testing with isolated content; MVP ships with full-collection mode

### 5. **Grounding via System Prompt**
- System prompt explicitly instructs: "Answer ONLY using the provided context. If context doesn't address the question, say 'The textbook does not cover this topic.'"
- Retrieved chunks passed as context in prompt
- No external knowledge injection allowed

**Rationale**: Constitutional requirement (Principle II); simple to implement, enforced by LLM instruction

### 6. **Error Handling & Response Format**
- Structured JSON responses: `{query, answer, retrieved_chunks, execution_metrics, status}`
- HTTP status codes: 200 (success), 400 (validation), 500 (API failures), 503 (timeout)
- Error responses include actionable messages

**Rationale**: Enables client error handling; visibility into failures

---

## Data Model & Entities

### Request Schema (ChatRequest)
```python
class ChatRequest:
    query: str                           # Required: user question
    retrieval_scope: Literal["text_only", "full_collection"] = "full_collection"
    top_k: int = 5                       # 1-100
    context_text: Optional[str] = None   # Optional: for text_only mode
```

### Response Schema (ChatResponse)
```python
class ChatResponse:
    query: str
    answer: str                          # Agent-generated response
    retrieved_chunks: List[RetrievedChunk]
    execution_metrics: {
        "retrieval_time_ms": float,
        "generation_time_ms": float,
        "total_time_ms": float
    }
    status: Literal["success", "error"]
    error: Optional[{"code": str, "message": str}] = None
```

### RetrievedChunk (from Spec 2)
```python
class RetrievedChunk:
    chunk_id: str
    text: str
    similarity_score: float
    source_url: str
    page_title: str
    section_headers: List[str]
```

---

## Implementation Phases

### Phase 1: Agent Core (Days 1-2)
1. Create `backend/agent.py` with imports and config loading
2. Implement `AgentConfig` class (load env vars: OPENAI_API_KEY, existing Qdrant/Cohere config)
3. Implement `GroundedAgent` class wrapper around OpenAI Agents SDK
4. Define system prompt with grounding instructions
5. Test agent initialization and connectivity to OpenAI API

### Phase 2: Retrieval Integration (Days 2-3)
1. Import `RetrieverClient` from `backend/retrieve.py`
2. Integrate retrieval into agent: pre-fetch chunks before agent invocation
3. Format chunks as context injection in agent prompt
4. Implement context window management (pass top_k chunks to agent)
5. Test that agent receives context and generates responses

### Phase 3: FastAPI Endpoint (Days 3-4)
1. Initialize FastAPI app in `agent.py`
2. Implement `/chat` POST endpoint with Pydantic request validation
3. Implement response formatting with execution metrics
4. Implement error handling and HTTP status codes
5. Test endpoint with curl/Postman; verify response schema

### Phase 4: Testing (Days 4-5)
1. Write unit tests with mocked OpenAI API
2. Write integration tests with live OpenAI API (uses real quota)
3. Test error scenarios (empty query, API timeout, malformed request)
4. Test retrieval scope configuration (P2 feature)
5. Run full test suite; verify <5s latency target

### Phase 5: Polish (Days 5-6)
1. Add comprehensive docstrings
2. Create `AGENT_API.md` documentation
3. Create `quickstart.md` with usage examples
4. Update `backend/README.md` with agent section
5. Prepare for code review and deployment

---

## Dependencies & Risks

### External Dependencies
- ✅ **Spec 1** (ingestion): Qdrant collection must exist with 192+ vectors
- ✅ **Spec 2** (retrieval): RetrieverClient module must be available
- ✅ **OpenAI API**: Requires valid API key and GPT-4/GPT-4o access (not free tier)
- ✅ **FastAPI**: Already in requirements for other projects
- ✅ **Python 3.10+**: Existing environment

### Risks & Mitigations

| Risk | Probability | Impact | Mitigation |
|------|-------------|--------|-----------|
| OpenAI API rate limiting | Medium | Medium | Implement retry logic with exponential backoff; queue requests if needed |
| Hallucination in responses | Medium | High | Strict system prompt; confidence threshold; test with samples |
| Qdrant connection timeout | Low | Medium | Reuse error handling from Spec 2; return 500 on retrieval failure |
| Token overflow in prompt | Low | High | Monitor token usage; truncate context if needed (keep last N chunks) |
| Agent doesn't ground in context | Medium | High | Test with known queries; validate citation accuracy manually |

---

## Success Metrics

### Functionality
- ✅ `/chat` endpoint returns JSON responses with agent answer and context
- ✅ Agent responses reference retrieved chunks and include source URLs
- ✅ System handles 10+ concurrent requests without errors
- ✅ Edge cases handled: empty query (400), API timeout (503), malformed request (400)

### Performance
- ✅ <5 second end-to-end latency per query (p95)
- ✅ Retrieval time <500ms
- ✅ OpenAI generation time <3 seconds

### Quality
- ✅ ≥80% of retrieved chunks are semantically relevant (manual sampling)
- ✅ ≥90% of responses cite or reference retrieved chunks (no hallucinations)
- ✅ Zero hardcoded secrets or API keys in code

### Testing
- ✅ Unit tests pass with mocked APIs (100% coverage of agent logic)
- ✅ Integration tests pass with live OpenAI API (≥5 sample queries)
- ✅ Error scenarios tested and handled gracefully

---

## Out of Scope (for Spec 3 MVP)
- ❌ Text-only retrieval scope mode (P2 feature, can ship later)
- ❌ Multi-turn conversation history
- ❌ WebSocket streaming or Server-Sent Events
- ❌ Authentication or authorization
- ❌ Persistent conversation logging to Neon DB
- ❌ Fine-tuning or custom prompt engineering beyond basic grounding
- ❌ Analytics dashboards

---

## Assumptions

- OpenAI API is available and responsive (<3s generation time)
- GPT-4 or GPT-4o model is used for reasoning capability
- Qdrant collection is stable and responsive (<500ms search)
- Retrieved chunks are well-formed and semantic meaning is preserved
- Users accept 5-second latency as acceptable for chatbot responses
- No authentication required for MVP (backend engineers only)
- Chat responses don't require multi-turn context preservation

---

## Notes

- Phase durations are estimates; actual time depends on OpenAI API complexity and debugging needs
- Integration tests consume OpenAI API quota; consider using cost-saving strategies (smaller models for testing if available)
- Agent system prompt is critical for grounding; iterative testing may be needed to refine
- Retrieved context window should be tuned based on token limits of chosen LLM model

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
