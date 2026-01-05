# Feature Specification: Build RAG Agent API with OpenAI Agents SDK

**Feature Branch**: `003-rag-agent-api`
**Created**: 2025-12-26
**Status**: Draft
**Input**: Spec-3: Build RAG agent API using OpenAI Agents SDK and FastAPI

---

## User Scenarios & Testing

### User Story 1 - Query Book via Chat Endpoint (Priority: P1)

Backend engineers need to submit natural language questions to the API and receive contextually relevant answers grounded in the textbook content.

**Why this priority**: Core functionality for RAG chatbot. Without this, there's no way to interact with the agent or validate that retrieval + generation works together.

**Independent Test**: Can be fully tested by calling the `/chat` endpoint with a query, verifying that the response contains relevant book content and a coherent answer based on retrieved chunks.

**Acceptance Scenarios**:

1. **Given** a question about textbook content (e.g., "What is ROS2?"), **When** submitting the query to `/chat`, **Then** the system returns a JSON response with the user's question, agent's answer, and retrieved context chunks.

2. **Given** a query that has relevant content in the collection, **When** the agent processes the request, **Then** the response cites specific retrieved chunks and includes metadata (URLs, page titles) for attribution.

3. **Given** multiple sequential queries in the same conversation, **When** sending them to `/chat`, **Then** each query is processed independently and returns relevant results.

---

### User Story 2 - Retrieve Context from Qdrant (Priority: P1)

Backend engineers need the agent to automatically retrieve relevant textbook chunks from Qdrant and use them as context for generating answers.

**Why this priority**: Ensures RAG pipeline integration works. If retrieval fails, answers won't be grounded in book content.

**Independent Test**: Can be fully tested by verifying that for each user query, the system retrieves 3-5 relevant chunks from Qdrant and includes them in the response metadata.

**Acceptance Scenarios**:

1. **Given** a user question, **When** the agent processes it, **Then** the system retrieves 3-5 relevant chunks from the Qdrant collection using semantic search.

2. **Given** retrieved chunks from Qdrant, **When** preparing the agent prompt, **Then** the chunks are formatted with metadata (source URL, page title, section headers) and passed to the LLM as context.

3. **Given** a query that has no relevant content in Qdrant, **When** the agent processes it, **Then** it returns a response indicating that the information is not in the textbook.

---

### User Story 3 - Ground Responses in Book Content (Priority: P1)

Backend engineers need the agent to answer only based on retrieved textbook chunks, avoiding hallucinations about content not in the book.

**Why this priority**: Critical for accuracy and trust. Users must know that answers are grounded in the actual textbook.

**Independent Test**: Can be fully tested by verifying that the agent's response references retrieved chunks and doesn't claim facts that aren't in the textbook content.

**Acceptance Scenarios**:

1. **Given** a query asking about ROS2 features, **When** the agent generates a response, **Then** it only mentions features found in the retrieved chunks and cites their source URLs.

2. **Given** a question about information not in the textbook, **When** the agent processes it, **Then** it explicitly states that the information is not available in the textbook instead of providing external information.

3. **Given** conflicting information in retrieved chunks, **When** the agent generates a response, **Then** it acknowledges the discrepancy and cites both sources.

---

### User Story 4 - Support Text-Only and Full-Book Query Modes (Priority: P2)

Backend engineers need flexibility to control retrieval scope: either retrieve only from provided text snippets or search the entire embedded collection.

**Why this priority**: Enables different use cases (limited context vs. comprehensive search). Allows testing with isolated content before full deployment.

**Independent Test**: Can be fully tested by configuring retrieval scope and verifying results differ based on the selected scope.

**Acceptance Scenarios**:

1. **Given** a request with `retrieval_scope: "text_only"` and provided text chunks, **When** querying, **Then** the agent only retrieves from provided chunks, not from Qdrant.

2. **Given** a request with `retrieval_scope: "full_collection"`, **When** querying, **Then** the agent retrieves from the entire 192-vector Qdrant collection.

3. **Given** both scenarios with the same query, **When** comparing results, **Then** text-only mode returns results only from provided snippets, while full-collection mode may return additional relevant chunks from Qdrant.

---

### User Story 5 - Handle API Errors Gracefully (Priority: P2)

Backend engineers need clear error messages when something fails (missing context, API timeout, invalid requests).

**Why this priority**: Enables debugging and improves user experience. Without clear errors, failures are ambiguous.

**Independent Test**: Can be fully tested by triggering error conditions (empty query, API failure, malformed request) and verifying structured error responses.

**Acceptance Scenarios**:

1. **Given** a malformed request (e.g., missing `query` field), **When** submitting to `/chat`, **Then** the system returns a 400 error with a clear message describing what's missing.

2. **Given** an OpenAI API timeout, **When** the agent can't generate a response, **Then** the system returns a 503 error with a message indicating the external service is unavailable.

3. **Given** a Qdrant connection failure, **When** retrieval fails, **Then** the system returns a 500 error with details about the retrieval failure and suggests retrying.

---

### Edge Cases

- What happens when the user submits an empty query or whitespace-only query? (System should return 400 error with helpful message)
- How does the system handle queries in non-English languages? (Cohere embedding supports multiple languages; agent should attempt to respond)
- What happens if retrieved chunks are empty or all have low similarity scores? (Agent should state that information isn't in the textbook)
- How does the system handle extremely long queries (>10,000 characters)? (Should truncate or reject with clear error)
- What if the OpenAI API is rate-limited or returns an error? (Should return 503 and suggest retrying)
- What happens when Qdrant collection has fewer chunks than requested top-k? (Should return all available chunks)

## Requirements

### Functional Requirements

- **FR-001**: System MUST expose a `/chat` POST endpoint that accepts a query and returns an agent response
- **FR-002**: System MUST embed user queries using the same embedding model as Spec 2 (Cohere embed-english-v3.0)
- **FR-003**: System MUST retrieve relevant chunks from the Qdrant collection using semantic similarity search
- **FR-004**: System MUST use OpenAI Agents SDK to initialize an agent with retrieval capabilities
- **FR-005**: System MUST pass retrieved chunks to the agent as context in the system prompt
- **FR-006**: System MUST configure the agent to answer ONLY based on retrieved textbook content
- **FR-007**: System MUST return structured JSON responses including: user query, agent answer, retrieved chunks, and metadata
- **FR-008**: System MUST support configurable retrieval scope (text-only snippets vs. full Qdrant collection)
- **FR-009**: System MUST validate requests and return 400 errors for malformed input
- **FR-010**: System MUST handle API failures gracefully with appropriate HTTP status codes and error messages
- **FR-011**: System MUST log all queries, retrieval operations, and agent responses for debugging
- **FR-012**: System MUST load configuration from environment variables (OpenAI API key, Qdrant credentials, etc.)

### Key Entities

- **ChatRequest**: User's query and optional retrieval configuration
  - `query`: Natural language question (required, 1-10,000 characters)
  - `retrieval_scope`: "text_only" | "full_collection" (default: "full_collection")
  - `top_k`: Number of chunks to retrieve (default: 5, range: 1-100)
  - `context_text`: Optional text snippets for text-only mode

- **ChatResponse**: Agent's answer with context
  - `query`: Original user query
  - `answer`: Agent-generated response grounded in textbook
  - `retrieved_chunks`: List of context chunks used for answering
  - `metadata`: For each chunk: source_url, page_title, section_headers
  - `execution_metrics`: Retrieval time, generation time, total time
  - `status`: "success" or "error"

- **RetrievedChunk**: Content chunk from Qdrant or provided text
  - `chunk_id`: Unique identifier
  - `text`: Chunk content
  - `similarity_score`: Relevance to query (if from Qdrant)
  - `source_url`: Where the chunk came from
  - `page_title`: Title of the page containing chunk

## Success Criteria

### Measurable Outcomes

- **SC-001**: API responds to chat requests in under 5 seconds (end-to-end from query to response)
- **SC-002**: Retrieved chunks are relevant to the user's query (manual evaluation: ≥80% of chunks are semantically related)
- **SC-003**: Agent answers are grounded in retrieved content (manual evaluation: ≥90% of responses cite or directly reference retrieved chunks)
- **SC-004**: System handles at least 10 concurrent requests without errors
- **SC-005**: Error responses include actionable error messages (e.g., "Query cannot be empty")
- **SC-006**: All requests are logged with query, response, and performance metrics
- **SC-007**: Retrieval scope configuration works as intended (text-only retrieves only from provided text; full-collection retrieves from Qdrant)
- **SC-008**: System successfully answers questions about book content with ≥80% relevance (manual testing with sample questions)

---

## Assumptions

- OpenAI API key is available and valid (requires API access to GPT-4 or GPT-4o)
- Qdrant collection `textbook_embeddings` is populated with 192+ vectors from Spec 1
- Cohere API key is available (same as Specs 1-2) for query embedding
- User questions are in English (embedding model assumes English text)
- Retrieved chunks should be presented in relevance order (highest similarity first)
- Responses should not hallucinate; if information isn't in textbook, agent should say so
- FastAPI is suitable for this backend (standard REST API, not WebSocket streaming)
- No user authentication is required for the initial MVP (backend engineers can test without auth)

---

## Constraints

- **Framework**: FastAPI (specified in requirements)
- **Agent SDK**: OpenAI Agents SDK (specified in requirements)
- **Embedding Model**: Cohere embed-english-v3.0 (must match Specs 1-2)
- **Vector Database**: Qdrant Cloud (reuse from Spec 1)
- **LLM**: OpenAI API (GPT-4 or GPT-4o recommended for reasoning)
- **Language**: Python 3.10+
- **Response Format**: Structured JSON (no HTML, streaming, or other formats in this spec)
- **Concurrency**: Single-threaded per request; system must handle concurrent requests via FastAPI
- **Latency**: Responses must complete in under 5 seconds (user-facing timeout)

---

## Out of Scope

- Frontend UI or chatbot interface (Spec 4 will cover this)
- User authentication, authorization, or session management
- WebSocket streaming or real-time responses
- Multi-turn conversation history or stateful sessions
- Fine-tuning or prompt engineering beyond basic grounding
- Caching or response memoization
- Analytics dashboards or usage tracking
- Docusaurus integration or documentation UI
- OpenAI Agents SDK advanced features (function calling, tool use) beyond basic retrieval

---

## Dependencies

- **Spec 1 Completion**: Website Ingestion & Vector Storage Pipeline (provides populated Qdrant collection with 192+ vectors)
- **Spec 2 Completion**: Retrieve Embedded Book Content (provides retrieval patterns and Cohere integration)
- **OpenAI API Access**: Requires active OpenAI account with API credentials
- **FastAPI Framework**: Backend web framework
- **Python 3.10+**: Language and runtime
- **Qdrant Python Client**: Vector search operations
- **Cohere SDK**: Query embedding
- **OpenAI Python SDK**: Access to Agents SDK and GPT models

---

## Technical Notes

- The OpenAI Agents SDK simplifies agent implementation compared to custom orchestration
- Spec 2's `RetrieverClient` can be reused to handle Qdrant queries
- Spec 1's config patterns should be followed for environment variable management
- Retrieved chunks should be passed to the agent's system prompt to ground responses
- Response time <5 seconds includes: query embedding (0.3s), retrieval (0.2s), LLM generation (2-3s), overhead (1s buffer)
