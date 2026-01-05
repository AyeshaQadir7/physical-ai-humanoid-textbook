# RAG Agent API Reference

**Version**: 0.1.0
**Status**: Production-Ready with Comprehensive Error Handling
**Last Updated**: 2025-12-26

## Overview

The RAG Agent API is a FastAPI-based service that answers natural language questions about a robotics textbook using Retrieval-Augmented Generation (RAG) with the OpenAI Agents SDK. The API uses an AI agent with automatic tool orchestration to retrieve relevant content from a Qdrant vector database and generate grounded, contextual responses.

### Key Features

- **Agentic Retrieval**: Uses OpenAI Agents SDK with autonomous tool calling for semantic search
- **Automatic Tool Orchestration**: Agent autonomously decides when and how to retrieve content
- **Grounded Responses**: Responses are constrained to only cite retrieved textbook content
- **Flexible Retrieval Modes**: Choose between full-collection (agent-driven) or text-only mode
- **Semantic Search**: Uses Cohere embeddings to find relevant textbook chunks
- **Comprehensive Error Handling**: Clear error messages with appropriate HTTP status codes
- **Performance Monitoring**: Tracks latency across retrieval and generation phases
- **Production Ready**: Full logging, auditability, and error recovery

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    FastAPI /chat Endpoint                     │
├──────────────────────────────────────────────────────────────┤
│  Input: ChatRequest (query, retrieval_scope, top_k, context) │
├──────────────────────────────────────────────────────────────┤
│ Phase 1: Request Validation (Pydantic Schemas)               │
│ Phase 2: Agent Initialization (Agents SDK with Tool)         │
│   - For full-collection: Agent retrieves via tool            │
│   - For text-only: Context provided directly to agent        │
│ Phase 3: Agent Execution (OpenAI Agents SDK Runner)          │
│   - Agent calls retrieve_textbook_chunks tool as needed      │
│   - Agent generates response grounded in chunks              │
│ Phase 4: Grounding Validation                                │
│ Phase 5: Response Formatting                                 │
├──────────────────────────────────────────────────────────────┤
│  Output: ChatResponse (answer, chunks, metrics, status)      │
└──────────────────────────────────────────────────────────────┘
```

### Implementation Details

**Agent Architecture**:
- Uses OpenAI Agents SDK for autonomous agent orchestration
- Agent has access to `retrieve_textbook_chunks` tool for semantic search
- Agent instructions enforce grounding in retrieved content only
- Automatic tool calling based on agent instructions

**Retrieval Modes**:

1. **Full-Collection Mode (default)**:
   - Agent autonomously searches textbook via `retrieve_textbook_chunks` tool
   - No manual retrieval needed at endpoint level
   - Agent decides when and how to retrieve

2. **Text-Only Mode**:
   - User provides context text directly
   - Pre-processed into chunks and injected into agent query
   - No tool calling (agent uses provided context only)

## API Endpoints

### POST /chat

**Description**: Query the RAG agent with a natural language question.

**Request Schema**:
```json
{
  "query": "What is ROS2?",
  "retrieval_scope": "full_collection",
  "top_k": 5,
  "context_text": null
}
```

**Request Parameters**:

| Field | Type | Required | Default | Constraints | Description |
|-------|------|----------|---------|-------------|-------------|
| `query` | string | Yes | - | 1-10,000 chars | Natural language question |
| `retrieval_scope` | enum | No | "full_collection" | "text_only", "full_collection" | Retrieval mode: search Qdrant or use provided text |
| `top_k` | integer | No | 5 | 1-100 | Number of chunks to retrieve |
| `context_text` | string | No | null | - | User-provided context (used in text_only mode) |

**Success Response (200 OK)**:
```json
{
  "query": "What is ROS2?",
  "answer": "ROS2 is a flexible middleware for robotics...",
  "retrieved_chunks": [
    {
      "chunk_id": "chunk_123",
      "text": "ROS2 is a flexible middleware for robotics applications...",
      "similarity_score": 0.92,
      "source_url": "https://example.com/chapter1",
      "page_title": "Introduction to ROS2",
      "section_headers": ["Robotics", "ROS2 Basics"]
    }
  ],
  "execution_metrics": {
    "retrieval_time_ms": 245.3,
    "generation_time_ms": 2150.7,
    "total_time_ms": 2396.0
  },
  "retrieval_scope": "full_collection",
  "status": "success",
  "error": null
}
```

**Response Schema**:

| Field | Type | Description |
|-------|------|-------------|
| `query` | string | Original user query |
| `answer` | string | Agent-generated response (null on error) |
| `retrieved_chunks` | array | List of relevant chunks from Qdrant or provided text |
| `execution_metrics` | object | Timing metrics for retrieval and generation phases |
| `retrieval_scope` | string | Which retrieval mode was used ("text_only" or "full_collection") |
| `status` | string | "success" or "error" |
| `error` | object | Error details (only present if status="error") |

**Error Response (400, 429, 500, 503)**:
```json
{
  "query": "What is...",
  "answer": null,
  "retrieved_chunks": [],
  "execution_metrics": null,
  "retrieval_scope": "full_collection",
  "status": "error",
  "error": {
    "code": "RETRIEVAL_FAILED",
    "message": "Retrieval service unavailable, please retry"
  }
}
```

### Error Status Codes

| Code | HTTP Status | Meaning | Retry |
|------|-------------|---------|-------|
| VALIDATION_ERROR | 400 | Invalid request (empty query, invalid top_k, etc) | No |
| EMPTY_QUERY | 400 | Query cannot be empty or whitespace | No |
| RETRIEVAL_FAILED | 500 | Qdrant service unavailable | Yes |
| AGENT_FAILED | 500 | OpenAI API error (other than timeout/rate limit) | Yes |
| OPENAI_TIMEOUT | 503 | OpenAI API timeout (>30 seconds) | Yes |
| RATE_LIMITED | 429 | OpenAI rate limit exceeded | Yes, with backoff |
| GENERATION_FAILED | 500 | OpenAI service unavailable | Yes |
| INTERNAL_ERROR | 500 | Unexpected server error | Yes |

### GET /health

**Description**: Health check endpoint.

**Response (200 OK)**:
```json
{
  "status": "healthy",
  "timestamp": "2025-12-26T12:34:56.789Z",
  "service": "rag-agent-api"
}
```

## Usage Examples

### Example 1: Basic Query (Full-Collection Mode)

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS2 and how does it differ from ROS1?",
    "retrieval_scope": "full_collection",
    "top_k": 5
  }'
```

**Response**:
```json
{
  "query": "What is ROS2 and how does it differ from ROS1?",
  "answer": "ROS2 is the next-generation middleware for robotics, offering improved real-time capabilities, deterministic execution, and security compared to ROS1. See https://example.com/chapter1 for more details.",
  "retrieved_chunks": [
    {
      "chunk_id": "chunk_456",
      "text": "ROS2 introduces real-time performance guarantees...",
      "similarity_score": 0.89,
      "source_url": "https://example.com/chapter1",
      "page_title": "ROS2 vs ROS1",
      "section_headers": ["Robotics", "ROS Evolution"]
    }
  ],
  "execution_metrics": {
    "retrieval_time_ms": 200.0,
    "generation_time_ms": 2500.0,
    "total_time_ms": 2700.0
  },
  "retrieval_scope": "full_collection",
  "status": "success",
  "error": null
}
```

### Example 2: Text-Only Mode

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Summarize the provided content",
    "retrieval_scope": "text_only",
    "context_text": "ROS2 is a flexible middleware for robotics. It provides improved performance and security compared to ROS1."
  }'
```

**Response**:
```json
{
  "query": "Summarize the provided content",
  "answer": "The provided content describes ROS2 as a flexible middleware for robotics that improves upon ROS1 with better performance and security.",
  "retrieved_chunks": [
    {
      "chunk_id": "user_provided_1",
      "text": "ROS2 is a flexible middleware for robotics. It provides improved performance and security compared to ROS1.",
      "similarity_score": 1.0,
      "source_url": "<user_provided>",
      "page_title": "User-Provided Context",
      "section_headers": ["User Input"]
    }
  ],
  "execution_metrics": {
    "retrieval_time_ms": 10.0,
    "generation_time_ms": 1200.0,
    "total_time_ms": 1210.0
  },
  "retrieval_scope": "text_only",
  "status": "success",
  "error": null
}
```

### Example 3: Error Handling

**Invalid Query (too long)**:
```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "' $(python3 -c "print('a' * 10001)") '"
  }'
```

**Response (400 Bad Request)**:
```json
{
  "query": "",
  "answer": null,
  "retrieved_chunks": [],
  "execution_metrics": null,
  "retrieval_scope": null,
  "status": "error",
  "error": {
    "code": "VALIDATION_ERROR",
    "message": "ensure this value has at most 10000 characters"
  }
}
```

## Configuration

### Environment Variables

Required environment variables (must be set before running):

```bash
# OpenAI Configuration
OPENAI_API_KEY=sk-...              # OpenAI API key (required)
OPENAI_MODEL=gpt-3.5-turbo         # Model name (default: gpt-3.5-turbo)

# Qdrant Configuration
QDRANT_URL=https://...qdrant.io   # Qdrant Cloud URL (required)
QDRANT_API_KEY=...                 # Qdrant API key (required)
QDRANT_COLLECTION_NAME=textbook_embeddings  # Collection name (default)

# Cohere Configuration
COHERE_API_KEY=...                 # Cohere API key (required)
COHERE_MODEL=embed-english-v3.0   # Cohere model (default)

# Logging
LOG_LEVEL=INFO                      # Logging level (default: INFO)
```

### Optional Configuration

```bash
# Timeout (seconds)
OPENAI_TIMEOUT=30                   # OpenAI API timeout (default: 30s)

# Rate Limiting (handled by OpenAI SDK)
# The API automatically retries on rate limits with exponential backoff
```

## Performance Characteristics

### Latency SLA

- **Target**: <5 seconds end-to-end (p95)
- **Retrieval Phase**: <500ms (Cohere embedding + Qdrant search)
- **Generation Phase**: <3.5 seconds (OpenAI GPT-4)
- **Overhead**: <100ms (formatting, validation)

### Monitoring

The `execution_metrics` field in the response provides latency breakdown:

```json
"execution_metrics": {
  "retrieval_time_ms": 245.3,      // Cohere embedding + Qdrant search
  "generation_time_ms": 2150.7,    // OpenAI API call
  "total_time_ms": 2396.0          // End-to-end latency
}
```

### Resource Requirements

- **Memory**: ~500MB base + 50MB per concurrent request
- **CPU**: 1-2 cores for ~10 RPS
- **Network**: 1-2 Mbps for typical queries

## Best Practices

### Handling Errors

**Rate Limits (429)**:
```python
import time

response = requests.post(...)
if response.status_code == 429:
    # Retry with exponential backoff
    time.sleep(2 ** retry_count)
    response = requests.post(...)
```

**Timeouts (503)**:
```python
if response.status_code == 503:
    # Retry immediately or with brief delay
    time.sleep(1)
    response = requests.post(...)
```

### Batch Processing

For processing multiple queries:
```python
queries = ["What is ROS2?", "Explain Gazebo simulation", ...]
for query in queries:
    response = client.chat(query=query)
    if response.status_code == 429:
        time.sleep(30)  # Rate limit, back off
    process_response(response)
```

### Monitoring in Production

Track these metrics:
- **Success Rate**: % of 200 responses
- **Error Rate by Type**: 4xx vs 5xx
- **Average Latency**: From execution_metrics
- **P95/P99 Latency**: For SLA tracking
- **Grounding Quality**: % of responses citing sources

## Changelog

### v0.1.0 (2025-12-26)
- Initial release
- Full RAG pipeline with semantic search
- Text-only mode for custom contexts
- Comprehensive error handling
- Grounding validation
- Performance monitoring

## Support

For issues or questions:
1. Check the error code in the response
2. Review logs for detailed error messages
3. Verify environment variables are set correctly
4. Test health endpoint: `GET /health`
