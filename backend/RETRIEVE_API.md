# Retrieval API Documentation

## Overview

The retrieval module (`backend/retrieve.py`) provides a complete implementation for querying the Qdrant vector database with natural language queries using Cohere embeddings. It's designed to validate the RAG (Retrieval-Augmented Generation) pipeline by retrieving relevant textbook content chunks.

**Status**: Production-ready for validation and testing
**Language**: Python 3.10+
**Dependencies**: Cohere SDK, Qdrant Python client

---

## Quick Start

### Installation

```bash
cd backend
pip install -r requirements.txt
```

### Basic Usage

```python
from retrieve import ValidationRunner

# Initialize retriever
runner = ValidationRunner()

# Run validation with default queries
responses = runner.validate_queries(top_k=5)

# Or search for a specific query
response = runner.client.search(
    query="What is ROS2?",
    top_k=5
)

# Print results
for result in response["results"]:
    print(f"Score: {result['similarity_score']:.3f}")
    print(f"URL: {result['metadata']['source_url']}")
    print(f"Text: {result['chunk_text'][:200]}...")
```

### CLI Usage

```bash
# Run validation with sample queries
python retrieve.py --validate

# Search for a custom query
python retrieve.py --query "How do I use Gazebo simulation?" --top-k 10

# Enable debug logging
python retrieve.py --validate --log-level DEBUG

# Save results to JSON
python retrieve.py --validate --output results.json

# Custom query with output
python retrieve.py --query "ROS2" --top-k 5 --output search_results.json
```

---

## Core Classes

### QueryEmbedder

Embeds natural language queries using Cohere API.

**Methods**:
- `embed_query(query: str) -> List[float]`
  - Embeds a search query (1024-dimensional vector)
  - Implements retry logic with exponential backoff
  - Raises `ValueError` for empty queries
  - Raises `RuntimeError` if Cohere API fails

**Example**:
```python
from retrieve import QueryEmbedder

embedder = QueryEmbedder(
    api_key="your-cohere-api-key",
    model="embed-english-v3.0"
)

embedding = embedder.embed_query("What is ROS2?")
# Returns: [0.123, 0.456, ..., 0.789] (1024 values)
```

### RetrieverClient

Queries Qdrant collection for relevant content chunks.

**Methods**:
- `search(query: str, top_k: int = 5, similarity_threshold: float = 0.0) -> Dict`
  - Searches for relevant chunks
  - Parameters:
    - `query`: Natural language search query
    - `top_k`: Number of results (1-100, default 5)
    - `similarity_threshold`: Minimum relevance score (0.0-1.0, default 0.0)
  - Returns structured response with results and metrics

- `validate_metadata(results: List[Dict]) -> Dict`
  - Validates metadata integrity in search results
  - Checks for required fields: source_url, page_title, section_headers, chunk_index
  - Returns validation report

**Example**:
```python
from retrieve import RetrieverClient

client = RetrieverClient(
    qdrant_url="https://your-cluster.qdrant.io:6333",
    qdrant_api_key="your-api-key",
    cohere_api_key="your-cohere-key",
    collection_name="textbook_embeddings"
)

# Search
response = client.search(
    query="What is ROS2?",
    top_k=5,
    similarity_threshold=0.5
)

# Validate metadata
report = client.validate_metadata(response["results"])
print(f"Valid results: {report['valid_results']}/{report['total_results']}")
```

### ValidationRunner

Executes validation queries and provides formatted output.

**Methods**:
- `validate_queries(queries: List[str] | None = None, top_k: int = 5) -> List[Dict]`
  - Runs validation with provided or default queries
  - Returns list of search responses
  - Logs results to `retrieval_validation.log`

**Example**:
```python
from retrieve import ValidationRunner

runner = ValidationRunner()

# Default queries
responses = runner.validate_queries(top_k=5)

# Custom queries
custom_queries = [
    "What is ROS2?",
    "How do I set up Isaac Sim?",
    "Explain reinforcement learning"
]
responses = runner.validate_queries(queries=custom_queries, top_k=10)

for response in responses:
    print(f"Query: {response['query']['text']}")
    print(f"Results: {response['total_results']}")
    print(f"Time: {response['execution_metrics']['total_execution_time_ms']}ms")
```

---

## Response Format

### Search Response (Success)

```json
{
  "status": "success",
  "query": {
    "text": "What is ROS2?"
  },
  "results": [
    {
      "chunk_id": "a7f3e2b9c1d4e8f2...",
      "chunk_text": "ROS2 (Robot Operating System 2) is...",
      "similarity_score": 0.876,
      "rank": 1,
      "metadata": {
        "source_url": "https://physical-ai-robotics.vercel.app/docs/module-1-ros2/overview",
        "page_title": "Module 1: ROS2 Overview",
        "section_headers": ["Module 1: ROS2", "Introduction"],
        "chunk_index": 0
      }
    }
  ],
  "total_results": 5,
  "requested_top_k": 5,
  "execution_metrics": {
    "query_embedding_time_ms": 234.5,
    "vector_search_time_ms": 87.3,
    "total_execution_time_ms": 321.8,
    "embedding_model": "embed-english-v3.0",
    "collection_name": "textbook_embeddings"
  }
}
```

### Search Response (Error)

```json
{
  "status": "error",
  "query": {
    "text": "What is ROS2?"
  },
  "results": [],
  "total_results": 0,
  "requested_top_k": 5,
  "execution_metrics": {
    "total_execution_time_ms": 45.2,
    "embedding_model": "embed-english-v3.0",
    "collection_name": "textbook_embeddings"
  },
  "error": {
    "code": "QDRANT_CONNECTION_FAILED",
    "message": "Failed to connect to Qdrant Cloud: Connection timeout after 30s"
  }
}
```

---

## Configuration

Configuration is loaded from environment variables via `.env` file:

```bash
# Cohere API
COHERE_API_KEY=your_cohere_api_key
COHERE_MODEL=embed-english-v3.0

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key
COLLECTION_NAME=textbook_embeddings

# Optional
LOG_LEVEL=INFO  # DEBUG, INFO, WARNING, ERROR
```

All configuration is reused from the ingestion pipeline (Spec 1).

---

## Testing

### Unit Tests (Mocked APIs)

```bash
pytest backend/tests/unit/test_retrieve.py -v
```

Tests cover:
- QueryEmbedder: embedding generation, error handling, retries
- RetrieverClient: search logic, metadata validation, parameter validation
- ValidationRunner: initialization, query execution

All tests use mocked Cohere and Qdrant APIs.

### Integration Tests (Live APIs)

```bash
# Requires valid .env with API keys and populated collection
pytest backend/tests/integration/test_retrieve_live.py -v

# Run specific test
pytest backend/tests/integration/test_retrieve_live.py::TestRetrievalLiveAPIs::test_single_query_search -v
```

Tests cover:
- Live API connectivity
- Result quality and metadata
- Performance metrics (<2s per query)
- Edge cases (empty results, special characters)
- Result serialization

### Test Coverage

```bash
pytest backend/tests/ --cov=retrieve --cov-report=html
```

---

## Performance

Typical latencies (Qdrant Cloud Free Tier):

| Operation | Latency |
|-----------|---------|
| Query embedding (Cohere) | 200-400ms |
| Vector search (Qdrant) | 50-150ms |
| **Total (end-to-end)** | **300-600ms** |

**Target**: <2 seconds per query (SLA from spec)

### Performance Tuning

1. **Batch queries** to amortize API overhead
2. **Increase top_k gradually** to find diminishing returns
3. **Monitor logs** for embedding/search bottlenecks
4. **Use similarity_threshold** to filter low-relevance results

---

## Logging

Logs are written to both console and file (`backend/retrieval_validation.log`).

**Log Levels**:
- `DEBUG`: Detailed API calls, embeddings, query traces
- `INFO`: Query execution, result summaries, timing
- `WARNING`: Retries, API delays, partial failures
- `ERROR`: Failed queries, connection errors, validation failures

**Example**:
```
2025-12-26 01:23:26,059 - retrieve - INFO - RetrieverClient initialized for collection: textbook_embeddings
2025-12-26 01:23:26,150 - retrieve - INFO - Searching for: What is ROS2?
2025-12-26 01:23:26,400 - retrieve - INFO - Query embedded successfully: 1024 dimensions
2025-12-26 01:23:26,500 - retrieve - INFO - Search complete: 5 results in 0.350s (embed: 0.250s, search: 0.100s)
```

---

## Error Handling

The module handles errors gracefully with structured error responses:

| Error | Code | Message | Handling |
|-------|------|---------|----------|
| Empty query | ValueError | "Query cannot be empty" | Validation |
| Cohere API timeout | RuntimeError | "Failed to embed query after 3 attempts" | Retry + Error response |
| Qdrant connection failure | RuntimeError | "Collection not found in Qdrant" | Error response |
| Invalid top_k | ValueError | "top_k must be between 1 and 100" | Validation |

**Error Response Structure**:
```python
{
    "status": "error",
    "error": {
        "code": "ERROR_CODE",
        "message": "Human-readable error description"
    }
}
```

---

## Usage Examples

### Example 1: Single Query Search

```bash
python retrieve.py --query "How do I use Gazebo?" --top-k 3
```

### Example 2: Batch Validation

```python
from retrieve import ValidationRunner
import json

runner = ValidationRunner()
responses = runner.validate_queries(top_k=5)

# Analyze results
for response in responses:
    if response["status"] == "success":
        print(f"Query: {response['query']['text']}")
        for result in response["results"]:
            print(f"  - {result['metadata']['page_title']} ({result['similarity_score']:.2f})")
```

### Example 3: Metadata Validation

```python
from retrieve import RetrieverClient

client = RetrieverClient(
    qdrant_url="https://...",
    qdrant_api_key="...",
    cohere_api_key="..."
)

response = client.search("ROS2", top_k=10)

if response["status"] == "success":
    report = client.validate_metadata(response["results"])
    if report["invalid_results"] == 0:
        print("✓ All metadata valid!")
    else:
        print(f"✗ {report['invalid_results']} invalid results:")
        for issue in report["issues"]:
            print(f"  - {issue}")
```

### Example 4: Save Results for Analysis

```bash
python retrieve.py --validate --output validation_results.json

# Then analyze with Python
import json
with open("validation_results.json") as f:
    results = json.load(f)

for response in results:
    avg_score = sum(r["similarity_score"] for r in response["results"]) / len(response["results"])
    print(f"{response['query']['text']}: avg score {avg_score:.3f}")
```

---

## FAQ

**Q: Why are my results not relevant?**
A: Check that:
1. Spec 1 ingestion completed successfully (192+ vectors in collection)
2. Using same embedding model (`embed-english-v3.0`)
3. Query is in English and reasonably specific
4. Collection is populated with expected content

**Q: How do I improve retrieval quality?**
A:
1. Adjust `top_k` to see more candidates
2. Lower `similarity_threshold` to be less strict
3. Try rephrasing queries (short vs. long, specific vs. general)
4. Check chunk quality in source ingestion

**Q: Can I use different embedding models?**
A: Not in this version. Spec 2 requires using the same model as Spec 1 (`embed-english-v3.0`). Changing models would require re-embedding the entire collection.

**Q: What's the maximum query length?**
A: Cohere model supports up to ~1000 characters. Longer queries are truncated to "END".

---

## Related

- **Spec 1**: Website Ingestion & Vector Storage Pipeline (creates the Qdrant collection)
- **Spec 3**: RAG Chatbot API (uses this retrieval module for context)
- **Data Model**: `specs/002-retrieve-content/data-model.md`
- **Quickstart**: `specs/002-retrieve-content/quickstart.md`
- **Full Specification**: `specs/002-retrieve-content/spec.md`
