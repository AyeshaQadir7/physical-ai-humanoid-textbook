# RAG Agent API - Quick Start Guide

Get up and running with the RAG Agent API in 5 minutes.

## Prerequisites

- Python 3.8+
- Access to:
  - OpenAI API (GPT-3.5-turbo or GPT-4 model)
  - Qdrant Cloud or self-hosted Qdrant
  - Cohere API (for embeddings)

## Architecture

The API uses **OpenAI Agents SDK** for autonomous agent-driven retrieval:
- Agent automatically calls `retrieve_textbook_chunks` tool when needed
- Responses are grounded strictly in retrieved content
- No hallucinations - agent cites all sources

## Installation

### 1. Clone the Repository

```bash
git clone <repo-url>
cd physical-ai-robotics-textbook
```

### 2. Install Dependencies

```bash
pip install -r backend/requirements.txt
```

Required packages:
- `fastapi>=0.100.0` - Web framework
- `uvicorn[standard]>=0.23.0` - ASGI server
- `pydantic>=2.0.0` - Data validation
- `openai>=1.0.0` - OpenAI API client
- `openai-agents>=0.1.0` - **NEW**: OpenAI Agents SDK for autonomous agent orchestration
- `qdrant-client>=2.0.0` - Qdrant client
- `cohere>=4.0.0` - Cohere embeddings
- `python-dotenv>=0.19.0` - Environment variable management
- `pytest>=7.0.0` - Testing framework

### 3. Set Up Environment Variables

Create a `.env` file in the `backend/` directory:

```bash
cd backend
cp .env.example .env
```

Then edit `.env` with your credentials:

```env
# OpenAI Configuration
OPENAI_API_KEY=sk-...your-key-here...
OPENAI_MODEL=gpt-3.5-turbo

# Qdrant Configuration
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=textbook_embeddings

# Cohere Configuration
COHERE_API_KEY=your-cohere-api-key
COHERE_MODEL=embed-english-v3.0

# Logging
LOG_LEVEL=INFO
```

**Get Your API Keys**:
- OpenAI: https://platform.openai.com/account/api-keys
- Qdrant: https://cloud.qdrant.io
- Cohere: https://dashboard.cohere.com

## Running the API

### Option 1: Run with Uvicorn (Production)

```bash
cd backend
python -m uvicorn agent:app --reload --host 0.0.0.0 --port 8000
```

Output:
```
INFO:     Uvicorn running on http://0.0.0.0:8000
INFO:     Application startup complete
```

### Option 2: Run Directly (Development)

```bash
cd backend
python agent.py
```

### Verify It's Working

```bash
# Health check
curl http://localhost:8000/health

# Interactive API docs (auto-generated)
# Open in browser: http://localhost:8000/docs
```

## Your First Query

### Using cURL

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "What is ROS2?",
    "retrieval_scope": "full_collection",
    "top_k": 5
  }'
```

### Using Python

```python
import requests
import json

url = "http://localhost:8000/chat"
payload = {
    "query": "What is ROS2?",
    "retrieval_scope": "full_collection",
    "top_k": 5
}

response = requests.post(url, json=payload)
result = response.json()

print(f"Query: {result['query']}")
print(f"Answer: {result['answer']}")
print(f"Status: {result['status']}")
print(f"Latency: {result['execution_metrics']['total_time_ms']:.1f}ms")

# Print retrieved chunks
for i, chunk in enumerate(result['retrieved_chunks'], 1):
    print(f"\nChunk {i}:")
    print(f"  Source: {chunk['source_url']}")
    print(f"  Page: {chunk['page_title']}")
    print(f"  Score: {chunk['similarity_score']:.2f}")
```

### Using Python Requests (Complete Example)

```python
#!/usr/bin/env python3

import requests
import json
from typing import Optional

class RAGAgentClient:
    def __init__(self, base_url: str = "http://localhost:8000"):
        self.base_url = base_url

    def query(self, question: str, top_k: int = 5,
              context_text: Optional[str] = None) -> dict:
        """Query the RAG agent."""
        payload = {
            "query": question,
            "top_k": top_k,
            "retrieval_scope": "text_only" if context_text else "full_collection",
            "context_text": context_text
        }

        response = requests.post(
            f"{self.base_url}/chat",
            json=payload,
            timeout=30
        )
        response.raise_for_status()
        return response.json()

# Example usage
if __name__ == "__main__":
    client = RAGAgentClient()

    # Basic query
    result = client.query("What is ROS2?")

    if result['status'] == 'success':
        print(f"Answer: {result['answer']}")
        print(f"Latency: {result['execution_metrics']['total_time_ms']:.1f}ms")
    else:
        print(f"Error: {result['error']['message']}")
```

## Different Query Modes

### Full-Collection Mode (Default)

Search the entire textbook collection using semantic similarity:

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "How do I install ROS2?",
    "retrieval_scope": "full_collection",
    "top_k": 5
  }'
```

**Use when**: You want to discover information across the textbook.

### Text-Only Mode

Answer questions using only provided text snippets (no Qdrant search):

```bash
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{
    "query": "Summarize this text",
    "retrieval_scope": "text_only",
    "context_text": "ROS2 is a flexible middleware for robotics applications. It provides improved real-time performance and security compared to ROS1."
  }'
```

**Use when**: You want to test with custom content or avoid Qdrant calls.

## Testing

### Run Unit Tests

```bash
# All unit tests
pytest backend/tests/unit/ -v

# Specific test class
pytest backend/tests/unit/test_agent.py::TestChatRequest -v

# With coverage
pytest backend/tests/unit/ --cov=backend --cov-report=html
```

### Run Integration Tests (Requires Live APIs)

```bash
# These tests call real OpenAI, Qdrant, and Cohere APIs
pytest backend/tests/integration/test_agent_live.py -v
```

### Test Individual Scenarios

```python
# Test empty query error handling
import pytest
from agent import ChatRequest
from pydantic import ValidationError

def test_empty_query():
    with pytest.raises(ValidationError):
        ChatRequest(query="")

# Run it
pytest test_script.py -v
```

## Understanding Response Structure

Every response follows this structure:

```json
{
  "query": "original question",
  "answer": "generated response",
  "retrieved_chunks": [
    {
      "chunk_id": "unique_id",
      "text": "chunk content",
      "similarity_score": 0.92,
      "source_url": "https://...",
      "page_title": "Page Title",
      "section_headers": ["Section", "Subsection"]
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

- **query**: Your original question
- **answer**: The AI-generated response (grounded in retrieved chunks)
- **retrieved_chunks**: Textbook content used to generate the answer
- **execution_metrics**: Performance metrics (useful for monitoring SLAs)
- **retrieval_scope**: Which mode was used ("text_only" or "full_collection")
- **status**: "success" or "error"
- **error**: Error details (only present if status="error")

## Error Handling

### Common Errors and Solutions

**401 - Unauthorized**:
```
Error: OpenAI API key is invalid
Solution: Check OPENAI_API_KEY in .env file
```

**400 - Bad Request**:
```
Error: "Query cannot be empty"
Solution: Provide a non-empty query string
```

**429 - Rate Limited**:
```
Error: "Rate limited by generation service"
Solution: Wait a moment and retry. The API will automatically retry with backoff.
```

**503 - Service Unavailable**:
```
Error: "Generation service timeout"
Solution: OpenAI API timed out. Retry immediately or after a brief delay.
```

**500 - Internal Server Error**:
```
Error: "Retrieval service unavailable"
Solution: Check if Qdrant and Cohere services are accessible.
```

### Error Code Reference

| Code | HTTP | Meaning | Action |
|------|------|---------|--------|
| VALIDATION_ERROR | 400 | Invalid request | Fix the request |
| RETRIEVAL_FAILED | 500 | Qdrant issue | Retry after 1-5 seconds |
| AGENT_FAILED | 500 | OpenAI API issue | Retry after 1-5 seconds |
| OPENAI_TIMEOUT | 503 | Timeout | Retry immediately |
| RATE_LIMITED | 429 | Rate limit | Backoff and retry |

## Production Deployment

### Environment Setup

For production, set these variables:

```bash
OPENAI_API_KEY=sk-...
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=...
COHERE_API_KEY=...
LOG_LEVEL=WARNING  # Reduce logging verbosity
OPENAI_MODEL=gpt-3.5-turbo
```

### Running with Gunicorn

```bash
pip install gunicorn

cd backend
gunicorn -w 4 -k uvicorn.workers.UvicornWorker \
  --bind 0.0.0.0:8000 \
  --access-logfile - \
  --error-logfile - \
  agent:app
```

### Docker Deployment

```dockerfile
FROM python:3.10-slim

WORKDIR /app
COPY backend/requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

COPY backend/ .

ENV LOG_LEVEL=INFO
CMD ["uvicorn", "agent:app", "--host", "0.0.0.0", "--port", "8000"]
```

Build and run:
```bash
docker build -t rag-agent .
docker run -e OPENAI_API_KEY=$OPENAI_API_KEY \
           -e QDRANT_URL=$QDRANT_URL \
           -p 8000:8000 \
           rag-agent
```

### Monitoring

Monitor these metrics:
- **Health**: `GET /health` should return 200
- **Latency**: Check `execution_metrics.total_time_ms` (target: <5000ms)
- **Error Rate**: Track 4xx and 5xx response codes
- **Grounding**: Manually verify responses cite sources
- **Relevance**: Spot-check retrieved chunks are relevant

## Next Steps

1. **Explore the API**: Try different queries and modes
2. **Read the Full API Reference**: See `AGENT_API.md` for all endpoints and error codes
3. **Check the Logs**: Monitor logs for performance and errors
4. **Deploy**: Follow production deployment guide for real-world usage

## Troubleshooting

### API won't start

```bash
# Check Python version
python --version  # Should be 3.8+

# Check dependencies are installed
pip list | grep fastapi

# Verify .env file exists and is readable
cat backend/.env
```

### Tests are failing

```bash
# Run with verbose output
pytest backend/tests/unit/ -vv -s

# Run a single test
pytest backend/tests/unit/test_agent.py::TestChatRequest::test_chat_request_valid -vv

# Check test output for specific errors
```

### Slow responses

1. Check `execution_metrics.retrieval_time_ms`:
   - If > 500ms: Qdrant or Cohere may be slow
   - Consider using text_only mode to skip retrieval

2. Check `execution_metrics.generation_time_ms`:
   - If > 3500ms: OpenAI API may be slow
   - This is expected during high load

3. Check network latency to Qdrant Cloud:
   - Use `time curl https://your-cluster.qdrant.io/health`
   - Consider geo-location of services

## Support & Documentation

- **API Reference**: See `AGENT_API.md`
- **Architecture**: See `plan.md`
- **Specification**: See `spec.md`
- **Issues**: Check error messages and logs
- **Logs Location**: stdout when running with Uvicorn

## Additional Resources

- OpenAI API Docs: https://platform.openai.com/docs
- Qdrant Docs: https://qdrant.tech/documentation/
- Cohere Docs: https://docs.cohere.com
- FastAPI Docs: https://fastapi.tiangolo.com
