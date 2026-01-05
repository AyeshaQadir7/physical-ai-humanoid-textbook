# Backend Services

This directory contains two main services:
1. **Website Ingestion Pipeline** (Spec 1): Ingests textbook content and generates embeddings
2. **RAG Agent API** (Spec 3): Query interface with semantic search and OpenAI integration

## RAG Agent API

**Query the textbook using natural language with Retrieval-Augmented Generation.**

Uses **OpenAI Agents SDK** for autonomous, intelligent agent orchestration:
- Agent automatically retrieves relevant chunks via semantic search
- Responses are grounded strictly in retrieved textbook content
- No hallucinations - all claims cite sources

### Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Set up environment
cp .env.example .env
# Edit .env with your OpenAI, Qdrant, and Cohere API keys

# Run the API
python -m uvicorn agent:app --reload --host 0.0.0.0 --port 8000

# Try it out
curl -X POST http://localhost:8000/chat \
  -H "Content-Type: application/json" \
  -d '{"query": "What is ROS2?"}'
```

### Configuration

Required environment variables (in `.env`):
```env
OPENAI_API_KEY=sk-...              # OpenAI API key
OPENAI_MODEL=gpt-3.5-turbo         # Model name (gpt-3.5-turbo or gpt-4)
QDRANT_URL=https://...qdrant.io   # Qdrant Cloud URL
QDRANT_API_KEY=...                 # Qdrant API key
COHERE_API_KEY=...                 # Cohere API key
QDRANT_COLLECTION_NAME=textbook_embeddings
LOG_LEVEL=INFO
USE_AGENTS_SDK=true                # Enable Agents SDK (default: true)
```

Optional configuration:
```env
USE_AGENTS_SDK=false               # Disable Agents SDK to use legacy Chat Completions API
```

### Documentation

- **API Reference**: See `../specs/003-rag-agent-api/AGENT_API.md`
- **Quick Start Guide**: See `../specs/003-rag-agent-api/quickstart.md`
- **Specification**: See `../specs/003-rag-agent-api/spec.md`

### Testing

```bash
# Run agent tests
pytest tests/unit/test_agent.py -v

# Run with coverage
pytest tests/unit/test_agent.py --cov=backend

# Test specific error handling
pytest tests/unit/test_agent.py::TestErrorHandlingPhase7 -v
```

### Running in Production

See `../specs/003-rag-agent-api/quickstart.md` for Docker and Gunicorn deployment.

---

## Website Ingestion Pipeline

Backend service for ingesting Docusaurus textbook content, generating embeddings, and storing vectors in Qdrant Cloud.

## Setup

```bash
cd backend
python -m venv venv
source venv/bin/activate  # or venv\Scripts\activate on Windows
pip install -r requirements.txt
```

## Configuration

Copy `.env.example` to `.env` and fill in your API credentials:

```bash
cp .env.example .env
```

Required environment variables:
- `COHERE_API_KEY`: Your Cohere production API key
- `QDRANT_URL`: Your Qdrant Cloud cluster URL
- `QDRANT_API_KEY`: Your Qdrant API key
- `TEXTBOOK_BASE_URL`: The textbook URL to ingest

## Usage

### Run full ingestion pipeline

```bash
python -m ingestion.main --base-url https://physical-ai-robotics.vercel.app
```

### Clear checkpoint and start fresh

```bash
python -m ingestion.main --clear-checkpoint
```

### Adjust log level

```bash
python -m ingestion.main --log-level DEBUG
```

## Testing

```bash
# Run all tests
pytest tests/ -v

# Run only unit tests
pytest tests/unit/ -v

# Run with coverage
pytest tests/ --cov=ingestion
```

## Project Structure

```
ingestion/
├── main.py              # Orchestrator and CLI
├── crawler.py           # URL crawling and HTML extraction
├── chunker.py           # Token-based text chunking
├── embedder.py          # Cohere embedding generation
├── qdrant_storage.py    # Qdrant collection management
├── checkpoint.py        # Resume checkpoint tracking
└── config.py            # Configuration loading

tests/
├── unit/                # Unit tests
└── integration/         # Integration tests
```

## Pipeline Stages

1. **Crawl**: Discover and fetch all internal URLs from base domain
2. **Chunk**: Split text into 512-token chunks with 50-token overlap
3. **Filter**: Skip already-processed chunks (resume capability)
4. **Embed**: Generate 1024-dimensional vectors via Cohere
5. **Store**: Upsert vectors + metadata to Qdrant Cloud
6. **Verify**: Validate vector count matches chunk count

## Configuration Options

Via environment variables in `.env`:

- `CHUNK_SIZE`: Tokens per chunk (default: 512)
- `CHUNK_OVERLAP`: Token overlap (default: 50)
- `BATCH_SIZE`: Cohere batch size (default: 96, max: 96)
- `MAX_RETRIES`: API retry attempts (default: 3)
- `COLLECTION_NAME`: Qdrant collection (default: textbook_embeddings)

## Performance

Typical 50-page book (~50k chunks):
- Crawling: ~45 seconds
- Chunking: ~12 seconds
- Embedding: ~240 seconds (Cohere API rate limits)
- Insertion: ~8 seconds
- **Total: ~5 minutes** (end-to-end)

## Troubleshooting

### "TooManyRequestsError" from Cohere
Use production API key (1000 req/min) instead of trial key (100 req/min).

### Connection timeout to Qdrant
Verify `QDRANT_URL` is correct and network connectivity is available.

### Duplicate vectors in collection
Checkpoint automatically prevents duplicates. Clear checkpoint for fresh run:
```bash
python -m ingestion.main --clear-checkpoint
```

## Architecture

**No external dependencies beyond requirements.txt**:
- Minimal, single-threaded CLI
- Resume capability via JSON checkpoint
- Deterministic deduplication via content hash
- Structured logging and error reporting

**Future**: Can wrap in FastAPI for Spec-2 (RAG Chatbot) integration.

## References

- Specification: `../specs/001-website-ingestion/spec.md`
- Implementation Plan: `../specs/001-website-ingestion/plan.md`
- Data Model: `../specs/001-website-ingestion/data-model.md`
