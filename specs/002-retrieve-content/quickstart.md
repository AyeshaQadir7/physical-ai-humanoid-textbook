# Quickstart: Retrieve Embedded Book Content

**Feature**: Spec 2 - Retrieve embedded book content and validate RAG pipeline
**Status**: Implementation Guide (use after code is written)

## Overview

This guide walks you through running the retrieval validation pipeline to test that your book content was successfully embedded and can be retrieved via semantic search.

**Prerequisites**:
- ✅ Spec 1 completed (Qdrant collection populated with 192+ vectors)
- ✅ `.env` file in `backend/` with API keys (from Spec 1)
- ✅ Python 3.10+ with dependencies installed

## Installation

### 1. Install/Update Dependencies

```bash
cd backend

# Install cohere SDK if not already present
pip install cohere

# Or reinstall from requirements.txt
pip install -r requirements.txt
```

### 2. Verify Environment Configuration

Ensure your `.env` file has these variables (same as Spec 1):

```bash
COHERE_API_KEY=your_cohere_api_key
COHERE_MODEL=embed-english-v3.0
QDRANT_URL=https://your-cluster.qdrant.io:6333
QDRANT_API_KEY=your_qdrant_api_key
COLLECTION_NAME=textbook_embeddings
```

Test that configuration loads:

```bash
python -c "from ingestion.config import Config; c = Config(); c.validate(); print('✓ Configuration OK')"
```

## Basic Usage

### As a Python Library

```python
from retrieve import RetrieverClient

# Initialize retriever (connects to Qdrant)
client = RetrieverClient(
    qdrant_url="https://...",
    qdrant_api_key="...",
    cohere_api_key="...",
    collection_name="textbook_embeddings"
)

# Search for content
results = client.search(
    query="What is ROS2?",
    top_k=5
)

# Print results
for i, result in enumerate(results, 1):
    print(f"\n{i}. Score: {result['similarity_score']:.3f}")
    print(f"   URL: {result['metadata']['source_url']}")
    print(f"   Title: {result['metadata']['page_title']}")
    print(f"   Text: {result['chunk_text'][:200]}...")
```

### Run Validation Tests

```bash
# Run sample validation queries
python retrieve.py --validate

# Or with custom top-k
python retrieve.py --validate --top-k 10

# With DEBUG logging
python retrieve.py --validate --log-level DEBUG
```

### Custom Queries

Create a test script:

```python
from retrieve import RetrieverClient, ValidationRunner

# Initialize
runner = ValidationRunner()

# Define queries to test
queries = [
    "What is ROS2?",
    "How do I use Gazebo for simulation?",
    "Explain Isaac Sim",
    "What are section headers in the textbook?",
    "How does reinforcement learning work?"
]

# Run validation
runner.validate_queries(queries, top_k=5)

# Results printed with relevance scores
```

## Understanding Results

Each retrieved chunk includes:

```json
{
  "chunk_text": "The actual content...",
  "similarity_score": 0.876,
  "metadata": {
    "source_url": "https://physical-ai-robotics.vercel.app/docs/...",
    "page_title": "Page Title",
    "section_headers": ["Main Section", "Subsection", "Topic"],
    "chunk_index": 0
  }
}
```

**Interpreting Scores**:
- **0.9+**: Highly relevant (excellent match)
- **0.7-0.9**: Good match (probably relevant)
- **0.5-0.7**: Moderate match (somewhat related)
- **<0.5**: Low relevance (likely false positive)

For a query like "What is ROS2?", you should see:
- ✅ Top results (score 0.8+) from "ROS2 Overview" and "Introduction" pages
- ✅ All results have metadata (URL, title, headers)
- ✅ Results ranked highest-score-first

## Running Integration Tests

```bash
# Test against live Qdrant and Cohere APIs
pytest tests/integration/test_retrieve_live.py -v

# Expected: All queries succeed, return results with metadata
```

## Running Unit Tests

```bash
# Test with mocked APIs (no real API calls)
pytest tests/unit/test_retrieve.py -v

# Expected: 100% pass rate, demonstrates logic without external dependencies
```

## Troubleshooting

### "Connection timeout to Qdrant"

```bash
# Check QDRANT_URL is correct
echo $QDRANT_URL

# Test connectivity
curl -X GET "https://your-cluster.qdrant.io:6333/collections/textbook_embeddings/exists" \
  -H "api-key: your_qdrant_api_key"
```

**Fix**: Verify URL format (should be `https://xxx.qdrant.io:6333`, not `https://xxx.qdrant.io`)

### "Invalid Cohere API key"

```bash
# Test Cohere API
python -c "import cohere; c = cohere.ClientV2('your_key'); print('✓ API OK')"
```

**Fix**: Ensure COHERE_API_KEY environment variable is set correctly

### "Collection not found"

```bash
# Verify collection exists (should return {"exists": true})
curl -X GET "https://your-cluster.qdrant.io:6333/collections/textbook_embeddings/exists" \
  -H "api-key: your_qdrant_api_key"
```

**Fix**: Run Spec 1 ingestion first to create and populate collection

### Low retrieval relevance

If most results have scores <0.7, the issue is likely:

1. **Different embedding model**: Verify both Spec 1 and Spec 2 use `embed-english-v3.0`
2. **Query too specific**: Try broader queries (e.g., "ROS2" instead of "What is ROS2 version 2.4.2?")
3. **Content not indexed**: Check that Spec 1 successfully created 192+ vectors

**Debug**:
```bash
# Print query embedding dimension
python -c "from retrieve import QueryEmbedder; e = QueryEmbedder('key'); emb = e.embed('test'); print(f'Dimension: {len(emb)}')"

# Should print: Dimension: 1024
```

## Manual Validation Checklist

After running retrieval tests, manually verify:

- [ ] Top 5 results make semantic sense for the query
- [ ] All results include non-empty metadata (URL, title, headers)
- [ ] URLs match the textbook site (physical-ai-robotics.vercel.app)
- [ ] Similarity scores are monotonically decreasing (highest to lowest)
- [ ] Execution time is <2 seconds per query
- [ ] No errors or exceptions in logs

## Performance Expectations

Typical latencies (from live Qdrant Cloud):

| Operation | Latency |
|-----------|---------|
| Cohere query embedding | 200-400ms |
| Qdrant similarity search (192 vectors) | 50-150ms |
| **Total end-to-end** | **300-600ms** |

**Note**: Qdrant Cloud Free Tier may have higher latency during peak hours. Target <2 seconds per query is achievable.

## Next Steps

Once retrieval is validated:

1. **Spec 3**: Build RAG chatbot API with response generation
   - Use `RetrieverClient` to fetch context chunks
   - Generate responses using LLM (OpenAI, Anthropic, etc.)
   - Return conversational answers with attribution

2. **Spec 4**: Deploy to production
   - Package as FastAPI service
   - Add authentication and rate limiting
   - Monitor retrieval latency and accuracy

3. **Iteration**:
   - Monitor query logs for frequently asked questions
   - Adjust chunk size or overlap if retrieval quality degrades
   - Fine-tune similarity threshold for production use

## Support

For issues, check:
1. `.env` file configuration
2. Network connectivity to Qdrant Cloud
3. Cohere API rate limits (rate limiting visible in logs)
4. `retrieval_validation.log` for detailed error messages
5. Test scripts in `tests/` directory for usage examples
