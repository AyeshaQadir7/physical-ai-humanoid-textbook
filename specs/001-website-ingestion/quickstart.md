# Quickstart: Website Ingestion & Vector Storage

**Date**: 2025-12-25
**Status**: Phase 1 - Design
**Target users**: Developers setting up the ingestion pipeline

---

## Prerequisites

### System Requirements
- **Python**: 3.10 or higher
- **OS**: Linux, macOS, or Windows (with WSL2)
- **Disk space**: ~500MB for dependencies + checkpoint files
- **Network**: Access to Vercel (textbook URL) + Cohere API + Qdrant Cloud

### External Services
1. **Cohere API Key** (production tier)
   - Sign up at https://cohere.com
   - Generate API key with sufficient quota for 50k+ embeddings
   - Estimated cost: $3-5 per full book run

2. **Qdrant Cloud Account** (free tier sufficient for testing)
   - Create cluster at https://cloud.qdrant.io
   - Note cluster URL and API key
   - Free tier: 1GB storage (~10k dense vectors)

3. **Textbook URL**
   - Published Docusaurus site (e.g., https://physical-ai-robotics.vercel.app/)
   - Must be publicly accessible (no authentication)

---

## Setup & Configuration

### 1. Create Backend Directory & Virtual Environment

```bash
# From project root
mkdir -p backend
cd backend

# Create Python virtual environment
python3 -m venv venv

# Activate environment
source venv/bin/activate  # macOS/Linux
# OR
venv\Scripts\activate     # Windows PowerShell
```

### 2. Install Dependencies

```bash
# Create requirements.txt
cat > requirements.txt << 'EOF'
requests>=2.31.0
beautifulsoup4>=4.12.0
langchain-text-splitters>=0.2.0
tiktoken>=0.5.0
cohere>=4.0.0
qdrant-client>=1.7.0
python-dotenv>=1.0.0
pytest>=7.0.0
EOF

# Install all dependencies
pip install -r requirements.txt
```

### 3. Create Environment Configuration

```bash
# Create .env file (never commit this!)
cat > .env << 'EOF'
# Cohere API
COHERE_API_KEY=your_production_key_here
COHERE_MODEL=embed-english-v3.0
BATCH_SIZE=96
MAX_RETRIES=3
REQUEST_TIMEOUT=60

# Qdrant Cloud
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_qdrant_key_here
COLLECTION_NAME=textbook_embeddings

# Chunking
CHUNK_SIZE=512
CHUNK_OVERLAP=50

# Pipeline
CHECKPOINT_FILE=ingestion_checkpoint.json
LOG_LEVEL=INFO

# Target
TEXTBOOK_BASE_URL=https://physical-ai-robotics.vercel.app
EOF

# Add to .gitignore (if not already present)
echo ".env" >> .gitignore
```

### 4. Verify Configuration

```bash
# Test Python environment
python -c "import cohere; import qdrant_client; print('✓ Dependencies installed')"

# Check environment variables
python << 'EOF'
import os
from dotenv import load_dotenv

load_dotenv()
required = ['COHERE_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY', 'TEXTBOOK_BASE_URL']
for key in required:
    if os.getenv(key):
        print(f"✓ {key} configured")
    else:
        print(f"✗ {key} missing")
EOF
```

---

## Project Structure

After setup, your backend directory should look like:

```
backend/
├── ingestion/
│   ├── __init__.py
│   ├── main.py                 # Entry point
│   ├── crawler.py              # URL crawling + text extraction
│   ├── chunker.py              # Text chunking
│   ├── embedder.py             # Cohere embedding generation
│   ├── qdrant_storage.py       # Qdrant insertion + verification
│   ├── checkpoint.py           # Resume capability
│   └── config.py               # Configuration loading
├── tests/
│   ├── unit/
│   │   ├── test_chunker.py
│   │   ├── test_embedder.py
│   │   └── test_qdrant_storage.py
│   └── integration/
│       └── test_full_pipeline.py
├── requirements.txt
├── setup.py                    # Package metadata
├── README.md                   # Detailed documentation
├── .env                        # Configuration (local only)
└── ingestion_checkpoint.json   # Checkpoint (auto-created)
```

---

## Basic Usage

### Option 1: Single File Ingestion (Recommended for Testing)

For initial testing with minimal setup, use a single combined script:

```bash
# From backend directory
python -c "
import os
from dotenv import load_dotenv

load_dotenv()

# Import ingestion modules (after implementation)
from ingestion.main import run_ingestion

# Run pipeline
result = run_ingestion(
    base_url=os.getenv('TEXTBOOK_BASE_URL'),
    collection_name=os.getenv('COLLECTION_NAME')
)

print(f'Ingestion complete: {result}')
"
```

### Option 2: Command-Line Interface (When Implemented)

```bash
# From backend directory
python -m ingestion.main \
  --base-url https://physical-ai-robotics.vercel.app \
  --collection-name textbook_embeddings \
  --chunk-size 512 \
  --log-level INFO
```

### Option 3: Python API

```python
from ingestion.main import IngestionPipeline

# Initialize
pipeline = IngestionPipeline()

# Run end-to-end
report = pipeline.run(
    base_url="https://physical-ai-robotics.vercel.app",
    collection_name="textbook_embeddings",
    clear_checkpoint=False  # Resume from checkpoint if exists
)

# Access results
print(f"Chunks created: {report['summary']['total_chunks_created']}")
print(f"Success rate: {report['summary']['insertion_success_rate']:.1%}")
```

---

## Example Workflow

### Step 1: Small Test Run (Single Page)

```bash
cd backend

# Ingest just one URL to test pipeline
python << 'EOF'
from ingestion.main import IngestionPipeline

pipeline = IngestionPipeline()
report = pipeline.run(
    base_url="https://physical-ai-robotics.vercel.app",
    max_urls=1,  # Only crawl 1 page
    collection_name="test_embeddings"
)

print(f"✓ Created {report['summary']['total_chunks_created']} chunks")
EOF
```

### Step 2: Verify Collections Created

```bash
python << 'EOF'
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# List collections
collections = client.get_collections()
for collection in collections.collections:
    print(f"✓ {collection.name}: {collection.points_count} points")
EOF
```

### Step 3: Full Book Ingestion

```bash
cd backend

# Clear checkpoint to force fresh run
rm -f ingestion_checkpoint.json

# Run full ingestion
python -m ingestion.main \
  --base-url https://physical-ai-robotics.vercel.app \
  --collection-name textbook_embeddings \
  --log-level INFO

# Monitor output for progress
```

### Step 4: Verify Results

```bash
python << 'EOF'
from ingestion.qdrant_storage import QdrantVerifier
import os
from dotenv import load_dotenv

load_dotenv()

verifier = QdrantVerifier(os.getenv("QDRANT_URL"), os.getenv("QDRANT_API_KEY"))

# Get collection statistics
stats = verifier.verify_collection("textbook_embeddings")
print(f"✓ Total points: {stats['points_count']}")
print(f"✓ Status: {stats['status']}")

# Sample similarity search
results = verifier.test_search(
    collection_name="textbook_embeddings",
    sample_size=5
)
print(f"✓ Sample queries successful")
EOF
```

---

## Common Tasks

### Monitor Ingestion Progress

```bash
# Watch checkpoint file (updates every batch)
watch -n 5 'tail ingestion_checkpoint.json | jq .'
```

### Resume from Failure

```bash
# Checkpoint automatically saved; just re-run
# Pipeline will skip already-processed chunks
python -m ingestion.main \
  --base-url https://physical-ai-robotics.vercel.app \
  --collection-name textbook_embeddings
```

### Clear and Start Fresh

```bash
# Delete checkpoint to force complete re-run
rm ingestion_checkpoint.json

# Optional: Delete collection in Qdrant and recreate
python << 'EOF'
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()

client = QdrantClient(
    url=os.getenv("QDRANT_URL"),
    api_key=os.getenv("QDRANT_API_KEY")
)

# Delete collection
client.delete_collection("textbook_embeddings")
print("✓ Collection deleted")
EOF

# Re-run pipeline
python -m ingestion.main --base-url https://physical-ai-robotics.vercel.app
```

### Run Tests

```bash
# Run all tests
pytest tests/ -v

# Run only unit tests
pytest tests/unit/ -v

# Run integration tests (requires live API keys)
pytest tests/integration/ -v

# Run with coverage
pytest tests/ --cov=ingestion --cov-report=html
```

### Adjust Chunking Parameters

```bash
# Override defaults via environment variables
export CHUNK_SIZE=256
export CHUNK_OVERLAP=25

# Re-run ingestion
python -m ingestion.main --base-url https://physical-ai-robotics.vercel.app
```

---

## Troubleshooting

### Issue: "TooManyRequestsError" from Cohere

**Cause**: Rate limit hit (trial key allows 100 requests/minute)

**Solution**:
```bash
# Use production API key instead of trial
# Production keys have 1000 req/min limit

# Verify in .env:
cat .env | grep COHERE_API_KEY

# Or reduce batch size:
export BATCH_SIZE=32  # Instead of 96
```

### Issue: "Connection timeout" to Qdrant

**Cause**: Qdrant cluster unavailable or slow network

**Solution**:
```bash
# Test connection
python << 'EOF'
from qdrant_client import QdrantClient
import os
from dotenv import load_dotenv

load_dotenv()

try:
    client = QdrantClient(
        url=os.getenv("QDRANT_URL"),
        api_key=os.getenv("QDRANT_API_KEY")
    )
    info = client.get_collection("test")
    print("✓ Qdrant connected successfully")
except Exception as e:
    print(f"✗ Connection failed: {e}")
EOF

# If connection fails, check:
# 1. QDRANT_URL is correct (https://xyz.qdrant.io)
# 2. QDRANT_API_KEY is valid
# 3. Network connectivity (ping qdrant.io)
```

### Issue: "Memory error" or slow performance

**Cause**: Batch size too large or chunk size too big

**Solution**:
```bash
# Reduce batch size
export BATCH_SIZE=32

# Or reduce chunk size
export CHUNK_SIZE=256

# Re-run pipeline
```

### Issue: Duplicate vectors in collection

**Cause**: Re-ran ingestion without deleting checkpoint

**Solution**:
```bash
# Checkpoint tracks already-embedded chunks
# Re-running automatically skips duplicates (idempotent)

# To force complete replacement:
rm ingestion_checkpoint.json
curl -X DELETE "https://your-cluster.qdrant.io/collections/textbook_embeddings" \
  -H "api-key: your_key"

# Then re-run:
python -m ingestion.main --base-url https://physical-ai-robotics.vercel.app
```

---

## Performance Expectations

### Typical Metrics (50-page book, 50k chunks)

| Stage | Duration | Notes |
|-------|----------|-------|
| **Crawling** | 45 seconds | Fetch + HTML parsing |
| **Chunking** | 12 seconds | Token-based splitting |
| **Embedding** | 240 seconds | Cohere API batching (96 chunks/req) |
| **Insertion** | 8 seconds | Qdrant batch upsert |
| **Verification** | 5 seconds | Collection stats + sample search |
| **Total** | ~5 minutes | End-to-end (without network delays) |

### Factors Affecting Speed

- **Network latency**: Major factor; typical 2-5s per Cohere batch request
- **Chunk size**: Larger chunks → fewer API calls but slower extraction
- **Batch size**: 96 (max) is optimal for throughput
- **Re-runs**: Checkpoint enables much faster incremental updates

---

## Next Steps

1. **Implement core modules** (crawler.py, chunker.py, embedder.py, etc.)
2. **Write unit tests** for each module
3. **Test with live APIs** (start small: 1 page)
4. **Scale to full book** (monitor performance)
5. **Integrate with CI/CD** (automate re-indexing on content updates)
6. **Build FastAPI wrapper** (for future RAG chatbot feature)

---

## Reference Documentation

- **Implementation Plan**: `specs/001-website-ingestion/plan.md`
- **Feature Specification**: `specs/001-website-ingestion/spec.md`
- **Data Model**: `specs/001-website-ingestion/data-model.md`
- **Research & Best Practices**: `specs/001-website-ingestion/research.md`

---

## Support & Issues

For bugs, questions, or feature requests, open an issue or contact the team.

**Key contacts**:
- Specification questions: See `specs/001-website-ingestion/spec.md`
- Architecture questions: See `specs/001-website-ingestion/plan.md`
- Implementation help: See individual module docstrings
