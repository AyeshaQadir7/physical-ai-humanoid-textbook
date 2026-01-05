# Research: Website Ingestion & Vector Storage Pipeline

**Date**: 2025-12-25
**Branch**: `001-website-ingestion`
**Status**: Complete - All NEEDS CLARIFICATION items resolved

## Overview

This document consolidates Phase 0 research findings into actionable decisions for the implementation plan. All technical unknowns have been resolved through library research, best practices analysis, and vendor documentation review.

---

## 1. HTML Parsing & Text Extraction

### Decision: BeautifulSoup for Static Content

**What was chosen**: BeautifulSoup 4 with targeted `<article>` tag selection + element blacklist approach

**Rationale**:
- Docusaurus sites are static HTML post-build; no JavaScript rendering needed
- BeautifulSoup is 3-6x faster than Selenium/Playwright for parsing
- Minimal resource overhead (critical for high-volume scraping)
- Simple API and excellent HTML parsing robustness

**Alternatives considered**:
- **Selenium**: Overkill for static content; high memory/CPU overhead; slow
- **Playwright**: Modern but heavier than needed; better for dynamic sites
- **Manual regex**: Error-prone; doesn't handle malformed HTML well

**Implementation**:

```python
from bs4 import BeautifulSoup, SoupStrainer
import html

def extract_docusaurus_content(html_content: str) -> str:
    """Extract clean text from Docusaurus HTML."""
    soup = BeautifulSoup(html_content, 'html.parser')

    # Target main content container (Docusaurus uses <article>)
    article = soup.find('article') or soup.find('main')
    if not article:
        return ""

    # Remove boilerplate: navigation, scripts, styles
    for tag in article(['script', 'style', 'nav', 'header', 'footer']):
        tag.decompose()

    # Extract and clean text
    text = article.get_text(separator=' ', strip=True)

    # Unescape HTML entities (&nbsp;, &lt;, &amp;, etc.)
    return html.unescape(text)
```

**Dependencies**: `beautifulsoup4>=4.12.0`, `requests>=2.31.0`

---

## 2. Text Chunking Strategy

### Decision: Token-Based Chunking with LangChain + tiktoken

**What was chosen**: RecursiveCharacterTextSplitter from LangChain with tiktoken encoding

**Configuration** (matches your spec):
- **Chunk size**: 512 tokens
- **Chunk overlap**: 50 tokens (10% overlap)
- **Tokenizer**: cl100k_base (GPT-3.5/4 compatible; works with Cohere embeddings)
- **Separator hierarchy**: `["\n\n", "\n", ". ", " ", ""]` (preserves paragraph/sentence structure)

**Rationale**:
- Token-based chunking aligns with embedding model limits and semantic boundaries
- Character-based chunking can split mid-word, breaking semantic coherence
- 512 tokens ≈ 1000-1500 characters ≈ 2-3 paragraphs (optimal for educational content)
- 10% overlap ensures key sentences split across chunks appear in both (maintains continuity)
- LangChain's RecursiveCharacterTextSplitter is industry-standard for RAG pipelines
- tiktoken is 3-6x faster than alternative tokenizers

**Alternatives considered**:
- **Character-based**: Simpler but less semantic; can split words
- **Sentence-based**: Too granular; loses context
- **spaCy tokenization**: More sophisticated but slower; not needed for this use case
- **NLTK**: Outdated; poor performance vs modern alternatives

**Implementation**:

```python
from langchain_text_splitters import RecursiveCharacterTextSplitter
import tiktoken

class TextChunker:
    def __init__(self, chunk_size: int = 512, chunk_overlap: int = 50):
        self.splitter = RecursiveCharacterTextSplitter.from_tiktoken_encoder(
            encoding_name="cl100k_base",
            chunk_size=chunk_size,
            chunk_overlap=chunk_overlap,
            separators=["\n\n", "\n", ". ", " ", ""]
        )
        self.encoder = tiktoken.get_encoding("cl100k_base")

    def chunk_text(self, text: str) -> list[dict]:
        """Split text into chunks with token count metadata."""
        chunk_texts = self.splitter.split_text(text)

        chunks = []
        for idx, chunk_text in enumerate(chunk_texts):
            token_count = len(self.encoder.encode(chunk_text))
            chunks.append({
                "text": chunk_text,
                "token_count": token_count,
                "index": idx
            })

        return chunks
```

**Dependencies**: `langchain-text-splitters>=0.2.0`, `tiktoken>=0.5.0`

**Validation**: Chunks must be within ±10% of 512 tokens (SC-003 from spec)

---

## 3. Embedding Generation

### Decision: Cohere embed-english-v3.0 with Production API Key

**What was chosen**: Cohere embed-english-v3.0 model with batch processing and exponential backoff

**Model specifications**:
- **Dimensions**: 1024 (high-quality semantic retrieval)
- **Input type**: "search_document" (critical for semantic accuracy)
- **Batch size**: 96 texts per request (maximum allowed)
- **Rate limits**: Requires production API key (1000 req/min vs 100 req/min on trial)
- **Cost**: ~$3-5 for 50k chunks

**Rationale**:
- embed-english-v3.0 is Cohere's recommended model for RAG/semantic search
- 1024 dimensions provide high-quality embeddings for text retrieval
- Production key essential: 50k chunks at trial limits would take 8+ hours; production allows ~10 minutes
- Batching (96 per request) minimizes API call overhead
- "search_document" input type critical for semantic accuracy in retrieval tasks

**Alternatives considered**:
- **embed-english-light-v3.0**: Faster (384 dims) but slightly lower retrieval quality; consider if performance issues arise
- **embed-english-v4.0**: Newer, supports images/mixed input; not needed for text-only textbook
- **OpenAI embeddings**: Different ecosystem; constitution specifies Cohere

**Implementation**:

```python
import cohere
import time
from typing import List

class CohereEmbedder:
    def __init__(self, api_key: str, model: str = "embed-english-v3.0"):
        self.client = cohere.ClientV2(api_key=api_key)
        self.model = model

    def embed_chunks(
        self,
        chunks: List[str],
        batch_size: int = 96,
        max_retries: int = 3
    ) -> List[List[float]]:
        """Generate embeddings with batching and exponential backoff."""
        all_embeddings = []

        for i in range(0, len(chunks), batch_size):
            batch = chunks[i:i + batch_size]

            for attempt in range(max_retries):
                try:
                    response = self.client.embed(
                        model=self.model,
                        texts=batch,
                        input_type="search_document",  # Critical!
                        embedding_types=["float"],
                        truncate="END"  # Handle long texts gracefully
                    )

                    all_embeddings.extend(response.embeddings.float)
                    break  # Success - exit retry loop

                except cohere.errors.TooManyRequestsError:
                    wait = 2 ** attempt  # Exponential backoff: 1s, 2s, 4s
                    time.sleep(wait)

        return all_embeddings
```

**Dependencies**: `cohere>=4.0.0`

**Configuration** (via environment variables):
```
COHERE_API_KEY=your_production_key_here
COHERE_MODEL=embed-english-v3.0
BATCH_SIZE=96
MAX_RETRIES=3
```

**Success criteria**:
- ≥99% embedding success rate (FR-006)
- Graceful handling of rate limits with exponential backoff (FR-013)
- Support for content deduplication (only embed once per content)

---

## 4. Vector Storage & Retrieval

### Decision: Qdrant Cloud with Content-Hash Deduplication

**What was chosen**: Qdrant Cloud with payload-based metadata + content-hash-based deduplication

**Collection schema**:

```python
from qdrant_client import QdrantClient, models

# Create collection
client.create_collection(
    collection_name="textbook_embeddings",
    vectors_config=models.VectorParams(
        size=1024,  # Match Cohere embed-english-v3.0
        distance=models.Distance.COSINE
    )
)

# Payload schema for each point
payload_template = {
    "source_url": "https://example.com/page",        # Source (indexed: keyword)
    "page_title": "Chapter Title",                    # Page title
    "section_headers": ["Module 1", "Chapter 2"],    # Hierarchical headers
    "chunk_text": "The actual content...",            # Chunk text
    "chunk_id": "sha256_hash_of_content",           # Content hash (indexed: keyword)
    "chunk_index": 0,                                # Position in document
    "timestamp": "2025-12-25T10:00:00Z"             # Ingestion timestamp (indexed: datetime)
}
```

**Deduplication strategy** (idempotent inserts):

```python
import hashlib
import uuid

def generate_point_id(text: str, url: str) -> str:
    """Generate deterministic UUID from content hash."""
    content = f"{url}:{text}"
    hash_hex = hashlib.sha256(content.encode()).hexdigest()
    return str(uuid.UUID(hash_hex[:32]))

# Upsert behavior: Same content hash → same point ID → automatic replacement
# This prevents duplicate vectors without explicit deduplication queries
client.upsert(
    collection_name="textbook_embeddings",
    points=[
        models.PointStruct(
            id=generate_point_id(chunk_text, source_url),
            vector=embedding,
            payload={
                "source_url": source_url,
                "page_title": title,
                "section_headers": headers,
                "chunk_text": chunk_text,
                "chunk_id": hashlib.sha256(chunk_text.encode()).hexdigest(),
                "chunk_index": idx,
                "timestamp": datetime.utcnow().isoformat() + "Z"
            }
        )
    ]
)
```

**Rationale**:
- Qdrant Cloud: Fully managed vector database (no infrastructure burden)
- COSINE distance: Standard for text embeddings; cosine similarity measures semantic closeness
- Content-hash IDs: Deterministic deduplication without separate queries
- Metadata indexing: Enables filtering by URL, section headers, timestamp
- Checkpoint system: JSON file tracks processed content hashes for resume capability

**Alternatives considered**:
- **Self-hosted Qdrant**: More control but requires ops; spec requires Cloud
- **Pinecone**: Similar to Qdrant but different API/metadata model
- **Weaviate**: Open-source but more complex setup
- **Milvus**: Scalable but over-engineered for this use case

**Verification strategy**:

```python
# After insertion, verify counts match
info = client.get_collection("textbook_embeddings")
assert info.points_count >= expected_count, f"Insertion failed: {info.points_count} < {expected_count}"

# Verify specific URL was indexed
results = client.scroll(
    collection_name="textbook_embeddings",
    scroll_filter=models.Filter(
        must=[models.FieldCondition(key="source_url", match=models.MatchValue(value=url))]
    ),
    limit=10000
)
chunk_count = len(results[0])
print(f"URL '{url}' has {chunk_count} chunks indexed")
```

**Dependencies**: `qdrant-client>=1.7.0`

**Configuration** (via environment variables):
```
QDRANT_URL=https://your-cluster.qdrant.io
QDRANT_API_KEY=your_api_key_here
COLLECTION_NAME=textbook_embeddings
```

---

## 5. Configuration & Checkpointing

### Decision: Environment Variables + JSON Checkpoint File

**Configuration approach**:
- All secrets (API keys, URLs) via environment variables (`.env` file)
- Configuration via environment variables (chunking size, batch size, retry limits)
- No hardcoded values in code

**Checkpoint system**:
- JSON file tracking processed chunk hashes
- Enables resume from failure without re-embedding/inserting
- Cleared on fresh runs; appended on re-runs

**Implementation**:

```python
from dotenv import load_dotenv
import os
import json
from pathlib import Path
from dataclasses import dataclass

load_dotenv()

@dataclass
class Config:
    # API Keys (from .env)
    cohere_api_key: str = os.getenv("COHERE_API_KEY")
    qdrant_url: str = os.getenv("QDRANT_URL")
    qdrant_api_key: os.getenv("QDRANT_API_KEY")

    # Chunking parameters
    chunk_size: int = int(os.getenv("CHUNK_SIZE", "512"))
    chunk_overlap: int = int(os.getenv("CHUNK_OVERLAP", "50"))

    # Embedding parameters
    batch_size: int = int(os.getenv("BATCH_SIZE", "96"))
    max_retries: int = int(os.getenv("MAX_RETRIES", "3"))

    # Checkpoint file
    checkpoint_file: str = os.getenv("CHECKPOINT_FILE", "ingestion_checkpoint.json")


class CheckpointManager:
    def __init__(self, checkpoint_file: str):
        self.checkpoint_file = Path(checkpoint_file)
        self.processed_hashes = self._load()

    def _load(self) -> set:
        if self.checkpoint_file.exists():
            with open(self.checkpoint_file) as f:
                data = json.load(f)
                return set(data.get("processed_hashes", []))
        return set()

    def mark_processed(self, chunk_hash: str):
        self.processed_hashes.add(chunk_hash)
        self._save()

    def _save(self):
        with open(self.checkpoint_file, 'w') as f:
            json.dump({"processed_hashes": list(self.processed_hashes)}, f)
```

**Dependencies**: `python-dotenv>=1.0.0`

---

## 6. Error Handling & Resilience

### Decision: Exponential Backoff with Jitter + Structured Logging

**HTTP/API error handling**:

```python
import logging
import time
import random
from typing import Callable, Any

logger = logging.getLogger(__name__)

def exponential_backoff_retry(
    func: Callable,
    max_attempts: int = 5,
    base_delay: float = 1.0,
    max_delay: float = 300.0
) -> Any:
    """Retry with exponential backoff and jitter."""
    for attempt in range(max_attempts):
        try:
            return func()
        except Exception as e:
            if attempt == max_attempts - 1:
                raise

            delay = min(base_delay * (2 ** attempt), max_delay)
            jitter = random.uniform(0, 1)
            wait = delay + jitter

            logger.warning(f"Attempt {attempt + 1} failed: {e}. Retrying in {wait:.1f}s...")
            time.sleep(wait)
```

**Logging structure**:

```python
import json
import logging

# Configure structured logging
logging.basicConfig(
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    level=logging.INFO
)

# Example log entries
logger.info(json.dumps({
    "stage": "crawling",
    "status": "success",
    "urls_crawled": 25,
    "duration_seconds": 42.5
}))

logger.error(json.dumps({
    "stage": "embedding",
    "status": "failed",
    "chunk_id": "abc123",
    "error": "TooManyRequestsError",
    "retry_count": 3
}))
```

---

## Summary Table: Technology Stack

| Component | Technology | Rationale |
|-----------|-----------|-----------|
| **HTML Parsing** | BeautifulSoup 4 | Fast, robust, minimal overhead; ideal for static Docusaurus |
| **Tokenization** | tiktoken (cl100k_base) | 3-6x faster than alternatives; aligns with Cohere |
| **Text Chunking** | LangChain RecursiveCharacterTextSplitter | Industry standard; respects semantic boundaries |
| **Chunk Size** | 512 tokens, 10% overlap | Optimal for educational content; matches spec |
| **Embeddings** | Cohere embed-english-v3.0 | High-quality 1024D vectors; production rate limits |
| **Vector DB** | Qdrant Cloud | Managed service; excellent metadata support |
| **Deduplication** | Content-hash (SHA256) → UUID | Idempotent; prevents re-embedding duplicates |
| **Configuration** | Environment variables + .env | Secure; no hardcoded secrets |
| **Checkpointing** | JSON file | Simple; enables resume on failure |
| **Error Handling** | Exponential backoff + structured logging | Resilient; observable |
| **Testing** | pytest | Unit + integration tests against live services |

---

## Next Steps: Phase 1 Design

All research complete. Ready to proceed to Phase 1 (design artifacts):

1. **data-model.md**: Entity definitions (Chunk, Embedding, Collection, Checkpoint)
2. **contracts/openapi.yaml**: API contracts (placeholder for future FastAPI)
3. **quickstart.md**: Setup, configuration, and execution instructions
4. **Agent context update**: Add new technology to `.claude/agent-context.md`

---

## References

- [BeautifulSoup Documentation](https://www.crummy.com/software/BeautifulSoup/bs4/doc/)
- [LangChain Text Splitters](https://python.langchain.com/docs/how_to/recursive_text_splitter/)
- [Cohere Embed API](https://docs.cohere.com/docs/embeddings)
- [Qdrant Python Client](https://python-client.qdrant.tech/)
- [tiktoken Documentation](https://github.com/openai/tiktoken)
