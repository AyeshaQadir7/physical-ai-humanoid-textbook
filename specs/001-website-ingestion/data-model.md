# Data Model: Website Ingestion & Vector Storage

**Date**: 2025-12-25
**Status**: Phase 1 - Design
**References**: `spec.md` (FR-001 through FR-016), `research.md` (technology stack)

---

## Overview

This document defines the core data entities for the ingestion pipeline: how content is extracted, chunked, embedded, stored, and verified.

---

## 1. Page (Document)

**Definition**: A single publicly-accessible URL from the textbook website.

**Attributes**:

| Attribute | Type | Source | Validation | Example |
|-----------|------|--------|-----------|---------|
| `url` | string (URL) | Target Vercel domain | Must be HTTPS; valid URI | `https://textbook.vercel.app/module-1/chapter-2` |
| `title` | string | HTML `<title>` or `<h1>` tag | Non-empty; ≤256 chars | `Introduction to Robot Kinematics` |
| `raw_html` | string | HTTP GET response | Non-empty; UTF-8 encoded | (HTML document) |
| `extracted_text` | string | BeautifulSoup `<article>` extraction | Non-empty; HTML entities unescaped | (clean plaintext) |
| `fetch_timestamp` | ISO 8601 datetime | System clock | UTC; RFC 3339 format | `2025-12-25T10:30:00Z` |
| `fetch_duration_seconds` | float | HTTP request timing | ≥0; ≤300 | `2.45` |
| `http_status` | integer | HTTP response code | 200-599 | `200` |
| `section_headers` | list[string] | Extracted from HTML hierarchy | Ordered list; can be empty | `["Module 1", "Chapter 2", "Forward Kinematics"]` |

**Relationships**:
- **1 → Many**: One Page has many Chunks (after chunking)
- **Lifecycle**: Page → extracted → chunked → embedded → inserted

**State transitions**:

```
created → fetched → parsed → chunked → embedded → stored → verified
```

**Validation rules** (FR-002, FR-003):
- URL must be from base domain (prevent crawling entire internet)
- Extracted text must preserve document structure (headers remain visible)
- Boilerplate (nav, footer) must be removed; only `<article>` body content retained
- Special characters and UTF-8 encoding preserved without corruption

**Error handling**:
- HTTP 404/503 → log and skip; continue with other URLs (FR-013)
- Encoding issues → log warning; attempt UTF-8 repair

**Example**:

```json
{
  "url": "https://textbook.vercel.app/module-1/chapter-2",
  "title": "Introduction to Robot Kinematics",
  "fetch_timestamp": "2025-12-25T10:30:00Z",
  "fetch_duration_seconds": 2.45,
  "http_status": 200,
  "section_headers": ["Module 1", "Chapter 2", "Forward Kinematics"],
  "extracted_text": "Chapter 2: Forward Kinematics\nIn robotics, forward kinematics is...",
  "raw_html": "<html>...</html>"
}
```

---

## 2. Chunk

**Definition**: A segment of extracted text from a Page, sized to fit embedding model limits, with metadata for retrieval and attribution.

**Attributes**:

| Attribute | Type | Source | Validation | Example |
|-----------|------|--------|-----------|---------|
| `chunk_id` | string (SHA256 hex) | SHA256(text) | 64 hex chars; deterministic | `a3c7e2b5...` |
| `text` | string | LangChain splitter | 1-10k chars; ≥10 tokens; ≤max_tokens | (512 tokens ≈ 1000-1500 chars) |
| `token_count` | integer | tiktoken encoding | Within ±10% of target (FR-004) | `512` |
| `source_url` | string (URL) | Parent Page.url | Inherited from Page | (same as Page) |
| `page_title` | string | Parent Page.title | Inherited from Page | (same as Page) |
| `section_headers` | list[string] | Parent Page.section_headers | Ordered; hierarchical | (same as Page) |
| `chunk_index` | integer | Chunk position in Page | 0-based; sequential | `5` |
| `created_timestamp` | ISO 8601 datetime | System clock | UTC; RFC 3339 | `2025-12-25T10:30:00Z` |

**Relationships**:
- **Many → 1**: Many Chunks belong to one Page
- **1 → 1**: One Chunk → one Embedding
- **1 → 1**: One Chunk → one Qdrant Point (payload + vector)

**Validation rules** (FR-004, FR-005):
- Token count must be within ±10% of configured size (SC-003)
- Chunk ID (content hash) must be deterministic (same text = same hash)
- Metadata (URL, title, headers) must be complete and non-empty
- Chunks must not be empty (minimum 10 tokens; FR-005 implicitly)

**Deduplication** (FR-008):
- Content hash (`chunk_id = SHA256(text)`) ensures deterministic ID
- Same text from different URLs → same chunk_id → upsert replaces (idempotent)
- No duplicate vectors if content is identical

**Example**:

```json
{
  "chunk_id": "a3c7e2b5f1e8d2c9a0b7c5f3e8a1d4b2",
  "text": "Forward kinematics is the process of computing end-effector position and orientation from joint angles. Given joint angles θ₁, θ₂, ..., θₙ, the forward kinematics function FK computes the transformation matrix T that maps from the base frame to the end-effector frame.",
  "token_count": 512,
  "source_url": "https://textbook.vercel.app/module-1/chapter-2",
  "page_title": "Introduction to Robot Kinematics",
  "section_headers": ["Module 1", "Chapter 2", "Forward Kinematics"],
  "chunk_index": 5,
  "created_timestamp": "2025-12-25T10:30:00Z"
}
```

---

## 3. Embedding

**Definition**: A numeric vector representation of a Chunk generated by Cohere, suitable for semantic similarity search.

**Attributes**:

| Attribute | Type | Source | Validation | Example |
|-----------|------|--------|-----------|---------|
| `chunk_id` | string | Parent Chunk | 64 hex chars; references Chunk | (same as Chunk.chunk_id) |
| `vector` | list[float] | Cohere API | 1024 dimensions; float32/float64 | `[0.123, -0.456, ...]` |
| `model` | string | Configuration | Constant; identifies encoder | `embed-english-v3.0` |
| `created_timestamp` | ISO 8601 datetime | System clock | UTC; RFC 3339 | `2025-12-25T10:35:00Z` |
| `embedding_type` | string | Configuration | "float" or "int8" (quantized) | `float` |

**Relationships**:
- **1 → 1**: One Embedding corresponds to one Chunk
- **1 → 1**: One Embedding stored as one Qdrant Point vector

**Validation rules** (FR-006):
- Vector must have exactly 1024 dimensions (Cohere embed-english-v3.0)
- All values must be finite floats (no NaN or infinity)
- L2 norm (magnitude) should be ~1.0 for cosine similarity (normalized)

**Generation** (FR-006, FR-013):
- Batch API calls (max 96 chunks per request)
- Exponential backoff retry on rate limits (TooManyRequestsError)
- Log failed chunks with chunk_id and error details
- Support up to 3 retry attempts with 2^n second delays

**Cost**:
- ~$3-5 for 50k chunks at Cohere's current pricing
- Deduplication prevents re-embedding identical content (FR-008)

**Example**:

```json
{
  "chunk_id": "a3c7e2b5f1e8d2c9a0b7c5f3e8a1d4b2",
  "vector": [0.123456, -0.234567, 0.345678, ...],  // 1024 dimensions
  "model": "embed-english-v3.0",
  "created_timestamp": "2025-12-25T10:35:00Z",
  "embedding_type": "float"
}
```

---

## 4. Qdrant Point

**Definition**: A point in Qdrant Cloud combining an embedding vector with searchable metadata.

**Structure in Qdrant**:

```
Point {
  id: string (UUID from content hash)
  vector: [1024 floats]
  payload: {
    // Source tracking
    source_url: string
    page_title: string
    section_headers: [string]

    // Chunk identification
    chunk_id: string  (same as hash)
    chunk_index: integer

    // Content
    chunk_text: string

    // Processing metadata
    timestamp: ISO 8601 datetime
    content_hash: string (SHA256)
  }
}
```

**Attributes**:

| Attribute | Type | Qdrant Field | Indexed? | Searchable? | Example |
|-----------|------|--------------|----------|------------|---------|
| `id` | UUID string | (point ID) | Yes (PK) | No | `a3c7e2b5-f1e8-d2c9-a0b7-c5f3e8a1d4b2` |
| `vector` | float[1024] | (vector field) | Yes | Similarity search | (1024 floats) |
| `source_url` | string | payload.source_url | Keyword | Exact match / filter | `https://textbook.vercel.app/module-1/chapter-2` |
| `page_title` | string | payload.page_title | Text | Full-text search | `Introduction to Robot Kinematics` |
| `section_headers` | list[string] | payload.section_headers | Keyword | Filter by header | `["Module 1", "Chapter 2"]` |
| `chunk_text` | string | payload.chunk_text | Text | Full-text search | (chunk content) |
| `chunk_id` | string | payload.chunk_id | Keyword | Exact match | `a3c7e2b5f1e8d2c9a0b7c5f3e8a1d4b2` |
| `chunk_index` | integer | payload.chunk_index | Integer | Range queries, sort | `5` |
| `timestamp` | ISO 8601 | payload.timestamp | Datetime | Range queries | `2025-12-25T10:35:00Z` |
| `content_hash` | string | payload.content_hash | Keyword | Duplicate detection | `a3c7e2b5f1e8d2c9a0b7c5f3e8a1d4b2` |

**Collection Schema**:

```python
from qdrant_client.models import VectorParams, Distance

VectorParams(
    size=1024,  # Cohere embed-english-v3.0 dimensionality
    distance=Distance.COSINE  # Cosine similarity for text embeddings
)
```

**Relationships**:
- **1 ← 1**: One Point holds one Embedding (vector) + one Chunk (metadata)
- **Filter by source_url**: Query all chunks from a specific page
- **Filter by section_headers**: Query all chunks under a specific topic
- **Similarity search**: Find top-K nearest neighbors by vector

**Validation rules** (FR-007, FR-008, FR-010):
- ID must be UUID format (deterministic from content hash)
- Vector must be 1024-dimensional
- Metadata (url, title, headers) must be queryable
- Deduplication: Same content hash → same point ID → upsert replaces

**Example**:

```json
{
  "id": "a3c7e2b5-f1e8-d2c9-a0b7-c5f3e8a1d4b2",
  "vector": [0.123456, -0.234567, ...],  // 1024 dimensions
  "payload": {
    "source_url": "https://textbook.vercel.app/module-1/chapter-2",
    "page_title": "Introduction to Robot Kinematics",
    "section_headers": ["Module 1", "Chapter 2", "Forward Kinematics"],
    "chunk_id": "a3c7e2b5f1e8d2c9a0b7c5f3e8a1d4b2",
    "chunk_index": 5,
    "chunk_text": "Forward kinematics is...",
    "timestamp": "2025-12-25T10:35:00Z",
    "content_hash": "a3c7e2b5f1e8d2c9a0b7c5f3e8a1d4b2"
  }
}
```

---

## 5. Checkpoint

**Definition**: A record of ingestion progress, enabling resumption from failure without re-processing completed work.

**Structure**:

```json
{
  "last_updated": "2025-12-25T10:45:00Z",
  "processed_hashes": [
    "a3c7e2b5f1e8d2c9a0b7c5f3e8a1d4b2",
    "b4d8f3c6g2f9e3d0a1b8c6f4e9b2e5c3",
    ...  // Set of chunk content hashes already processed
  ],
  "stats": {
    "total_chunks_processed": 1250,
    "total_embeddings_created": 1250,
    "total_points_inserted": 1248,
    "failed_chunks": 2,
    "duration_seconds": 305.45
  }
}
```

**Attributes**:

| Attribute | Type | Validation | Purpose |
|-----------|------|-----------|---------|
| `last_updated` | ISO 8601 datetime | UTC; RFC 3339 | Track freshness |
| `processed_hashes` | set[string] | 64 hex chars each | Which chunks already embedded/inserted |
| `stats.total_chunks_processed` | integer | ≥0 | Total chunks encountered |
| `stats.total_embeddings_created` | integer | ≤ total_chunks | Successful embeddings |
| `stats.total_points_inserted` | integer | ≤ total_embeddings | Successful insertions |
| `stats.failed_chunks` | integer | ≥0 | Chunks that failed |
| `stats.duration_seconds` | float | ≥0 | End-to-end runtime |

**Lifecycle**:
1. **Initialize**: Create empty checkpoint on first run
2. **Record progress**: Add chunk hash after successful embedding + insertion
3. **Save periodically**: Save to JSON file after each batch (fault tolerance)
4. **Resume**: On re-run, load checkpoint and skip already-processed chunks
5. **Clear**: For fresh indexing, delete checkpoint file

**File format**: `ingestion_checkpoint.json` (project root or configurable via `CHECKPOINT_FILE`)

**Example**:

```json
{
  "last_updated": "2025-12-25T10:45:00Z",
  "processed_hashes": [
    "a3c7e2b5f1e8d2c9a0b7c5f3e8a1d4b2",
    "b4d8f3c6g2f9e3d0a1b8c6f4e9b2e5c3"
  ],
  "stats": {
    "total_chunks_processed": 1250,
    "total_embeddings_created": 1248,
    "total_points_inserted": 1248,
    "failed_chunks": 2,
    "duration_seconds": 305.45
  }
}
```

**Resume behavior** (FR-011):
- Load checkpoint at start
- Skip any chunk with hash in `processed_hashes`
- Only embed/insert new chunks
- Update `processed_hashes` incrementally
- On completion, write final stats

---

## 6. Ingestion Report

**Definition**: Final summary of the complete ingestion run, produced at end of execution.

**Format**: JSON file or console output (human-readable + machine-parseable)

**Contents** (FR-012, FR-016):

```json
{
  "run_id": "2025-12-25T10:30:00Z",
  "status": "success",
  "summary": {
    "urls_crawled": 25,
    "urls_failed": 1,
    "total_chunks_created": 1250,
    "total_embeddings_generated": 1248,
    "total_points_inserted": 1248,
    "insertion_success_rate": 0.998,
    "deduplication_skipped": 2
  },
  "timings": {
    "total_duration_seconds": 305.45,
    "crawling_seconds": 45.2,
    "chunking_seconds": 12.3,
    "embedding_seconds": 240.1,
    "insertion_seconds": 7.85
  },
  "errors": [
    {
      "stage": "crawling",
      "url": "https://textbook.vercel.app/broken-link",
      "error": "404 Not Found",
      "timestamp": "2025-12-25T10:31:00Z"
    },
    {
      "stage": "embedding",
      "chunk_id": "bad_hash_123",
      "error": "Cohere API error after 3 retries",
      "timestamp": "2025-12-25T10:35:00Z"
    }
  ],
  "verification": {
    "vector_count": 1248,
    "expected_count": 1248,
    "count_match": true,
    "sample_queries": [
      {
        "query_chunk_id": "a3c7e2b5f1e8d2c9a0b7c5f3e8a1d4b2",
        "top_3_results": [
          {"url": "...", "score": 0.95},
          {"url": "...", "score": 0.88},
          {"url": "...", "score": 0.82}
        ]
      }
    ]
  }
}
```

**Attributes**:

| Attribute | Type | Validation | Example |
|-----------|------|-----------|---------|
| `run_id` | ISO 8601 | UTC start timestamp | `2025-12-25T10:30:00Z` |
| `status` | enum | success \| partial \| failed | `success` |
| `urls_crawled` | integer | ≥0 | `25` |
| `urls_failed` | integer | ≥0; ≤ urls_crawled | `1` |
| `total_chunks_created` | integer | ≥0 | `1250` |
| `insertion_success_rate` | float | 0.0-1.0; should be ≥0.99 | `0.998` |
| `errors` | list[object] | Each with stage, error, timestamp | (error log) |

**Use cases** (SC-001, SC-012):
- Human review: See how many pages crawled, any errors, final vector count
- CI/CD validation: Check `insertion_success_rate ≥ 0.99` (SC-005)
- Debugging: Identify which URLs/chunks failed
- Monitoring: Track ingestion duration trends

---

## Entity Relationship Diagram

```
Page (1)
├── url
├── title
├── extracted_text
└── section_headers
    │
    └──→ (1:Many)
        │
        Chunk (Many)
        ├── chunk_id (PK)
        ├── text
        ├── token_count
        ├── source_url (FK → Page.url)
        ├── page_title
        └── section_headers
            │
            └──→ (1:1)
                │
                Embedding (1)
                ├── chunk_id (FK)
                ├── vector [1024]
                └── model
                    │
                    └──→ (1:1)
                        │
                        Qdrant Point (1)
                        ├── id (UUID from chunk_id)
                        ├── vector [1024]
                        └── payload
                            ├── source_url
                            ├── chunk_id
                            └── ...metadata

Checkpoint (1)
├── processed_hashes [chunk_id, ...]
└── stats
    ├── total_chunks_processed
    ├── total_embeddings_created
    └── total_points_inserted
```

---

## Validation & Acceptance Criteria

### Per-Page Validation (FR-001, FR-002, FR-003):
- ✅ All accessible pages crawled
- ✅ Text extracted with structure preserved (headers visible)
- ✅ Boilerplate (nav, footer) removed
- ✅ Special characters preserved (UTF-8)

### Per-Chunk Validation (FR-004, FR-005):
- ✅ Token count within ±10% of 512 tokens
- ✅ Metadata (URL, title, headers) attached
- ✅ No empty chunks (minimum 10 tokens)

### Per-Embedding Validation (FR-006):
- ✅ All chunks embedded (≥99% success rate)
- ✅ Vector size = 1024 dimensions
- ✅ Retry logic handles rate limits

### Per-Insertion Validation (FR-010):
- ✅ Vector count matches chunk count (within 1%)
- ✅ Metadata queryable by URL, headers, timestamp
- ✅ Deduplication prevents duplicate vectors

### End-to-End Validation (SC-001–SC-012):
- ✅ 100% page coverage (pages with errors logged)
- ✅ Document structure preserved (manual review 10+ pages)
- ✅ Chunk sizes within tolerance (±10%)
- ✅ ≥99% embedding success rate
- ✅ ≥99% insertion success rate
- ✅ Vector count = expected count (±1%)
- ✅ Similarity search returns metadata correctly
- ✅ Re-run doesn't duplicate vectors
- ✅ Full pipeline completes without manual intervention
- ✅ Configuration applied correctly (env vars)
- ✅ Processing time <30 minutes for 50-page book
- ✅ Final report includes all required statistics
