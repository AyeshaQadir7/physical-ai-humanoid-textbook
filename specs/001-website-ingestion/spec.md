# Feature Specification: Website Ingestion and Vector Storage

**Feature Branch**: `001-website-ingestion`
**Created**: 2025-12-25
**Status**: Draft
**Input**: Ingest published book content from live website URLs, generate embeddings using Cohere models, and store them in Qdrant Cloud for later retrieval.

## User Scenarios & Testing _(mandatory)_

### User Story 1 - Crawl and Index Book Content (Priority: P1)

A developer needs to extract clean, structured text from all pages of a published textbook website (hosted on Vercel) and prepare it for embedding generation. This is the foundational task that enables all downstream RAG operations.

**Why this priority**: This is the critical first step. Without successfully extracting and chunking content, no embeddings can be generated or stored. The entire feature depends on reliable text extraction and preparation.

**Independent Test**: Can be fully tested by running the ingestion script against a live Vercel URL, extracting text from all pages, verifying chunk counts, and confirming that chunks contain valid content with proper metadata (URL, title, headers).

**Acceptance Scenarios**:

1. **Given** a published textbook URL on Vercel, **When** the ingestion script runs, **Then** all accessible pages are crawled and text is extracted with headers and structure preserved
2. **Given** multi-page book content, **When** text is chunked, **Then** each chunk includes source URL, page title, and section headers as metadata
3. **Given** a page with navigation elements and footer text, **When** text is extracted, **Then** boilerplate content is filtered and only main content is retained
4. **Given** a re-run of the ingestion script, **When** duplicate chunks are detected, **Then** they are deduplicated before embedding
5. **Given** a URL that returns HTTP errors (404, 503), **When** the crawler encounters it, **Then** the page is logged as skipped and crawling continues for other pages
6. **Given** pages with encoding issues or special characters, **When** text is extracted, **Then** content is properly decoded and special characters are preserved without data loss

---

### User Story 2 - Generate and Store Embeddings (Priority: P1)

A developer needs to take the extracted, chunked content and generate embeddings using Cohere's embedding API, then store both the vectors and metadata in Qdrant Cloud for fast retrieval.

**Why this priority**: Once content is prepared, generating embeddings and storing them in the vector database is the core value delivery. Without this, the RAG system has no searchable knowledge base.

**Independent Test**: Can be fully tested by taking pre-chunked content, generating embeddings via Cohere API, inserting into Qdrant, verifying vector count matches chunk count, and confirming metadata is queryable.

**Acceptance Scenarios**:

1. **Given** a set of text chunks with metadata, **When** embeddings are generated via Cohere, **Then** each chunk receives a unique vector representation
2. **Given** generated embeddings, **When** they are inserted into Qdrant Cloud, **Then** the collection records match the vector count and metadata is searchable
3. **Given** an existing Qdrant collection, **When** new chunks are processed, **Then** only new chunks are embedded and inserted (no duplicate vectors)
4. **Given** an embedding request, **When** Cohere API rate limits are hit, **Then** the process retries with exponential backoff
5. **Given** a network interruption during embedding insertion, **When** the process resumes, **Then** previously inserted vectors are not re-embedded and the insertion continues from the last successful checkpoint
6. **Given** Cohere API returns an error for a specific chunk, **When** the error is logged, **Then** the pipeline captures the error details (chunk ID, error message) and continues processing remaining chunks

---

### User Story 3 - Verify Indexing Success (Priority: P1)

A developer needs to verify that the entire pipeline completed successfully by checking vector counts, sampling stored embeddings, and validating metadata integrity.

**Why this priority**: Verification ensures data integrity and gives confidence that the knowledge base is ready for retrieval. This is essential before downstream RAG systems attempt to query.

**Independent Test**: Can be fully tested by running a verification query against the populated Qdrant collection, checking vector counts match expected chunk counts, and confirming sample similarity queries return results with correct metadata.

**Acceptance Scenarios**:

1. **Given** a populated Qdrant collection, **When** vector count is checked, **Then** it matches the total number of chunks ingested
2. **Given** a sample query vector, **When** similarity search is performed, **Then** results are returned with correct metadata (URL, title, headers)
3. **Given** a completed ingestion run, **When** collection stats are logged, **Then** vector count, metadata fields, and last update timestamp are reported
4. **Given** a collection with missing vectors (e.g., insertion failures), **When** verification runs, **Then** discrepancies are reported with details on missing chunks and affected URLs
5. **Given** metadata in Qdrant, **When** sample records are queried, **Then** all metadata fields are queryable and filterable by URL and section headers

---

### User Story 4 - Configure and Re-index (Priority: P2)

A developer needs to customize chunking parameters (size, overlap) and re-run the pipeline without duplicating existing vectors, to support iterative refinement of the knowledge base.

**Why this priority**: After initial ingestion, users may need to re-index with different chunking strategies or add new book URLs. This feature allows non-destructive updates and configuration flexibility.

**Independent Test**: Can be fully tested by ingesting content with one chunking strategy, then re-running with different parameters and verifying only new chunks are added without removing old ones.

**Acceptance Scenarios**:

1. **Given** a chunking configuration (size, overlap), **When** the ingestion script runs, **Then** chunks are created according to the specified parameters
2. **Given** a re-run with new URLs, **When** existing URLs are skipped, **Then** only new content is processed and embedded
3. **Given** a collection with existing vectors, **When** new chunks are added, **Then** the collection size increases and no existing vectors are duplicated
4. **Given** updated content on an existing page URL, **When** the page is re-crawled, **Then** old chunks are identified and can be removed or updated based on configuration
5. **Given** environment variables that change chunking parameters, **When** the script runs with new config, **Then** only the newly-chunked content is embedded and stored (old chunks remain unchanged)

---

### Edge Cases

- **Unreachable URLs**: What happens when a website URL becomes unreachable during crawling? → System logs the error, records affected URLs, and continues with remaining URLs. Final report lists all failed URLs.
- **Mixed languages/special characters**: How does the system handle pages with mixed languages or special characters? → Text is extracted with UTF-8 encoding preserved; non-ASCII characters are retained without corruption.
- **API errors mid-pipeline**: What occurs if Cohere embedding API returns errors mid-pipeline? → Errors are logged with chunk IDs and error messages; pipeline retries failed chunks up to max retry limit (e.g., 3 attempts); final report lists permanently failed chunks.
- **Extremely large or small content**: How should the system handle extremely large pages or extremely small chunks? → System enforces minimum chunk size (e.g., 10 tokens) and maximum page size (e.g., 100MB); pages exceeding limits are logged with recommendations.
- **Duplicate chunks across URLs**: What happens if duplicate chunks are detected across multiple URLs? → Duplicates are identified by content hash before embedding; system optionally deduplicates or flags them based on configuration.
- **Interrupted mid-run**: How is idempotency maintained if the script is interrupted mid-run? → Checkpoint system records progress (processed URLs, embedded chunks); resume capability allows re-running from last checkpoint without re-processing completed work.
- **Network timeouts**: How are persistent network issues handled? → Configurable timeouts with exponential backoff (e.g., start at 1s, max 60s); persistent timeouts result in the URL being logged and skipped with retry count tracked.
- **Qdrant collection schema mismatch**: What if schema differs from expected? → System validates schema on startup; if mismatch detected, ingestion halts with clear error message and recovery instructions.

## Requirements _(mandatory)_

### Functional Requirements

- **FR-001**: System MUST crawl all accessible pages from a provided Vercel URL and extract clean text content
- **FR-002**: System MUST preserve document structure including page titles, section headers, and logical content hierarchy
- **FR-003**: System MUST filter out navigation elements, footers, and other boilerplate content from extracted text
- **FR-004**: System MUST split extracted text into chunks using configurable size and overlap parameters
- **FR-005**: System MUST attach metadata to each chunk including source URL, page title, section headers, and chunk ID
- **FR-006**: System MUST generate embeddings for all text chunks using Cohere's embedding API with support for batching
- **FR-007**: System MUST establish and manage a Qdrant Cloud collection with appropriate schema for vector storage
- **FR-008**: System MUST insert embeddings and metadata into Qdrant with deduplication to prevent duplicate vectors using content-based hashing
- **FR-009**: System MUST support configuration via environment variables (API keys, URLs, chunking parameters, batch size, retry count)
- **FR-010**: System MUST verify successful insertion by confirming vector count matches chunk count and validating metadata queryability
- **FR-011**: System MUST be re-runnable without duplicating existing vectors; support resume from checkpoints on failure
- **FR-012**: System MUST provide clear, structured logging of ingestion progress, errors, and final statistics (format: JSON or CSV)
- **FR-013**: System MUST handle network errors, rate limits, and API failures gracefully with exponential backoff retry logic (configurable max retries)
- **FR-014**: System MUST validate collection schema on startup and fail with clear error message if mismatch detected
- **FR-015**: System MUST support incremental indexing: crawling new/updated URLs while preserving existing vectors for unchanged content
- **FR-016**: System MUST track and report processing statistics: URLs crawled, chunks created, embeddings generated, insertion success rate, and processing duration

### Key Entities

- **Chunk**: A segment of extracted text (e.g., 512 tokens) with associated metadata (source URL, page title, section headers, chunk ID). Represents the atomic unit for embedding and storage. Uniquely identified by content hash to support deduplication.
- **Embedding**: A vector representation of a chunk generated by Cohere (typically 768 or 1024 dimensions depending on model). Enables similarity-based retrieval in Qdrant.
- **Collection**: A Qdrant Cloud collection containing vectors and metadata for all chunks. Serves as the persistent knowledge base for RAG queries. Schema includes vector field, chunk text, metadata fields (URL, title, headers, chunk_id), and timestamp.
- **Document Metadata**: URL, page title, section headers, and chunk ID associated with each chunk. Enables metadata-based filtering, attribution, and traceability of retrieved results. Metadata must be queryable and filterable.
- **Checkpoint**: Record of processing progress (URLs completed, chunks embedded, timestamp). Enables resume capability on failure without re-processing completed work.

## Success Criteria _(mandatory)_

### Measurable Outcomes

- **SC-001**: All pages from a target book URL are successfully crawled and text is extracted (100% page coverage; pages with retrieval errors are logged and reported)
- **SC-002**: Extracted text maintains document structure with headers and sections clearly preserved (verified by manual sample review of at least 10 pages)
- **SC-003**: Chunking produces consistent, appropriately-sized segments (e.g., 512 tokens per chunk with 50-token overlap) as configured; actual chunk sizes within ±10% of target
- **SC-004**: Embeddings are generated for all chunks with no failures (≥99% embedding success rate; failed chunks logged with retry details)
- **SC-005**: All embeddings are successfully inserted into Qdrant Cloud (insertion success rate ≥99%; failed insertions retried)
- **SC-006**: Vector count in Qdrant matches total number of chunks ingested within 1% tolerance (data integrity verified post-ingestion)
- **SC-007**: Sample similarity queries (e.g., 5 random chunks) return results with correct metadata and expected relevance ranking (top-3 results match query intent)
- **SC-008**: Re-running the pipeline on identical input does not duplicate vectors (vector count remains stable; duplicate detection works correctly)
- **SC-009**: Complete ingestion pipeline (crawl → chunk → embed → store → verify) completes without manual intervention for typical book (10-100 pages)
- **SC-010**: Configuration changes (chunking parameters, URLs) are applied correctly without code modification; environment variables properly override defaults
- **SC-011**: Processing time for typical book (50 pages, 50k chunks) is <30 minutes end-to-end (including network delays)
- **SC-012**: Final ingestion report includes: URLs processed, chunk count, embedding count, insertion success rate, processing duration, and detailed error log

## Constraints & Assumptions

### Constraints

- Must use Cohere as the embeddings provider (latest stable model available at time of implementation)
- Must use Qdrant Cloud for vector storage (not self-hosted Qdrant)
- Implementation language is Python (3.9+)
- No frontend, chatbot, or retrieval/search logic in this feature
- Metadata schema must support page-level (URL, title) and chunk-level (section headers, chunk ID) information
- Must support re-indexing and incremental updates without duplicating or losing existing vectors
- No authentication required for target book URLs (public Vercel hosting)
- Deduplication strategy must be deterministic and based on content hash

### Assumptions

- Book content is publicly accessible via Vercel URLs (no authentication required)
- Vercel-hosted websites have standard HTML structure with identifiable content areas (body, article, main, or similar semantic tags)
- HTML content uses UTF-8 encoding; non-standard encodings will be logged as warnings
- Cohere API quotas are sufficient for embedding all extracted chunks in a single run
- Qdrant Cloud account is pre-configured with appropriate storage limits (estimated as 1GB per 10k chunks at default dimensions)
- Network connectivity to both Cohere and Qdrant Cloud is available with reasonable latency (<5s round-trip)
- Text extraction using HTML parsing (e.g., BeautifulSoup) is sufficient; no OCR required for embedded PDFs or images
- Chunking strategy is token-based using standard tokenizers (e.g., SpaCy or similar); token counts are approximate
- Metadata deduplication can be reliably achieved by checking content hash (SHA-256) before insertion
- Chunk size configuration will be provided as token count targets (not character counts)
- Processing single book URLs sequentially is acceptable; no requirement for parallel URL processing

## Dependencies & Out of Scope

### External Dependencies

- **Cohere API**: Provides embedding generation (assumed account with valid API key exists)
- **Qdrant Cloud**: Provides vector storage (assumed account with collection ready)
- **Vercel Hosting**: Target book content is hosted and publicly accessible

### Out of Scope

- Retrieval or similarity search logic (reserved for future RAG feature)
- OpenAI Agents or FastAPI integration
- Frontend or UI components
- User query handling or chatbot functionality
- Authentication for protected content
- Multi-language translation
- PDF extraction or OCR

## Deliverables

- **URL ingestion and text extraction module**: Crawls Vercel URLs, extracts clean text, preserves structure
- **Chunking and embedding generation pipeline**: Splits text, generates vectors via Cohere
- **Qdrant collection setup and insertion logic**: Initializes schema, inserts embeddings with metadata
- **Configuration module**: Supports environment variables for API keys, URLs, chunking parameters
- **Verification and logging**: Reports vector count, logs progress, validates data integrity
- **Minimal README**: Documents setup, configuration, and execution instructions
