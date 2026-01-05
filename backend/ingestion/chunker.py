"""Text chunking with token-based splitting."""

import hashlib
import logging
from datetime import datetime
from typing import List, Dict

import tiktoken
from langchain_text_splitters import RecursiveCharacterTextSplitter


logger = logging.getLogger(__name__)


class TextChunker:
    """Token-based text chunker with metadata attachment."""

    def __init__(self, chunk_size: int = 512, chunk_overlap: int = 50):
        """Initialize chunker with token-based splitting.

        Args:
            chunk_size: Target tokens per chunk
            chunk_overlap: Token overlap between chunks
        """
        self.chunk_size = chunk_size
        self.chunk_overlap = chunk_overlap

        # Use tiktoken-based splitter for accurate token counting
        self.splitter = RecursiveCharacterTextSplitter.from_tiktoken_encoder(
            encoding_name="cl100k_base",
            chunk_size=chunk_size,
            chunk_overlap=chunk_overlap,
            separators=["\n\n", "\n", ". ", " ", ""]
        )

        self.encoder = tiktoken.get_encoding("cl100k_base")

    def chunk_text(
        self,
        text: str,
        url: str,
        page_title: str,
        section_headers: List[str]
    ) -> List[Dict]:
        """Split text into chunks with metadata.

        Args:
            text: Text to chunk
            url: Source URL
            page_title: Page title
            section_headers: List of section headers

        Returns:
            List of dicts with 'text', 'metadata', and 'hash'
        """
        chunk_texts = self.splitter.split_text(text)

        chunks_with_metadata = []
        for idx, chunk_text in enumerate(chunk_texts):
            # Generate content hash for deduplication
            chunk_hash = hashlib.sha256(chunk_text.encode('utf-8')).hexdigest()

            # Count actual tokens
            token_count = len(self.encoder.encode(chunk_text))

            chunks_with_metadata.append({
                "text": chunk_text,
                "metadata": {
                    "source_url": url,
                    "page_title": page_title,
                    "section_headers": section_headers,
                    "chunk_id": chunk_hash,
                    "chunk_index": idx,
                    "timestamp": datetime.utcnow().isoformat() + "Z",
                    "chunk_size_tokens": token_count
                },
                "hash": chunk_hash
            })

        logger.info(f"Chunked {url}: {len(chunks_with_metadata)} chunks")
        return chunks_with_metadata

    def validate_chunk_sizes(self, chunks: List[Dict]) -> Dict:
        """Validate chunk sizes are within ±10% of target.

        Args:
            chunks: List of chunk dicts

        Returns:
            Validation stats
        """
        token_counts = [c["metadata"]["chunk_size_tokens"] for c in chunks]
        if not token_counts:
            return {"total_chunks": 0, "within_tolerance": True}

        avg_size = sum(token_counts) / len(token_counts)
        min_size = min(token_counts)
        max_size = max(token_counts)

        target_min = self.chunk_size * 0.9
        target_max = self.chunk_size * 1.1

        within_tolerance = all(target_min <= size <= target_max for size in token_counts)

        return {
            "total_chunks": len(chunks),
            "avg_tokens": round(avg_size, 1),
            "min_tokens": min_size,
            "max_tokens": max_size,
            "within_tolerance": within_tolerance,
            "target_size": self.chunk_size
        }
