"""Tests for text chunking."""

import pytest
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "ingestion"))

from chunker import TextChunker


def test_chunker_creates_chunks():
    """Test chunker creates chunks with metadata."""
    chunker = TextChunker(chunk_size=100, chunk_overlap=10)

    text = "This is a sample text. " * 30  # Long enough to create multiple chunks
    chunks = chunker.chunk_text(
        text,
        "https://example.com/page",
        "Example Page",
        ["Section 1", "Subsection 1.1"]
    )

    assert len(chunks) > 1
    assert all("text" in c for c in chunks)
    assert all("metadata" in c for c in chunks)
    assert all("hash" in c for c in chunks)


def test_chunker_attaches_metadata():
    """Test chunker attaches correct metadata to chunks."""
    chunker = TextChunker()

    text = "Sample text. " * 50
    chunks = chunker.chunk_text(
        text,
        "https://example.com/page",
        "Test Page",
        ["Header 1", "Header 2"]
    )

    for chunk in chunks:
        assert chunk["metadata"]["source_url"] == "https://example.com/page"
        assert chunk["metadata"]["page_title"] == "Test Page"
        assert chunk["metadata"]["section_headers"] == ["Header 1", "Header 2"]
        assert "chunk_id" in chunk["metadata"]
        assert "chunk_index" in chunk["metadata"]
        assert "timestamp" in chunk["metadata"]


def test_chunker_generates_deterministic_hash():
    """Test chunker generates deterministic hashes."""
    chunker = TextChunker()

    text = "Same text for hashing"
    chunks1 = chunker.chunk_text(text, "url1", "title1", [])
    chunks2 = chunker.chunk_text(text, "url2", "title2", [])

    # Same content should have same hash
    assert chunks1[0]["hash"] == chunks2[0]["hash"]


def test_chunker_validates_sizes():
    """Test chunker validates chunk sizes within tolerance."""
    chunker = TextChunker(chunk_size=100, chunk_overlap=10)

    text = "Word. " * 500  # Enough for multiple chunks
    chunks = chunker.chunk_text(
        text,
        "https://example.com/page",
        "Test",
        []
    )

    validation = chunker.validate_chunk_sizes(chunks)
    assert validation["total_chunks"] == len(chunks)
    assert validation["avg_tokens"] > 0


def test_chunker_preserves_content():
    """Test chunker preserves text content."""
    chunker = TextChunker()

    text = "Important information should be preserved. " * 50
    chunks = chunker.chunk_text(
        text,
        "https://example.com",
        "Test",
        []
    )

    combined = " ".join(c["text"] for c in chunks)
    assert "Important information" in combined
