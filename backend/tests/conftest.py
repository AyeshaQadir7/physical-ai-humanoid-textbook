"""Pytest fixtures for testing."""

import pytest
from unittest.mock import MagicMock

import sys
from pathlib import Path

# Add backend to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent))


@pytest.fixture
def mock_cohere_client():
    """Mock Cohere client."""
    mock = MagicMock()
    mock.embed.return_value.embeddings.float = [
        [0.1] * 1024 for _ in range(10)  # 10 sample embeddings
    ]
    return mock


@pytest.fixture
def mock_qdrant_client():
    """Mock Qdrant client."""
    mock = MagicMock()
    mock.collection_exists.return_value = True
    mock.get_collection.return_value = MagicMock(
        name="test_collection",
        points_count=100,
        indexed_vectors_count=100,
        status="green"
    )
    return mock


@pytest.fixture
def sample_chunks():
    """Sample chunks for testing."""
    return [
        {
            "text": "This is a sample chunk of text for testing purposes.",
            "metadata": {
                "source_url": "https://example.com/page1",
                "page_title": "Example Page",
                "section_headers": ["Section 1", "Subsection 1.1"],
                "chunk_id": "abc123",
                "chunk_index": 0,
                "timestamp": "2025-12-25T00:00:00Z",
                "chunk_size_tokens": 512
            },
            "hash": "abc123"
        },
        {
            "text": "Another sample chunk with different content.",
            "metadata": {
                "source_url": "https://example.com/page1",
                "page_title": "Example Page",
                "section_headers": ["Section 2"],
                "chunk_id": "def456",
                "chunk_index": 1,
                "timestamp": "2025-12-25T00:00:00Z",
                "chunk_size_tokens": 512
            },
            "hash": "def456"
        }
    ]


@pytest.fixture
def sample_html():
    """Sample HTML for crawler testing."""
    return """
    <html>
        <head><title>Test Page</title></head>
        <body>
            <nav>Navigation</nav>
            <article>
                <h1>Main Content</h1>
                <p>This is the main content of the page.</p>
                <section>
                    <h2>Section 1</h2>
                    <p>Content for section 1.</p>
                </section>
            </article>
            <footer>Footer</footer>
        </body>
    </html>
    """
