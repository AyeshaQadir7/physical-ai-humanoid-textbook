"""Tests for Cohere embedder."""

import pytest
from unittest.mock import patch, MagicMock

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "ingestion"))

from embedder import CohereEmbedder


def test_embedder_batches_requests():
    """Test embedder batches requests correctly."""
    with patch('embedder.cohere.ClientV2') as mock_client_class:
        mock_client = MagicMock()
        mock_client_class.return_value = mock_client
        mock_client.embed.return_value.embeddings.float = [[0.1] * 1024 for _ in range(96)]

        embedder = CohereEmbedder("test_key", batch_size=96)
        chunks = ["text"] * 200

        embeddings = embedder.embed_chunks(chunks)

        assert len(embeddings) > 0
        assert mock_client.embed.call_count >= 2  # At least 2 batches


def test_embedder_handles_rate_limit():
    """Test embedder handles rate limits with retry."""
    import cohere

    with patch('embedder.cohere.ClientV2') as mock_client_class:
        mock_client = MagicMock()
        mock_client_class.return_value = mock_client

        # First call raises rate limit, second succeeds
        mock_client.embed.side_effect = [
            cohere.errors.TooManyRequestsError("Rate limit"),
            MagicMock(embeddings=MagicMock(float=[[0.1] * 1024]))
        ]

        embedder = CohereEmbedder("test_key", max_retries=3)
        embeddings = embedder.embed_chunks(["test text"])

        assert embedder.get_stats()["successful"] > 0


def test_embedder_tracks_stats():
    """Test embedder tracks statistics."""
    with patch('embedder.cohere.ClientV2') as mock_client_class:
        mock_client = MagicMock()
        mock_client_class.return_value = mock_client
        mock_client.embed.return_value.embeddings.float = [[0.1] * 1024]

        embedder = CohereEmbedder("test_key")
        chunks = ["text1", "text2"]
        embedder.embed_chunks(chunks)

        stats = embedder.get_stats()
        assert stats["total_chunks"] == 2
        assert stats["successful"] > 0
