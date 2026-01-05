"""Tests for Qdrant storage."""

import pytest
from unittest.mock import patch, MagicMock

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "ingestion"))

from qdrant_storage import QdrantManager


def test_qdrant_initialize_collection():
    """Test Qdrant collection initialization."""
    with patch('qdrant_storage.QdrantClient') as mock_client_class:
        mock_client = MagicMock()
        mock_client_class.return_value = mock_client
        mock_client.collection_exists.return_value = False

        manager = QdrantManager("http://localhost:6333", "test_key", "test_collection")
        manager.initialize_collection()

        mock_client.create_collection.assert_called_once()


def test_qdrant_upsert_embeddings(sample_chunks):
    """Test upserting embeddings to Qdrant."""
    with patch('qdrant_storage.QdrantClient') as mock_client_class:
        mock_client = MagicMock()
        mock_client_class.return_value = mock_client

        manager = QdrantManager("http://localhost:6333", "test_key", "test_collection")
        embeddings = [[0.1] * 1024, [0.2] * 1024]

        count = manager.upsert_embeddings(sample_chunks, embeddings)

        assert count == 2
        mock_client.upsert.assert_called_once()


def test_qdrant_generate_deterministic_id(sample_chunks):
    """Test point IDs are deterministic."""
    with patch('qdrant_storage.QdrantClient'):
        manager = QdrantManager("http://localhost:6333", "test_key", "test_collection")

        chunk = sample_chunks[0]
        id1 = manager.generate_point_id(chunk["text"], chunk["metadata"]["source_url"])
        id2 = manager.generate_point_id(chunk["text"], chunk["metadata"]["source_url"])

        assert id1 == id2  # Same content = same ID (idempotent)


def test_qdrant_get_stats():
    """Test retrieving collection stats."""
    with patch('qdrant_storage.QdrantClient') as mock_client_class:
        mock_client = MagicMock()
        mock_client_class.return_value = mock_client
        mock_client.get_collection.return_value = MagicMock(
            name="test",
            points_count=100,
            indexed_vectors_count=100,
            status="green"
        )

        manager = QdrantManager("http://localhost:6333", "test_key", "test_collection")
        stats = manager.get_collection_stats()

        assert stats["points_count"] == 100
        assert stats["status"] == "green"
