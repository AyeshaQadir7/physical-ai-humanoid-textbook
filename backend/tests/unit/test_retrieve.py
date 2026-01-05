"""Unit tests for retrieval pipeline with mocked APIs."""

import pytest
from unittest.mock import Mock, MagicMock, patch
import json

from retrieve import QueryEmbedder, RetrieverClient, ValidationRunner


class TestQueryEmbedder:
    """Test QueryEmbedder class with mocked Cohere API."""

    @pytest.fixture
    def embedder(self):
        """Create embedder with mock API key."""
        return QueryEmbedder(api_key="test-key-12345", model="embed-english-v3.0")

    @patch("retrieve.cohere.ClientV2")
    def test_embed_query_success(self, mock_cohere, embedder):
        """Test successful query embedding."""
        # Setup mock
        mock_client = MagicMock()
        mock_response = MagicMock()
        mock_response.embeddings.float = [[0.1, 0.2, 0.3] + [0.0] * 1021]  # 1024 dimensions
        mock_client.embed.return_value = mock_response

        embedder.client = mock_client

        # Execute
        result = embedder.embed_query("What is ROS2?")

        # Assert
        assert len(result) == 1024
        assert result[0] == 0.1
        assert result[1] == 0.2
        mock_client.embed.assert_called_once()

    @patch("retrieve.cohere.ClientV2")
    def test_embed_query_empty_string(self, mock_cohere):
        """Test that empty query raises ValueError."""
        embedder = QueryEmbedder(api_key="test-key", model="embed-english-v3.0")

        with pytest.raises(ValueError, match="Query cannot be empty"):
            embedder.embed_query("")

    @patch("retrieve.cohere.ClientV2")
    def test_embed_query_whitespace_only(self, mock_cohere):
        """Test that whitespace-only query raises ValueError."""
        embedder = QueryEmbedder(api_key="test-key", model="embed-english-v3.0")

        with pytest.raises(ValueError, match="Query cannot be empty"):
            embedder.embed_query("   ")

    @patch("retrieve.cohere.ClientV2")
    def test_embed_query_api_failure(self, mock_cohere, embedder):
        """Test retry logic on API failure."""
        # Setup mock to fail first 2 times, succeed on 3rd
        mock_client = MagicMock()
        mock_response = MagicMock()
        mock_response.embeddings.float = [[0.1] * 1024]

        mock_client.embed.side_effect = [
            Exception("API timeout"),
            Exception("Rate limited"),
            mock_response
        ]

        embedder.client = mock_client
        embedder.max_retries = 3

        # Execute
        result = embedder.embed_query("Test query")

        # Assert - should succeed on 3rd attempt
        assert len(result) == 1024
        assert mock_client.embed.call_count == 3


class TestRetrieverClient:
    """Test RetrieverClient class with mocked Qdrant."""

    @pytest.fixture
    def retriever(self):
        """Create retriever with mock clients."""
        with patch("retrieve.QdrantClient"):
            with patch("retrieve.QueryEmbedder"):
                return RetrieverClient(
                    qdrant_url="https://test.qdrant.io",
                    qdrant_api_key="test-key",
                    cohere_api_key="test-key",
                    collection_name="test_collection",
                    model="embed-english-v3.0"
                )

    @patch("retrieve.QueryEmbedder.embed_query")
    def test_search_success(self, mock_embed, retriever):
        """Test successful search with valid results."""
        # Setup mocks
        mock_embed.return_value = [0.1] * 1024

        mock_point1 = MagicMock()
        mock_point1.id = 1
        mock_point1.score = 0.85
        mock_point1.payload = {
            "chunk_text": "ROS2 is a middleware...",
            "source_url": "https://textbook.io/ros2",
            "page_title": "ROS2 Overview",
            "section_headers": ["Module 1", "Getting Started"],
            "chunk_index": 0
        }

        mock_point2 = MagicMock()
        mock_point2.id = 2
        mock_point2.score = 0.72
        mock_point2.payload = {
            "chunk_text": "ROS2 middleware architecture...",
            "source_url": "https://textbook.io/ros2-arch",
            "page_title": "ROS2 Architecture",
            "section_headers": ["Module 1", "Architecture"],
            "chunk_index": 1
        }

        # Mock the search method on the instance
        retriever.qdrant_client.search = MagicMock(return_value=[mock_point1, mock_point2])

        # Execute
        result = retriever.search("What is ROS2?", top_k=5)

        # Assert
        assert result["status"] == "success"
        assert result["total_results"] == 2
        assert len(result["results"]) == 2
        assert result["results"][0]["similarity_score"] == 0.85
        assert result["results"][1]["similarity_score"] == 0.72
        assert result["execution_metrics"]["embedding_model"] == "embed-english-v3.0"

    @patch("retrieve.QueryEmbedder.embed_query")
    def test_search_invalid_top_k(self, mock_embed, retriever):
        """Test that invalid top_k raises ValueError."""
        with pytest.raises(ValueError, match="top_k must be between 1 and 100"):
            retriever.search("test", top_k=101)

        with pytest.raises(ValueError, match="top_k must be between 1 and 100"):
            retriever.search("test", top_k=0)

    @patch("retrieve.QueryEmbedder.embed_query")
    def test_search_invalid_threshold(self, mock_embed, retriever):
        """Test that invalid similarity_threshold raises ValueError."""
        with pytest.raises(ValueError, match="similarity_threshold must be between"):
            retriever.search("test", similarity_threshold=1.5)

        with pytest.raises(ValueError, match="similarity_threshold must be between"):
            retriever.search("test", similarity_threshold=-0.1)

    @patch("retrieve.QueryEmbedder.embed_query")
    def test_search_filters_by_threshold(self, mock_embed, retriever):
        """Test that results below threshold are filtered."""
        mock_embed.return_value = [0.1] * 1024

        # Create mock points with different scores
        mock_point1 = MagicMock()
        mock_point1.id = 1
        mock_point1.score = 0.8
        mock_point1.payload = {
            "chunk_text": "High relevance...",
            "source_url": "https://test.io/1",
            "page_title": "Page 1",
            "section_headers": ["A"],
            "chunk_index": 0
        }

        mock_point2 = MagicMock()
        mock_point2.id = 2
        mock_point2.score = 0.4  # Below 0.5 threshold
        mock_point2.payload = {
            "chunk_text": "Low relevance...",
            "source_url": "https://test.io/2",
            "page_title": "Page 2",
            "section_headers": ["B"],
            "chunk_index": 1
        }

        # Mock the search method on the instance
        retriever.qdrant_client.search = MagicMock(return_value=[mock_point1, mock_point2])

        # Execute with threshold
        result = retriever.search("query", top_k=5, similarity_threshold=0.5)

        # Assert - only first point should be included
        assert result["total_results"] == 1
        assert result["results"][0]["similarity_score"] == 0.8

    def test_validate_metadata_valid(self, retriever):
        """Test metadata validation with valid results."""
        results = [
            {
                "chunk_text": "Test content 1",
                "metadata": {
                    "source_url": "https://test.io",
                    "page_title": "Title",
                    "section_headers": ["A", "B"],
                    "chunk_index": 0
                }
            },
            {
                "chunk_text": "Test content 2",
                "metadata": {
                    "source_url": "https://test2.io",
                    "page_title": "Title 2",
                    "section_headers": ["C"],
                    "chunk_index": 1
                }
            }
        ]

        report = retriever.validate_metadata(results)

        assert report["total_results"] == 2
        assert report["valid_results"] == 2
        assert report["invalid_results"] == 0
        assert len(report["issues"]) == 0

    def test_validate_metadata_missing_fields(self, retriever):
        """Test metadata validation with missing fields."""
        results = [
            {
                "metadata": {
                    "source_url": "",  # Empty
                    "page_title": "Title",
                    "section_headers": ["A"],
                    "chunk_index": 0
                }
            }
        ]

        report = retriever.validate_metadata(results)

        assert report["invalid_results"] == 1
        assert len(report["issues"]) > 0
        assert "source_url" in report["issues"][0]


class TestValidationRunner:
    """Test ValidationRunner class."""

    @patch("retrieve.RetrieverClient")
    @patch("retrieve.Config")
    def test_initialization(self, mock_config_class, mock_client_class):
        """Test ValidationRunner initialization."""
        mock_config = MagicMock()
        mock_config_class.return_value = mock_config

        mock_client = MagicMock()
        mock_client_class.return_value = mock_client

        runner = ValidationRunner(config=mock_config)

        assert runner.config == mock_config
        mock_config.validate.assert_called_once()

    @patch("retrieve.RetrieverClient")
    @patch("retrieve.Config")
    def test_validate_queries_default(self, mock_config_class, mock_client_class):
        """Test validate_queries with default queries."""
        mock_config = MagicMock()
        mock_config_class.return_value = mock_config

        mock_client = MagicMock()
        mock_client.search.return_value = {
            "status": "success",
            "results": [],
            "total_results": 0
        }
        mock_client_class.return_value = mock_client

        runner = ValidationRunner(config=mock_config)
        responses = runner.validate_queries()

        # Should run 4 default queries
        assert len(responses) == 4
        assert mock_client.search.call_count == 4

    @patch("retrieve.RetrieverClient")
    @patch("retrieve.Config")
    def test_validate_queries_custom(self, mock_config_class, mock_client_class):
        """Test validate_queries with custom queries."""
        mock_config = MagicMock()
        mock_config_class.return_value = mock_config

        mock_client = MagicMock()
        mock_client.search.return_value = {
            "status": "success",
            "results": [],
            "total_results": 0
        }
        mock_client_class.return_value = mock_client

        runner = ValidationRunner(config=mock_config)
        custom_queries = ["Query 1", "Query 2"]
        responses = runner.validate_queries(queries=custom_queries)

        # Should run only custom queries
        assert len(responses) == 2
        assert mock_client.search.call_count == 2
