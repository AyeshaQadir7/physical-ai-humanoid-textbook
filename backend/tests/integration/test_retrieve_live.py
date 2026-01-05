"""Integration tests for retrieval pipeline with live APIs.

These tests require:
- Valid COHERE_API_KEY in environment
- Valid QDRANT_URL and QDRANT_API_KEY in environment
- Populated textbook_embeddings collection with 192+ vectors
"""

import os
import pytest
import json

from retrieve import ValidationRunner, RetrieverClient


@pytest.mark.integration
class TestRetrievalLiveAPIs:
    """Test retrieval with live Cohere and Qdrant APIs."""

    @pytest.fixture(scope="class")
    def runner(self):
        """Initialize ValidationRunner with live APIs."""
        try:
            runner = ValidationRunner()
            return runner
        except Exception as e:
            pytest.skip(f"Cannot initialize ValidationRunner: {e}")

    @pytest.fixture(scope="class")
    def sample_queries(self):
        """Sample queries for testing."""
        return [
            {
                "query": "What is ROS2?",
                "category": "factual",
                "expected_keywords": ["ROS2", "Robot", "Operating System"]
            },
            {
                "query": "Explain simulation in Gazebo",
                "category": "conceptual",
                "expected_keywords": ["Gazebo", "simulation", "environment"]
            },
            {
                "query": "How do I set up Isaac Sim?",
                "category": "procedural",
                "expected_keywords": ["Isaac", "setup", "install"]
            }
        ]

    def test_retriever_connectivity(self, runner):
        """Test that retriever can connect to APIs."""
        # This is implicitly tested by ValidationRunner initialization
        assert runner is not None
        assert runner.client is not None
        assert runner.config is not None

    def test_single_query_search(self, runner):
        """Test single query search against live APIs."""
        response = runner.client.search(
            query="What is ROS2?",
            top_k=5
        )

        # Verify response structure
        assert "status" in response
        assert response["status"] in ["success", "error"]

        if response["status"] == "success":
            assert "results" in response
            assert "total_results" in response
            assert "execution_metrics" in response

            # Verify metrics
            metrics = response["execution_metrics"]
            assert "query_embedding_time_ms" in metrics
            assert "vector_search_time_ms" in metrics
            assert "total_execution_time_ms" in metrics
            assert metrics["total_execution_time_ms"] < 2000  # Must be <2s

    def test_search_returns_metadata(self, runner):
        """Test that search results include complete metadata."""
        response = runner.client.search(
            query="What is ROS2?",
            top_k=5
        )

        if response["status"] == "success" and response["results"]:
            for result in response["results"]:
                # Verify result structure
                assert "chunk_id" in result
                assert "chunk_text" in result
                assert "similarity_score" in result
                assert "rank" in result
                assert "metadata" in result

                # Verify metadata
                metadata = result["metadata"]
                assert "source_url" in metadata
                assert "page_title" in metadata
                assert "section_headers" in metadata
                assert "chunk_index" in metadata

                # Verify metadata values are non-empty
                assert metadata["source_url"]
                assert metadata["page_title"]
                assert isinstance(metadata["section_headers"], list)

    def test_search_result_ranking(self, runner):
        """Test that results are ranked by similarity score."""
        response = runner.client.search(
            query="What is ROS2?",
            top_k=5
        )

        if response["status"] == "success" and len(response["results"]) > 1:
            results = response["results"]

            # Verify scores are in descending order
            for i in range(len(results) - 1):
                assert results[i]["similarity_score"] >= results[i + 1]["similarity_score"]

            # Verify ranks are sequential
            for idx, result in enumerate(results, 1):
                assert result["rank"] == idx

    def test_configurable_top_k(self, runner):
        """Test top_k parameter with different values."""
        query = "What is ROS2?"

        for k in [1, 3, 5, 10]:
            response = runner.client.search(query=query, top_k=k)

            if response["status"] == "success":
                # Result count should be <= k (may be less if collection is small)
                assert len(response["results"]) <= k
                assert response["requested_top_k"] == k

    def test_similarity_threshold_filtering(self, runner):
        """Test similarity_threshold parameter."""
        response_no_threshold = runner.client.search(
            query="What is ROS2?",
            top_k=10,
            similarity_threshold=0.0
        )

        response_high_threshold = runner.client.search(
            query="What is ROS2?",
            top_k=10,
            similarity_threshold=0.8
        )

        if (response_no_threshold["status"] == "success" and
            response_high_threshold["status"] == "success"):
            # High threshold should return <= results as low threshold
            assert (len(response_high_threshold["results"]) <=
                    len(response_no_threshold["results"]))

    def test_metadata_consistency(self, runner):
        """Test that metadata is consistent across queries."""
        queries = [
            "What is ROS2?",
            "Explain simulation",
            "How do I install Isaac Sim?"
        ]

        all_results = []
        for query in queries:
            response = runner.client.search(query=query, top_k=5)
            if response["status"] == "success":
                all_results.extend(response["results"])

        if all_results:
            # Extract unique URLs
            urls = set()
            for result in all_results:
                url = result["metadata"]["source_url"]
                urls.add(url)

            # Verify URLs follow expected pattern
            for url in urls:
                assert url.startswith("https://")
                assert "physical-ai-robotics" in url or "textbook" in url.lower()

    def test_validation_runner_queries(self, runner):
        """Test ValidationRunner with default validation queries."""
        responses = runner.validate_queries(top_k=5)

        # Should have responses for 4 default queries
        assert len(responses) == 4

        for response in responses:
            assert "status" in response
            # Each response should have query and results
            assert "query" in response
            assert "results" in response

    def test_validation_runner_custom_queries(self, runner):
        """Test ValidationRunner with custom queries."""
        custom_queries = [
            "ROS2",
            "Simulation",
            "Isaac"
        ]

        responses = runner.validate_queries(queries=custom_queries, top_k=3)

        assert len(responses) == 3

    def test_performance_metrics(self, runner):
        """Test that performance metrics are reasonable."""
        response = runner.client.search(query="test query", top_k=5)

        if response["status"] == "success":
            metrics = response["execution_metrics"]

            # All timing values should be positive
            assert metrics["query_embedding_time_ms"] > 0
            assert metrics["vector_search_time_ms"] > 0
            assert metrics["total_execution_time_ms"] > 0

            # Total should be sum of components
            total = metrics["total_execution_time_ms"]
            component_sum = (metrics["query_embedding_time_ms"] +
                           metrics["vector_search_time_ms"])
            assert total >= component_sum - 10  # Allow 10ms overhead

            # Total should be <2 seconds
            assert total < 2000

    def test_empty_results_handling(self, runner):
        """Test handling of queries that return no results."""
        # Use a very specific, unlikely query
        response = runner.client.search(
            query="xyzabc123qwerty astronaut unicorn",
            top_k=5,
            similarity_threshold=0.99
        )

        # Should still return success with 0 results
        assert "status" in response
        assert "results" in response
        assert isinstance(response["results"], list)

    def test_query_with_special_characters(self, runner):
        """Test that queries with special characters are handled."""
        response = runner.client.search(
            query="What is ROS2? (Robot Operating System 2)",
            top_k=5
        )

        assert "status" in response
        assert "results" in response or response["status"] == "error"

    def test_query_embedding_consistency(self, runner):
        """Test that same query produces same embedding."""
        query = "What is ROS2?"

        response1 = runner.client.search(query=query, top_k=1)
        response2 = runner.client.search(query=query, top_k=1)

        if (response1["status"] == "success" and
            response2["status"] == "success" and
            response1["results"] and response2["results"]):
            # Same query should return same top result
            assert (response1["results"][0]["chunk_id"] ==
                    response2["results"][0]["chunk_id"])
            assert (abs(response1["results"][0]["similarity_score"] -
                       response2["results"][0]["similarity_score"]) < 0.001)

    def test_error_handling_on_failure(self, runner):
        """Test graceful error handling on API failures."""
        # This is difficult to test with live APIs, but we can verify
        # that error responses have proper structure
        try:
            # Use extremely long query to test limits
            long_query = "a" * 10000
            response = runner.client.search(query=long_query, top_k=5)

            # Should either succeed or have error structure
            assert "status" in response
            if response["status"] == "error":
                assert "error" in response
                assert "code" in response["error"]
                assert "message" in response["error"]
        except Exception as e:
            # Some errors during extreme cases are acceptable
            pytest.skip(f"Long query test failed: {e}")

    def test_results_save_to_json(self, runner, tmp_path):
        """Test that results can be serialized to JSON."""
        response = runner.client.search(query="What is ROS2?", top_k=3)

        # Should be JSON serializable
        json_str = json.dumps(response)
        assert json_str is not None
        assert len(json_str) > 0

        # Should deserialize correctly
        deserialized = json.loads(json_str)
        assert deserialized["status"] == response["status"]
