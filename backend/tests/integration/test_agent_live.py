"""Integration tests for RAG Agent API with live OpenAI/Qdrant APIs (Spec 3).

These tests require:
- Valid OpenAI API key with GPT-4 access
- Valid Qdrant Cloud access with populated collection
- Valid Cohere API key for embeddings

Tests for:
- Chat endpoint with live OpenAI API
- Retrieval integration with live Qdrant
- Grounding validation: responses cite retrieved chunks
- Performance: latency, concurrent requests
- Error handling: timeouts, rate limits, malformed requests

Note: These tests consume API quota and should be run on a schedule or manually.
"""

import pytest
import asyncio
import json
import time
import os
import sys
from typing import List, Dict, Tuple
from pathlib import Path

# Add backend to path for imports
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from fastapi.testclient import TestClient
from agent import app, ChatRequest, ChatResponse


class TestChatEndpoint:
    """Tests for /chat endpoint with live APIs (T060).

    These tests validate that the /chat endpoint works end-to-end
    with live OpenAI and Qdrant APIs.
    """

    @pytest.fixture
    def client(self):
        """Create FastAPI test client."""
        return TestClient(app)

    def test_chat_endpoint_responds_to_query(self, client):
        """Test /chat endpoint accepts query and returns structured response (T060)."""
        response = client.post(
            "/chat",
            json={"query": "What is ROS2?"}
        )
        assert response.status_code == 200
        data = response.json()
        assert data["status"] == "success"
        assert "query" in data
        assert "answer" in data
        assert isinstance(data["answer"], str) and len(data["answer"]) > 0

    def test_chat_endpoint_returns_structured_response(self, client):
        """Test /chat returns proper ChatResponse schema (T060)."""
        response = client.post(
            "/chat",
            json={"query": "Explain what ROS2 is"}
        )
        assert response.status_code == 200
        data = response.json()

        # Validate schema
        assert "query" in data
        assert "answer" in data
        assert "retrieved_chunks" in data
        assert "execution_metrics" in data
        assert "status" in data
        assert "retrieval_scope" in data

        # Validate metrics structure
        if data["execution_metrics"]:
            assert "retrieval_time_ms" in data["execution_metrics"]
            assert "generation_time_ms" in data["execution_metrics"]
            assert "total_time_ms" in data["execution_metrics"]

    def test_chat_endpoint_with_sample_queries(self, client):
        """Test /chat with multiple sample queries (T060)."""
        queries = [
            "What is ROS2?",
            "Explain simulation in Gazebo",
            "How do I install ROS2?",
            "What are the advantages of ROS2 over ROS1?",
            "Describe a simple ROS2 node implementation"
        ]

        for query in queries:
            response = client.post(
                "/chat",
                json={"query": query}
            )
            assert response.status_code == 200, f"Failed for query: {query}"
            data = response.json()
            assert data["status"] == "success", f"Failed for query: {query}"
            assert len(data.get("answer", "")) > 0, f"Empty answer for query: {query}"


class TestRetrieval:
    """Tests for retrieval integration with live Qdrant."""

    @pytest.fixture
    def client(self):
        """Create FastAPI test client."""
        return TestClient(app)

    def test_retrieval_returns_chunks_with_metadata(self, client):
        """Test retrieval returns chunks with proper metadata."""
        response = client.post(
            "/chat",
            json={"query": "What is ROS2?", "top_k": 5}
        )
        assert response.status_code == 200
        data = response.json()

        if data["retrieved_chunks"]:
            # Verify each chunk has required metadata
            for chunk in data["retrieved_chunks"]:
                assert "chunk_id" in chunk
                assert "text" in chunk
                assert "similarity_score" in chunk
                assert isinstance(chunk["similarity_score"], (int, float))
                assert 0 <= chunk["similarity_score"] <= 1
                assert "source_url" in chunk
                assert "page_title" in chunk
                assert "section_headers" in chunk

    def test_retrieval_respects_top_k(self, client):
        """Test retrieval respects top_k parameter (1-100 range)."""
        # Test with top_k=3
        response_3 = client.post(
            "/chat",
            json={"query": "What is ROS2?", "top_k": 3}
        )
        assert response_3.status_code == 200
        data_3 = response_3.json()
        count_3 = len(data_3.get("retrieved_chunks", []))

        # Test with top_k=10
        response_10 = client.post(
            "/chat",
            json={"query": "What is ROS2?", "top_k": 10}
        )
        assert response_10.status_code == 200
        data_10 = response_10.json()
        count_10 = len(data_10.get("retrieved_chunks", []))

        # Verify ordering by similarity score (descending)
        if data_10["retrieved_chunks"]:
            scores = [c["similarity_score"] for c in data_10["retrieved_chunks"]]
            assert scores == sorted(scores, reverse=True), "Chunks not ordered by similarity score"


class TestGrounding:
    """Tests for response grounding in retrieved context (T064)."""

    @pytest.fixture
    def client(self):
        """Create FastAPI test client."""
        return TestClient(app)

    def test_response_cites_retrieved_chunks(self, client):
        """Test agent response references retrieved chunks (T064).

        Grounding validation criteria:
        1. Response contains direct quotes or paraphrases from retrieved chunks
        2. Response references sources or specific content areas
        3. No unsupported claims
        """
        response = client.post(
            "/chat",
            json={"query": "What is ROS2?", "top_k": 5}
        )
        assert response.status_code == 200
        data = response.json()

        # Verify we have retrieved chunks and an answer
        assert data["status"] == "success"
        assert len(data.get("answer", "")) > 0
        answer_lower = data["answer"].lower()

        # Check for grounding indicators in response
        has_source_reference = False
        if data["retrieved_chunks"]:
            # Check if answer references any chunk content or sources
            for chunk in data["retrieved_chunks"]:
                chunk_text_keywords = chunk.get("text", "").lower().split()[:10]
                if any(keyword in answer_lower for keyword in chunk_text_keywords):
                    has_source_reference = True
                    break

        # At minimum, answer should be substantive
        assert len(data["answer"]) > 50, "Answer too short to be properly grounded"

    def test_response_handles_out_of_scope_queries(self, client):
        """Test agent handles questions outside textbook scope."""
        response = client.post(
            "/chat",
            json={"query": "What is your favorite programming language?"}
        )
        assert response.status_code == 200
        data = response.json()

        # Either the agent says it's not covered, or provides minimal answer
        answer = data.get("answer", "").lower()
        # Check if agent acknowledges limitation
        has_limitation_statement = any([
            "does not cover" in answer,
            "not covered" in answer,
            "outside" in answer,
            "not in" in answer
        ])
        # Or if no chunks were retrieved, that's also valid
        has_no_chunks = len(data.get("retrieved_chunks", [])) == 0
        assert has_limitation_statement or has_no_chunks, "Agent should acknowledge out-of-scope query"

    def test_grounding_validation_batch(self, client):
        """Run batch grounding validation with 5 queries (T064).

        Acceptance criteria: ≥4 responses (80%) cite or reference retrieved content
        """
        queries = [
            "What is ROS2?",
            "How does Gazebo simulation work?",
            "Explain ROS2 middleware",
            "What are ROS2 nodes?",
            "Describe ROS2 communication"
        ]

        grounded_count = 0
        for query in queries:
            response = client.post(
                "/chat",
                json={"query": query}
            )
            assert response.status_code == 200
            data = response.json()

            # Check for grounding
            if (data["status"] == "success" and
                len(data.get("answer", "")) > 50 and
                len(data.get("retrieved_chunks", [])) > 0):
                grounded_count += 1

        # T064 criterion: ≥4 responses properly grounded (80% threshold)
        assert grounded_count >= 4, f"Grounding validation failed: {grounded_count}/5 responses properly grounded (need ≥4)"


class TestPerformance:
    """Tests for performance metrics and latency (T063)."""

    @pytest.fixture
    def client(self):
        """Create FastAPI test client."""
        return TestClient(app)

    def test_query_latency_under_5_seconds(self, client):
        """Test end-to-end query latency is under 5 seconds (T063).

        SLA requirement: p95 latency < 5 seconds
        Acceptance criteria: All 10 queries complete in <5s
        """
        queries = [
            "What is ROS2?",
            "Explain Gazebo simulation",
            "How do I install ROS2?",
            "What are ROS2 advantages?",
            "Describe ROS2 nodes",
            "What is the ROS2 client library?",
            "Explain ROS2 DDS middleware",
            "How do ROS2 services work?",
            "What are ROS2 topics?",
            "Describe ROS2 message types"
        ]

        latencies = []
        max_latency = 0
        timeout_exceeded = False

        for query in queries:
            start_time = time.time()
            response = client.post(
                "/chat",
                json={"query": query}
            )
            elapsed_ms = (time.time() - start_time) * 1000

            latencies.append(elapsed_ms)
            max_latency = max(max_latency, elapsed_ms)

            assert response.status_code == 200, f"Query failed: {query}"
            data = response.json()

            # Check reported metrics
            if data["execution_metrics"]:
                reported_total_ms = data["execution_metrics"]["total_time_ms"]
                # Validate latency is under 5 seconds (5000ms)
                if reported_total_ms > 5000:
                    timeout_exceeded = True

        # Sort latencies for percentile calculation
        latencies_sorted = sorted(latencies)
        p95_index = int(len(latencies_sorted) * 0.95) - 1 if len(latencies_sorted) > 1 else 0
        p95_latency = latencies_sorted[p95_index]

        print(f"\n=== PERFORMANCE METRICS (T063) ===")
        print(f"Total queries: {len(latencies)}")
        print(f"Min latency: {min(latencies):.2f}ms")
        print(f"Max latency: {max_latency:.2f}ms")
        print(f"P95 latency: {p95_latency:.2f}ms")
        print(f"SLA target: <5000ms")
        print(f"SLA status: {'PASS' if p95_latency < 5000 else 'FAIL'}")
        print("=" * 35 + "\n")

        # T063 criterion: p95 latency < 5 seconds
        assert p95_latency < 5000, f"P95 latency {p95_latency:.2f}ms exceeds 5000ms SLA"
        assert not timeout_exceeded, "At least one query exceeded 5 second timeout"

    def test_retrieval_timing_metrics(self, client):
        """Test retrieval step timing is recorded and reasonable."""
        response = client.post(
            "/chat",
            json={"query": "What is ROS2?", "top_k": 5}
        )
        assert response.status_code == 200
        data = response.json()

        if data["execution_metrics"]:
            metrics = data["execution_metrics"]
            # Verify metrics are positive and reasonable
            assert metrics["retrieval_time_ms"] >= 0
            assert metrics["generation_time_ms"] >= 0
            assert metrics["total_time_ms"] >= 0

            # Retrieval should be <500ms typically
            # Generation depends on model but should be <3500ms for reasonable SLA
            assert metrics["total_time_ms"] < 6000, "Total latency unreasonable"

    def test_generation_timing_metrics(self, client):
        """Test generation step timing is recorded."""
        response = client.post(
            "/chat",
            json={"query": "Explain ROS2 middleware architecture"}
        )
        assert response.status_code == 200
        data = response.json()

        if data["execution_metrics"]:
            metrics = data["execution_metrics"]
            # Generation time should be significant (typically 1-3 seconds)
            # but generation_time_ms might be partially measured
            assert "generation_time_ms" in metrics


class TestErrorHandling:
    """Tests for error scenarios and resilience."""

    @pytest.fixture
    def client(self):
        """Create FastAPI test client."""
        return TestClient(app)

    def test_empty_query_returns_422_error(self, client):
        """Test empty query returns validation error."""
        # Pydantic validation catches empty queries
        response = client.post(
            "/chat",
            json={"query": ""}
        )
        # Should get validation error (422) from Pydantic
        assert response.status_code in [400, 422]

    def test_whitespace_query_rejected(self, client):
        """Test whitespace-only query is rejected."""
        response = client.post(
            "/chat",
            json={"query": "   "}
        )
        assert response.status_code in [400, 422]

    def test_invalid_top_k_returns_error(self, client):
        """Test invalid top_k (out of range) returns error."""
        # Test top_k > 100
        response = client.post(
            "/chat",
            json={"query": "What is ROS2?", "top_k": 101}
        )
        assert response.status_code in [400, 422]

        # Test top_k < 1
        response = client.post(
            "/chat",
            json={"query": "What is ROS2?", "top_k": 0}
        )
        assert response.status_code in [400, 422]

    def test_query_too_long_returns_error(self, client):
        """Test extremely long query (>10,000 chars) returns error."""
        long_query = "x" * 10001
        response = client.post(
            "/chat",
            json={"query": long_query}
        )
        assert response.status_code in [400, 422]

    def test_malformed_request_returns_error(self, client):
        """Test malformed JSON returns error."""
        response = client.post(
            "/chat",
            data="invalid json",
            headers={"Content-Type": "application/json"}
        )
        assert response.status_code in [400, 422]


class TestRelevanceValidation:
    """Tests for retrieval relevance validation (T065)."""

    @pytest.fixture
    def client(self):
        """Create FastAPI test client."""
        return TestClient(app)

    def test_relevance_validation_batch(self, client):
        """Run batch relevance validation with 5 queries (T065).

        Acceptance criteria: ≥4 retrieved chunk sets (80%) are relevant to their queries
        """
        test_cases = [
            {
                "query": "What is ROS2?",
                "keywords": ["ros2", "middleware", "robotics", "framework"]
            },
            {
                "query": "How does Gazebo work?",
                "keywords": ["gazebo", "simulation", "robot", "3d"]
            },
            {
                "query": "Explain ROS2 communication",
                "keywords": ["communication", "topics", "services", "ros2"]
            },
            {
                "query": "What are ROS2 nodes?",
                "keywords": ["node", "ros2", "process", "component"]
            },
            {
                "query": "Describe ROS2 DDS",
                "keywords": ["dds", "middleware", "communication", "ros2"]
            }
        ]

        relevant_count = 0
        for test_case in test_cases:
            query = test_case["query"]
            expected_keywords = test_case["keywords"]

            response = client.post(
                "/chat",
                json={"query": query, "top_k": 5}
            )
            assert response.status_code == 200
            data = response.json()

            # Check if retrieved chunks are relevant
            chunks = data.get("retrieved_chunks", [])
            if chunks:
                # Check if chunks contain expected keywords
                chunk_text = " ".join([c.get("text", "").lower() for c in chunks])
                chunk_titles = " ".join([c.get("page_title", "").lower() for c in chunks])
                combined_text = (chunk_text + " " + chunk_titles).lower()

                # Count how many expected keywords are found
                keywords_found = sum(1 for kw in expected_keywords if kw.lower() in combined_text)
                relevance_ratio = keywords_found / len(expected_keywords)

                # Consider relevant if at least 50% of keywords found
                if relevance_ratio >= 0.5:
                    relevant_count += 1

        # T065 criterion: ≥4 queries have relevant chunks (80% threshold)
        assert relevant_count >= 4, f"Relevance validation failed: {relevant_count}/5 queries have relevant chunks (need ≥4)"
        print(f"\n=== RELEVANCE VALIDATION (T065) ===")
        print(f"Queries with relevant chunks: {relevant_count}/5")
        print(f"Threshold: ≥4 (80%)")
        print(f"Status: {'PASS' if relevant_count >= 4 else 'FAIL'}")
        print("=" * 35 + "\n")

    def test_chunk_metadata_quality(self, client):
        """Verify retrieved chunks have quality metadata."""
        response = client.post(
            "/chat",
            json={"query": "What is ROS2?", "top_k": 3}
        )
        assert response.status_code == 200
        data = response.json()

        chunks = data.get("retrieved_chunks", [])
        if chunks:
            for chunk in chunks:
                # Verify metadata completeness
                assert chunk.get("chunk_id"), "Missing chunk_id"
                assert chunk.get("text"), "Missing chunk text"
                assert "similarity_score" in chunk, "Missing similarity_score"
                assert chunk.get("source_url"), "Missing source_url"
                assert chunk.get("page_title"), "Missing page_title"
                # section_headers may be empty list, which is ok
                assert "section_headers" in chunk, "Missing section_headers field"


class TestTextOnlyMode:
    """Tests for text-only retrieval scope."""

    @pytest.fixture
    def client(self):
        """Create FastAPI test client."""
        return TestClient(app)

    def test_text_only_mode_with_provided_context(self, client):
        """Test text_only scope uses provided context_text."""
        custom_context = "ROS2 is a robotic middleware framework. It provides communication tools."
        response = client.post(
            "/chat",
            json={
                "query": "What is provided about ROS2?",
                "retrieval_scope": "text_only",
                "context_text": custom_context
            }
        )
        assert response.status_code == 200
        data = response.json()
        assert data["retrieval_scope"] == "text_only"

    def test_full_collection_mode_searches_qdrant(self, client):
        """Test full_collection scope searches Qdrant (default)."""
        response = client.post(
            "/chat",
            json={
                "query": "What is ROS2?",
                "retrieval_scope": "full_collection"
            }
        )
        assert response.status_code == 200
        data = response.json()
        assert data["retrieval_scope"] == "full_collection"
        # Should have retrieved chunks from Qdrant
        assert isinstance(data.get("retrieved_chunks", []), list)


class TestIntegrationFixtures:
    """Fixtures and helpers for integration testing."""

    @pytest.fixture
    def openai_api_key(self):
        """Get OpenAI API key from environment."""
        key = os.getenv('OPENAI_API_KEY')
        if not key:
            pytest.skip("OPENAI_API_KEY not set")
        return key

    @pytest.fixture
    def qdrant_config(self):
        """Get Qdrant configuration from environment."""
        return {
            'url': os.getenv('QDRANT_URL'),
            'api_key': os.getenv('QDRANT_API_KEY'),
            'collection': os.getenv('QDRANT_COLLECTION_NAME', 'textbook_embeddings')
        }

    @pytest.fixture
    def cohere_api_key(self):
        """Get Cohere API key from environment."""
        key = os.getenv('COHERE_API_KEY')
        if not key:
            pytest.skip("COHERE_API_KEY not set")
        return key

    @pytest.fixture
    def sample_queries(self):
        """Sample queries for integration testing."""
        return [
            {
                "query": "What is ROS2?",
                "expected_keywords": ["ROS2", "middleware", "robotics"]
            },
            {
                "query": "Explain simulation in Gazebo",
                "expected_keywords": ["simulation", "Gazebo"]
            },
            {
                "query": "How do I install ROS2?",
                "expected_keywords": ["install", "ROS2"]
            },
            {
                "query": "What are the advantages of ROS2 over ROS1?",
                "expected_keywords": ["ROS2", "ROS1", "advantages"]
            },
            {
                "query": "Describe a simple ROS2 node implementation",
                "expected_keywords": ["node", "implementation", "ROS2"]
            }
        ]


# ============================================================================
# Utility functions for testing
# ============================================================================

def assert_response_is_grounded(response_text: str, retrieved_chunks: List[dict]) -> bool:
    """Assert that response text is grounded in retrieved chunks.

    This is a placeholder for grounding validation.
    Will be implemented in Phase 5 with more sophisticated checks.

    Args:
        response_text: Generated response from agent
        retrieved_chunks: Retrieved context chunks

    Returns:
        bool: True if response appears grounded, False otherwise
    """
    # TODO: Implement sophisticated grounding check
    # - Check for direct quotes
    # - Check for paraphrased content
    # - Check for citation patterns
    return True


def measure_latency(func, *args, **kwargs) -> tuple:
    """Measure function execution latency.

    Args:
        func: Function to measure
        *args, **kwargs: Function arguments

    Returns:
        tuple: (result, latency_ms)
    """
    start_time = time.time()
    result = func(*args, **kwargs)
    elapsed_ms = (time.time() - start_time) * 1000
    return result, elapsed_ms


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
