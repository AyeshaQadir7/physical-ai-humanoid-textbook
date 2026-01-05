"""Unit tests for RAG Agent API (Spec 3).

Tests for:
- AgentConfig: Environment variable loading and validation
- Request schema validation: ChatRequest, RetrievedChunk
- Response schema: ChatResponse
- GroundedAgent placeholder
- Chat endpoint placeholder (will be tested in Phase 3+)
"""

import pytest
from unittest.mock import MagicMock, patch
import os
from pydantic import ValidationError

# Import agent module
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from agent import (
    AgentConfig,
    ChatRequest,
    ChatResponse,
    RetrievedChunk,
    ExecutionMetrics,
    ErrorInfo,
    GroundedAgent,
)


# ============================================================================
# Tests for AgentConfig
# ============================================================================

class TestAgentConfig:
    """Test AgentConfig environment variable loading and validation."""

    def test_agent_config_loads_from_env(self):
        """Test AgentConfig loads all required environment variables."""
        # Set up environment variables
        os.environ['OPENAI_API_KEY'] = 'test-key-123'
        os.environ['QDRANT_URL'] = 'https://test.qdrant.io'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'
        os.environ['COHERE_API_KEY'] = 'test-cohere-key'

        try:
            config = AgentConfig()
            assert config.openai_api_key == 'test-key-123'
            assert config.qdrant_url == 'https://test.qdrant.io'
            assert config.qdrant_api_key == 'test-qdrant-key'
            assert config.cohere_api_key == 'test-cohere-key'
        finally:
            # Clean up
            for key in ['OPENAI_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY', 'COHERE_API_KEY']:
                os.environ.pop(key, None)

    def test_agent_config_validates_required_vars(self):
        """Test AgentConfig validates required environment variables."""
        # Clear environment variables
        for key in ['OPENAI_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY', 'COHERE_API_KEY']:
            os.environ.pop(key, None)

        # Should raise ValueError when OPENAI_API_KEY is missing
        with pytest.raises(ValueError, match="OPENAI_API_KEY"):
            AgentConfig()

    def test_agent_config_defaults(self):
        """Test AgentConfig uses sensible defaults."""
        os.environ['OPENAI_API_KEY'] = 'test-key'
        os.environ['QDRANT_URL'] = 'https://test.qdrant.io'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'
        os.environ['COHERE_API_KEY'] = 'test-cohere-key'

        try:
            config = AgentConfig()
            assert config.openai_model == 'gpt-3.5-turbo'  # Default model
            assert config.collection_name == 'textbook_embeddings'  # Default collection
            assert config.log_level == 'INFO'  # Default log level
        finally:
            for key in ['OPENAI_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY', 'COHERE_API_KEY']:
                os.environ.pop(key, None)


# ============================================================================
# Tests for Pydantic Schemas
# ============================================================================

class TestChatRequest:
    """Test ChatRequest schema validation."""

    def test_chat_request_valid(self):
        """Test ChatRequest accepts valid input."""
        request = ChatRequest(
            query="What is ROS2?",
            retrieval_scope="full_collection",
            top_k=5,
            context_text=None
        )
        assert request.query == "What is ROS2?"
        assert request.retrieval_scope == "full_collection"
        assert request.top_k == 5
        assert request.context_text is None

    def test_chat_request_query_not_empty(self):
        """Test ChatRequest rejects empty query."""
        with pytest.raises(ValidationError) as exc_info:
            ChatRequest(query="")
        assert "at least 1 character" in str(exc_info.value).lower()

    def test_chat_request_query_not_whitespace(self):
        """Test ChatRequest rejects whitespace-only query."""
        with pytest.raises(ValidationError):
            ChatRequest(query="   ")

    def test_chat_request_top_k_validation(self):
        """Test ChatRequest validates top_k range (1-100)."""
        # Valid
        request = ChatRequest(query="test", top_k=1)
        assert request.top_k == 1

        request = ChatRequest(query="test", top_k=100)
        assert request.top_k == 100

        # Invalid - too low
        with pytest.raises(ValidationError):
            ChatRequest(query="test", top_k=0)

        # Invalid - too high
        with pytest.raises(ValidationError):
            ChatRequest(query="test", top_k=101)

    def test_chat_request_query_max_length(self):
        """Test ChatRequest enforces max query length (10,000 chars)."""
        # Valid: at limit
        request = ChatRequest(query="a" * 10000)
        assert len(request.query) == 10000

        # Invalid: over limit
        with pytest.raises(ValidationError):
            ChatRequest(query="a" * 10001)

    def test_chat_request_retrieval_scope_literal(self):
        """Test ChatRequest validates retrieval_scope is text_only or full_collection."""
        # Valid
        ChatRequest(query="test", retrieval_scope="text_only")
        ChatRequest(query="test", retrieval_scope="full_collection")

        # Invalid
        with pytest.raises(ValidationError):
            ChatRequest(query="test", retrieval_scope="invalid_scope")


class TestRetrievedChunk:
    """Test RetrievedChunk schema."""

    def test_retrieved_chunk_valid(self):
        """Test RetrievedChunk accepts valid chunk."""
        chunk = RetrievedChunk(
            chunk_id="chunk_123",
            text="ROS2 is a flexible middleware...",
            similarity_score=0.85,
            source_url="https://example.com/chapter1",
            page_title="Introduction to ROS2",
            section_headers=["Robotics", "ROS2 Basics"]
        )
        assert chunk.chunk_id == "chunk_123"
        assert chunk.similarity_score == 0.85

    def test_retrieved_chunk_similarity_score_bounds(self):
        """Test RetrievedChunk validates similarity score is 0.0-1.0."""
        # Valid
        chunk = RetrievedChunk(
            chunk_id="c1", text="test", similarity_score=0.0,
            source_url="url", page_title="title"
        )
        assert chunk.similarity_score == 0.0

        chunk = RetrievedChunk(
            chunk_id="c1", text="test", similarity_score=1.0,
            source_url="url", page_title="title"
        )
        assert chunk.similarity_score == 1.0

        # Invalid
        with pytest.raises(ValidationError):
            RetrievedChunk(
                chunk_id="c1", text="test", similarity_score=-0.1,
                source_url="url", page_title="title"
            )

        with pytest.raises(ValidationError):
            RetrievedChunk(
                chunk_id="c1", text="test", similarity_score=1.1,
                source_url="url", page_title="title"
            )


class TestChatResponse:
    """Test ChatResponse schema."""

    def test_chat_response_success(self):
        """Test ChatResponse accepts valid success response."""
        response = ChatResponse(
            query="What is ROS2?",
            answer="ROS2 is a flexible middleware...",
            retrieved_chunks=[],
            execution_metrics=ExecutionMetrics(
                retrieval_time_ms=200.0,
                generation_time_ms=2500.0,
                total_time_ms=2800.0
            ),
            status="success",
            error=None
        )
        assert response.status == "success"
        assert response.error is None
        assert response.execution_metrics.total_time_ms == 2800.0

    def test_chat_response_error(self):
        """Test ChatResponse accepts error response."""
        error = ErrorInfo(
            code="EMPTY_QUERY",
            message="Query cannot be empty"
        )
        response = ChatResponse(
            query="",
            answer=None,
            retrieved_chunks=[],
            execution_metrics=None,
            status="error",
            error=error
        )
        assert response.status == "error"
        assert response.error.code == "EMPTY_QUERY"


# ============================================================================
# Tests for GroundedAgent (Placeholder)
# ============================================================================

class TestChatRequestPhase6:
    """Test ChatRequest Phase 6 features - Text-only and full-collection modes (T035-T037)."""

    def test_chat_request_text_only_mode(self):
        """Test ChatRequest accepts text_only retrieval scope (T036)."""
        request = ChatRequest(
            query="Test query",
            retrieval_scope="text_only",
            context_text="Custom context for testing"
        )
        assert request.retrieval_scope == "text_only"
        assert request.context_text == "Custom context for testing"

    def test_chat_request_full_collection_mode(self):
        """Test ChatRequest accepts full_collection retrieval scope (T037)."""
        request = ChatRequest(
            query="Test query",
            retrieval_scope="full_collection",
            top_k=5
        )
        assert request.retrieval_scope == "full_collection"
        assert request.top_k == 5

    def test_chat_request_default_retrieval_scope(self):
        """Test ChatRequest defaults to full_collection retrieval scope (T037)."""
        request = ChatRequest(query="Test query")
        assert request.retrieval_scope == "full_collection"


class TestChatResponsePhase6:
    """Test ChatResponse Phase 6 features - Retrieval scope tracking (T039)."""

    def test_chat_response_includes_retrieval_scope_text_only(self):
        """Test ChatResponse includes retrieval_scope field for text_only mode (T039)."""
        response = ChatResponse(
            query="What is ROS2?",
            answer="ROS2 is...",
            retrieved_chunks=[],
            execution_metrics=ExecutionMetrics(
                retrieval_time_ms=0.0,
                generation_time_ms=100.0,
                total_time_ms=100.0
            ),
            retrieval_scope="text_only",
            status="success",
            error=None
        )
        assert response.retrieval_scope == "text_only"

    def test_chat_response_includes_retrieval_scope_full_collection(self):
        """Test ChatResponse includes retrieval_scope field for full_collection mode (T039)."""
        response = ChatResponse(
            query="What is ROS2?",
            answer="ROS2 is...",
            retrieved_chunks=[],
            execution_metrics=ExecutionMetrics(
                retrieval_time_ms=100.0,
                generation_time_ms=100.0,
                total_time_ms=200.0
            ),
            retrieval_scope="full_collection",
            status="success",
            error=None
        )
        assert response.retrieval_scope == "full_collection"

    def test_chat_response_schema_consistent(self):
        """Test ChatResponse schema is consistent across both retrieval modes (T042)."""
        # Text-only response
        text_only_response = ChatResponse(
            query="Test query",
            answer="Test answer",
            retrieved_chunks=[],
            execution_metrics=ExecutionMetrics(
                retrieval_time_ms=10.0,
                generation_time_ms=100.0,
                total_time_ms=110.0
            ),
            retrieval_scope="text_only",
            status="success"
        )

        # Full-collection response
        full_collection_response = ChatResponse(
            query="Test query",
            answer="Test answer",
            retrieved_chunks=[],
            execution_metrics=ExecutionMetrics(
                retrieval_time_ms=50.0,
                generation_time_ms=100.0,
                total_time_ms=150.0
            ),
            retrieval_scope="full_collection",
            status="success"
        )

        # Both should have the same schema structure
        text_only_dict = text_only_response.dict()
        full_collection_dict = full_collection_response.dict()

        # Both should have retrieval_scope field
        assert "retrieval_scope" in text_only_dict
        assert "retrieval_scope" in full_collection_dict

        # Both should have all required fields
        for key in ["query", "answer", "retrieved_chunks", "execution_metrics", "status"]:
            assert key in text_only_dict
            assert key in full_collection_dict


class TestErrorHandlingPhase7:
    """Test error handling Phase 7 features - Error responses and status codes (T043-T053)."""

    def test_empty_query_validation_error(self):
        """Test empty query raises validation error (T043)."""
        with pytest.raises(ValidationError) as exc_info:
            ChatRequest(query="")
        assert "at least 1 character" in str(exc_info.value).lower()

    def test_whitespace_only_query_validation_error(self):
        """Test whitespace-only query raises validation error (T043)."""
        with pytest.raises(ValidationError):
            ChatRequest(query="   ")

    def test_query_too_long_validation_error(self):
        """Test query exceeding 10,000 chars raises validation error (T049)."""
        with pytest.raises(ValidationError) as exc_info:
            ChatRequest(query="a" * 10001)
        assert "at most 10000 characters" in str(exc_info.value).lower()

    def test_invalid_top_k_too_low_validation_error(self):
        """Test top_k below 1 raises validation error (T045)."""
        with pytest.raises(ValidationError):
            ChatRequest(query="test", top_k=0)

    def test_invalid_top_k_too_high_validation_error(self):
        """Test top_k above 100 raises validation error (T045)."""
        with pytest.raises(ValidationError):
            ChatRequest(query="test", top_k=101)

    def test_chat_response_error_field_structure(self):
        """Test ChatResponse error field has code and message (T050)."""
        error = ErrorInfo(
            code="TEST_ERROR",
            message="Test error message"
        )
        assert error.code == "TEST_ERROR"
        assert error.message == "Test error message"

    def test_chat_response_error_status(self):
        """Test ChatResponse with error status includes error details (T050)."""
        response = ChatResponse(
            query="Test query",
            answer=None,
            retrieved_chunks=[],
            execution_metrics=None,
            retrieval_scope="full_collection",
            status="error",
            error=ErrorInfo(
                code="RETRIEVAL_FAILED",
                message="Retrieval service unavailable, please retry"
            )
        )
        assert response.status == "error"
        assert response.error is not None
        assert response.error.code == "RETRIEVAL_FAILED"
        assert response.answer is None

    def test_error_response_includes_retrieval_scope(self):
        """Test error responses include retrieval_scope for auditability (T039, T053)."""
        error_response = ChatResponse(
            query="Test",
            answer=None,
            retrieved_chunks=[],
            execution_metrics=None,
            retrieval_scope="text_only",
            status="error",
            error=ErrorInfo(code="TEST_ERROR", message="Test")
        )
        assert error_response.retrieval_scope == "text_only"

    def test_validation_error_codes(self):
        """Test error codes for validation errors are consistent (T043-T049)."""
        # Test various validation error codes
        error_codes = [
            "EMPTY_QUERY",
            "VALIDATION_ERROR",
            "RETRIEVAL_FAILED",
            "AGENT_FAILED",
            "OPENAI_TIMEOUT",
            "RATE_LIMITED",
            "GENERATION_FAILED"
        ]
        for code in error_codes:
            error = ErrorInfo(code=code, message="Test message")
            assert error.code == code


class TestGroundedAgent:
    """Test GroundedAgent initialization with Agents SDK."""

    @patch('agent.OPENAI_AVAILABLE', True)
    @patch('agent.AGENTS_SDK_AVAILABLE', True)
    def test_grounded_agent_init_with_agents_sdk(self):
        """Test GroundedAgent initializes with Agents SDK Agent instance."""
        os.environ['OPENAI_API_KEY'] = 'test-key'
        os.environ['QDRANT_URL'] = 'https://test.qdrant.io'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'
        os.environ['COHERE_API_KEY'] = 'test-cohere-key'

        try:
            config = AgentConfig()
            agent = GroundedAgent(config)
            assert agent.config == config
            assert agent.instructions is not None
            assert agent.agent is not None  # Agents SDK Agent instance
            assert "retrieve_textbook_chunks" in agent.instructions.lower()
        finally:
            for key in ['OPENAI_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY', 'COHERE_API_KEY']:
                os.environ.pop(key, None)

    @patch('agent.OPENAI_AVAILABLE', True)
    @patch('agent.AGENTS_SDK_AVAILABLE', True)
    def test_grounded_agent_has_generate_response(self):
        """Test GroundedAgent.generate_response() is implemented for Agents SDK."""
        os.environ['OPENAI_API_KEY'] = 'test-key'
        os.environ['QDRANT_URL'] = 'https://test.qdrant.io'
        os.environ['QDRANT_API_KEY'] = 'test-qdrant-key'
        os.environ['COHERE_API_KEY'] = 'test-cohere-key'

        try:
            config = AgentConfig()
            agent = GroundedAgent(config)
            # Test that method exists and is callable
            assert hasattr(agent, 'generate_response')
            assert callable(agent.generate_response)
            assert hasattr(agent, 'generate_response_async')
            assert callable(agent.generate_response_async)
        finally:
            for key in ['OPENAI_API_KEY', 'QDRANT_URL', 'QDRANT_API_KEY', 'COHERE_API_KEY']:
                os.environ.pop(key, None)


# ============================================================================
# Integration Fixtures for Later Use (Phase 3+)
# ============================================================================

@pytest.fixture
def valid_chat_request():
    """Fixture: Valid chat request for testing endpoints."""
    return ChatRequest(
        query="What is ROS2?",
        retrieval_scope="full_collection",
        top_k=5
    )


@pytest.fixture
def sample_retrieved_chunks():
    """Fixture: Sample retrieved chunks for response testing."""
    return [
        RetrievedChunk(
            chunk_id="chunk_1",
            text="ROS2 is a flexible middleware for robotics applications.",
            similarity_score=0.90,
            source_url="https://example.com/chapter1",
            page_title="Introduction to ROS2",
            section_headers=["Robotics", "ROS2 Basics"]
        ),
        RetrievedChunk(
            chunk_id="chunk_2",
            text="Key features of ROS2 include improved performance and security.",
            similarity_score=0.85,
            source_url="https://example.com/chapter1",
            page_title="Introduction to ROS2",
            section_headers=["Robotics", "ROS2 Features"]
        )
    ]


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
