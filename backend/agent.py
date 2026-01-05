"""
RAG Agent API - OpenAI Agents SDK with Qdrant Retrieval Integration

This module implements a FastAPI-based RAG (Retrieval-Augmented Generation) agent service
that answers natural language questions about the textbook by retrieving relevant content
from Qdrant and using OpenAI Agents SDK to generate grounded responses.

Architecture:
- AgentConfig: Environment variable loading and validation
- GroundedAgent: Wrapper around OpenAI Agents SDK with retrieval integration
- FastAPI app: HTTP endpoint (/chat) for querying the agent
- Request/Response schemas: Pydantic models for validation and serialization

Dependencies:
- openai: OpenAI API and Agents SDK
- fastapi: HTTP framework
- pydantic: Request/response validation
- backend.retrieve: RetrieverClient for Qdrant semantic search (from Spec 2)
"""

import os
import time
import logging
import asyncio
from typing import Optional, List, Literal, Union
from datetime import datetime

from agents import OpenAIChatCompletionsModel
from openai import AsyncOpenAI
from pydantic import BaseModel, Field, validator
from fastapi import FastAPI, HTTPException, status, Response
from fastapi.middleware.cors import CORSMiddleware

from agents import OpenAIChatCompletionsModel
from openai import AsyncOpenAI
from dotenv import load_dotenv

load_dotenv()

client = AsyncOpenAI(
    api_key=os.getenv("OPENROUTER_API_KEY"),
    base_url="https://openrouter.ai/api/v1",
)

third_party_model = OpenAIChatCompletionsModel(
    openai_client=client,
    model="mistralai/devstral-2512:free",
)
# Import RetrieverClient from Spec 2 for semantic search
try:
    from retrieve import RetrieverClient

    RETRIEVER_AVAILABLE = True
except ImportError as e:
    logger.warning(f"RetrieverClient not available: {e}. Spec 2 integration will fail.")
    RETRIEVER_AVAILABLE = False

# OpenAI Agents SDK integration
try:
    from openai import OpenAI, APITimeoutError, RateLimitError, APIConnectionError
    from agents import Agent, Runner, function_tool

    from dotenv import load_dotenv

    load_dotenv()
    # Initialize OpenAI client and Agents SDK
    # Note: Uses OpenAI API directly (not OpenRouter) for Agents SDK compatibility
    OPENAI_AVAILABLE = True
    AGENTS_SDK_AVAILABLE = True
except ImportError as e:
    logger.warning(f"OpenAI SDK or Agents SDK not available: {e}")
    OPENAI_AVAILABLE = False
    AGENTS_SDK_AVAILABLE = False
    OpenAI = None
    Agent = None
    Runner = None
    function_tool = None
    APITimeoutError = None  # Placeholder
    RateLimitError = None  # Placeholder
    APIConnectionError = None  # Placeholder

# Configure logging
logging.basicConfig(
    level=logging.INFO, format="%(asctime)s - %(name)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)


# ============================================================================
# Configuration
# ============================================================================


class AgentConfig:
    """Load and validate environment variables for agent configuration.

    This class encapsulates all configuration needed for the RAG Agent API,
    including credentials for OpenAI, Qdrant, and Cohere services. It performs
    validation on initialization to fail fast if configuration is incomplete.

    Attributes:

        qdrant_url (str): Qdrant Cloud URL
        qdrant_api_key (str): Qdrant API key
        cohere_api_key (str): Cohere API key for query embeddings
        collection_name (str): Qdrant collection name (default: textbook_embeddings)
        log_level (str): Logging level (default: INFO)
        retriever_client (RetrieverClient): Client for semantic search

    Environment Variables (Required):
        OPENROUTER_API_KEY: OPENROUTER API key
        QDRANT_URL: Qdrant Cloud URL (inherited from Spec 1)
        QDRANT_API_KEY: Qdrant API key (inherited from Spec 1)
        COHERE_API_KEY: Cohere API key for query embedding (inherited from Spec 2)

    Environment Variables (Optional):

        QDRANT_COLLECTION_NAME: Qdrant collection name (default: textbook_embeddings)
        LOG_LEVEL: Logging level (default: INFO)

    Raises:
        ValueError: If any required environment variable is missing
        ValueError: If RetrieverClient fails to initialize
    """

    def __init__(self):
        """Initialize configuration from environment variables."""
        # OpenAI configuration (for Agents SDK)
        self.openai_api_key = os.getenv("OPENAI_API_KEY")
        self.openai_model = os.getenv("OPENAI_MODEL", "gpt-3.5-turbo")

        # Qdrant configuration
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")

        # Cohere configuration
        self.cohere_api_key = os.getenv("COHERE_API_KEY")

        # Agent configuration
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "textbook_embeddings")
        self.log_level = os.getenv("LOG_LEVEL", "INFO")
        self.use_agents_sdk = os.getenv("USE_AGENTS_SDK", "true").lower() == "true"

        # Validate critical variables
        if not self.openai_api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")
        if not self.qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")
        if not self.qdrant_api_key:
            raise ValueError("QDRANT_API_KEY environment variable is required")
        if not self.cohere_api_key:
            raise ValueError("COHERE_API_KEY environment variable is required")

        agent_mode = "Agents SDK" if self.use_agents_sdk else "Chat Completions (legacy)"
        logger.info(
            f"AgentConfig initialized with model={self.openai_model}, agent_mode={agent_mode}, collection={self.collection_name}"
        )

        # Initialize RetrieverClient for Spec 2 integration (T007)
        self.retriever_client = None
        if RETRIEVER_AVAILABLE:
            try:
                self.retriever_client = self._initialize_retriever_client()
                logger.info("RetrieverClient initialized successfully (Spec 2 integration ready)")
            except Exception as e:
                logger.error(f"Failed to initialize RetrieverClient: {e}")
                raise ValueError(f"Cannot initialize retrieval system: {e}")

    def _initialize_retriever_client(self) -> "RetrieverClient":
        """Initialize RetrieverClient for Qdrant semantic search.

        This reuses the RetrieverClient from Spec 2 to avoid code duplication.
        Configuration comes from environment variables set in __init__.

        Returns:
            RetrieverClient instance for semantic search operations
        """
        if not RETRIEVER_AVAILABLE:
            raise RuntimeError("RetrieverClient not available")

        retriever = RetrieverClient(
            qdrant_url=self.qdrant_url,
            qdrant_api_key=self.qdrant_api_key,
            cohere_api_key=self.cohere_api_key,
            collection_name=self.collection_name,
        )
        logger.info(
            f"RetrieverClient connected to {self.collection_name} collection at {self.qdrant_url}"
        )
        return retriever


# ============================================================================
# Pydantic Schemas
# ============================================================================


class RetrievedChunk(BaseModel):
    """Schema for a chunk retrieved from Qdrant with metadata.

    Maps to chunks retrieved via RetrieverClient.search() from Spec 2.
    """

    chunk_id: str = Field(..., description="Unique identifier for the chunk")
    text: str = Field(..., description="Content of the chunk")
    similarity_score: float = Field(..., ge=0.0, le=1.0, description="Similarity score to query")
    source_url: str = Field(..., description="Source URL of the chunk")
    page_title: str = Field(..., description="Title of the page containing chunk")
    section_headers: List[str] = Field(
        default_factory=list, description="Hierarchy of section headers"
    )

    class Config:
        schema_extra = {
            "example": {
                "chunk_id": "chunk_123",
                "text": "ROS2 is a flexible middleware for robotics...",
                "similarity_score": 0.85,
                "source_url": "https://example.com/chapter1",
                "page_title": "Introduction to ROS2",
                "section_headers": ["Robotics", "ROS2 Basics"],
            }
        }


class ChatRequest(BaseModel):
    """Schema for chat endpoint request.

    Attributes:
    - query: Natural language question (required, 1-10,000 characters)
    - retrieval_scope: "text_only" or "full_collection" (default: "full_collection")
    - top_k: Number of chunks to retrieve (default: 5, range: 1-100)
    - context_text: Optional text snippets for text-only mode
    """

    query: str = Field(..., min_length=1, max_length=10000, description="User question")
    retrieval_scope: Literal["text_only", "full_collection"] = Field(
        default="full_collection",
        description="Retrieval scope: text_only uses provided context, full_collection searches Qdrant",
    )
    top_k: int = Field(default=5, ge=1, le=100, description="Number of chunks to retrieve")
    context_text: Optional[str] = Field(
        default=None, description="Optional text for text_only mode"
    )

    @validator("query")
    def validate_query_not_empty(cls, v):
        """Ensure query is not just whitespace."""
        if not v.strip():
            raise ValueError("Query cannot be empty or whitespace")
        return v.strip()

    class Config:
        schema_extra = {
            "example": {
                "query": "What is ROS2 and how does it differ from ROS?",
                "retrieval_scope": "full_collection",
                "top_k": 5,
                "context_text": None,
            }
        }


class ExecutionMetrics(BaseModel):
    """Metrics for request execution."""

    retrieval_time_ms: float = Field(
        ..., ge=0, description="Time to retrieve chunks (milliseconds)"
    )
    generation_time_ms: float = Field(
        ..., ge=0, description="Time to generate response (milliseconds)"
    )
    total_time_ms: float = Field(..., ge=0, description="Total end-to-end time (milliseconds)")


class ErrorInfo(BaseModel):
    """Error details for error responses."""

    code: str = Field(..., description="Error code (e.g., 'EMPTY_QUERY', 'API_TIMEOUT')")
    message: str = Field(..., description="Human-readable error message")


class ChatResponse(BaseModel):
    """Schema for chat endpoint response.

    Attributes:
    - query: Original user query
    - answer: Agent-generated response
    - retrieved_chunks: List of chunks used as context
    - execution_metrics: Timing information
    - retrieval_scope: Which retrieval mode was used (T039)
    - status: "success" or "error"
    - error: Error details (only if status="error")
    """

    query: str = Field(..., description="Original user query")
    answer: Optional[str] = Field(default=None, description="Agent-generated response")
    retrieved_chunks: List[RetrievedChunk] = Field(
        default_factory=list, description="Context chunks"
    )
    execution_metrics: Optional[ExecutionMetrics] = Field(
        default=None, description="Execution timing"
    )
    retrieval_scope: Optional[Literal["text_only", "full_collection"]] = Field(
        default=None, description="Which retrieval mode was used (T039 - for auditability)"
    )
    status: Literal["success", "error"] = Field(..., description="Request status")
    error: Optional[ErrorInfo] = Field(default=None, description="Error details if status=error")

    class Config:
        schema_extra = {
            "example": {
                "query": "What is ROS2?",
                "answer": "ROS2 is a flexible middleware...",
                "retrieved_chunks": [
                    {
                        "chunk_id": "chunk_123",
                        "text": "ROS2 is a flexible middleware...",
                        "similarity_score": 0.85,
                        "source_url": "https://example.com/chapter1",
                        "page_title": "Introduction to ROS2",
                        "section_headers": ["Robotics", "ROS2 Basics"],
                    }
                ],
                "execution_metrics": {
                    "retrieval_time_ms": 200.0,
                    "generation_time_ms": 2500.0,
                    "total_time_ms": 2800.0,
                },
                "retrieval_scope": "full_collection",
                "status": "success",
                "error": None,
            }
        }


# ============================================================================
# Retrieval Tool for Agents SDK
# ============================================================================


@function_tool
def retrieve_textbook_chunks(query: str, top_k: int = 5) -> List[dict]:
    """
    Retrieve relevant chunks from the robotics textbook based on semantic search.

    This function is designed to be used as a tool by the OpenAI Agents SDK.
    It performs semantic search using Cohere embeddings and Qdrant vector database.

    Args:
        query: Natural language question to search for in the textbook
        top_k: Number of relevant chunks to retrieve (1-10, default: 5)

    Returns:
        List of dictionaries containing:
        - text: Chunk content
        - source_url: URL where chunk came from
        - page_title: Title of the page
        - section_headers: Hierarchy of section headers
        - similarity_score: Similarity score to query (0-1)

    Example:
        >>> chunks = retrieve_textbook_chunks("What is ROS2?", top_k=5)
        >>> print(chunks[0]['text'])
        "ROS2 is a flexible middleware for robotics..."
    """
    # Get global app config with RetrieverClient
    config = get_app_config()

    if not config.retriever_client:
        logger.warning("RetrieverClient not available. Cannot retrieve chunks.")
        return []

    try:
        # Perform semantic search via Qdrant using Cohere embeddings
        search_response = config.retriever_client.search(
            query=query, top_k=min(top_k, 10)  # Cap at 10 chunks maximum
        )

        # Format search results for agent consumption
        chunks = []
        for result in search_response.get("results", []):
            metadata = result.get("metadata", {})
            chunk_dict = {
                "text": result.get("chunk_text", ""),
                "source_url": metadata.get("source_url", ""),
                "page_title": metadata.get("page_title", ""),
                "section_headers": metadata.get("section_headers", []),
                "similarity_score": float(result.get("similarity_score", 0.0)),
            }
            chunks.append(chunk_dict)

        logger.info(f"Retrieved {len(chunks)} chunks for query: '{query[:80]}...'")
        return chunks

    except Exception as e:
        logger.error(f"Error retrieving chunks: {type(e).__name__}: {str(e)}", exc_info=True)
        return []


# ============================================================================
# GroundedAgent - OpenAI Agents SDK Integration
# ============================================================================


class GroundedAgent:
    """Wrapper around OpenAI Agents SDK for RAG with response grounding.

    This class uses the OpenAI Agents SDK to implement a retrieval-augmented
    generation (RAG) system that answers questions about a robotics textbook.

    Architecture:
    - Agent: Uses OpenAI Agents SDK with grounding instructions
    - Tool: retrieve_textbook_chunks function for semantic search
    - Runner: Executes agent with automatic tool orchestration
    - Grounding: System instructions enforce citation of sources only

    The agent automatically:
    1. Receives user question
    2. Calls retrieve_textbook_chunks tool to search textbook
    3. Analyzes retrieved chunks
    4. Generates response grounded strictly in retrieved content
    5. Cites sources for all claims

    Attributes:
        config (AgentConfig): Configuration with OpenAI credentials
        agent (Agent): OpenAI Agents SDK Agent instance
        instructions (str): System instructions for grounding behavior

    Raises:
        ValueError: If OpenAI SDK or Agents SDK unavailable
        RuntimeError: If response generation fails
    """

    def __init__(self, config: AgentConfig):
        """Initialize agent with Agents SDK.

        Args:
            config: AgentConfig instance with OpenAI API key and model

        Raises:
            ValueError: If required SDK is unavailable
        """
        if not OPENAI_AVAILABLE or not AGENTS_SDK_AVAILABLE:
            raise ValueError("OpenAI SDK and Agents SDK required. Cannot initialize GroundedAgent.")

        self.config = config
        self.instructions = self._create_agent_instructions()

        # Create agent with retrieval tool
        try:
            self.agent = Agent(
                name="Robotics Textbook Expert",
                model=third_party_model,
                instructions=self.instructions,
                tools=[retrieve_textbook_chunks],  # Agent can call this tool automatically
            )
            logger.info(f"GroundedAgent initialized with Agents SDK: model={third_party_model}")
        except Exception as e:
            logger.error(f"Failed to initialize Agents SDK Agent: {e}")
            raise ValueError(f"Cannot initialize agent: {e}")

    def _create_agent_instructions(self) -> str:
        """Create instructions that enforce grounding in retrieved content.

        These instructions tell the agent:
        1. To use the retrieve_textbook_chunks tool to find relevant content
        2. To answer ONLY based on retrieved content
        3. To cite sources for all claims
        4. To refuse out-of-scope questions

        Returns:
            Instructions text for the agent
        """
        return """You are an expert assistant answering questions about a robotics textbook.

WORKFLOW:
1. FIRST: Use the retrieve_textbook_chunks tool with the user's question to search for relevant content
2. THEN: Carefully review all retrieved chunks
3. FINALLY: Provide an answer based ONLY on the retrieved content

CRITICAL INSTRUCTIONS:
- Answer ONLY based on retrieved chunks. Do NOT use external knowledge or make assumptions.
- If retrieved content doesn't address the question, respond: "The textbook does not cover this topic."
- Always cite sources by including URLs and page titles from retrieved chunks
- Include relevant quotes or paraphrases from the chunks you retrieve
- If chunks contain conflicting information, acknowledge both versions and cite both sources
- Cite sources using format: "[Source: {source_url} - {page_title}]"
- Keep responses concise and focused on answering the question
- Your responses must be grounded strictly in the textbook content. No hallucinations.

IMPORTANT: Always use the retrieve_textbook_chunks tool before answering any question.
"""

    async def generate_response_async(
        self, query: str, context_chunks: Optional[List[RetrievedChunk]] = None
    ) -> str:
        """Generate response using Agents SDK with automatic retrieval.

        For full-collection mode: Agent autonomously calls retrieve_textbook_chunks tool
        For text-only mode: Pre-formatted context is provided to the agent

        Args:
            query: User's natural language question
            context_chunks: Optional pre-retrieved chunks (for text-only mode only)

        Returns:
            Agent-generated response grounded in retrieved content

        Raises:
            RuntimeError: If agent generation fails
        """
        logger.info(f"Agent invocation: model={self.config.openai_model}, query='{query[:80]}...'")

        generation_start = time.time()

        try:
            if context_chunks:
                # Text-only mode: Format context and provide to agent
                context_str = self._format_context_for_prompt(context_chunks)
                full_query = f"""Use the following provided context to answer the question.

PROVIDED CONTEXT:
{context_str}

QUESTION: {query}

Answer based ONLY on the provided context above.
"""
            else:
                # Full-collection mode: Agent retrieves automatically via tool
                full_query = query

            # Run agent using Agents SDK
            # Runner.run is async, so we await it
            result = await Runner.run(self.agent, full_query)

            generation_time = time.time() - generation_start
            logger.info(f"Agent response generated: time={generation_time:.2f}s")

            # Extract final output from agent result
            return result.final_output

        except Exception as e:
            generation_time = time.time() - generation_start
            logger.error(f"Agent generation failed: {type(e).__name__}: {str(e)}", exc_info=True)
            raise RuntimeError(f"Failed to generate response: {str(e)}") from e

    def generate_response(
        self, query: str, context_chunks: Optional[List[RetrievedChunk]] = None
    ) -> str:
        """Synchronous wrapper for generate_response_async.

        Maintains backward compatibility with existing /chat endpoint code
        that expects a synchronous interface.

        Args:
            query: User's natural language question
            context_chunks: Optional pre-retrieved chunks (for text-only mode)

        Returns:
            Agent-generated response

        Raises:
            RuntimeError: If agent generation fails
        """
        # Run async function in event loop
        return asyncio.run(self.generate_response_async(query, context_chunks))

    def _format_context_for_prompt(self, context_chunks: List[RetrievedChunk]) -> str:
        """Format retrieved chunks as context string for prompt injection.

        Args:
            context_chunks: List of RetrievedChunk objects

        Returns:
            Formatted context string for inclusion in system prompt

        Example output:
            [Chunk 1] (Score: 0.90)
            Source: https://example.com/chapter1 - Introduction to ROS2
            Sections: Robotics > ROS2 Basics
            Text: ROS2 is a flexible middleware...

            [Chunk 2] (Score: 0.85)
            ...
        """
        if not context_chunks:
            return "(No relevant content found in textbook)"

        formatted_chunks = []
        for i, chunk in enumerate(context_chunks, 1):
            section_path = " > ".join(chunk.section_headers) if chunk.section_headers else "General"
            formatted = f"""[Chunk {i}] (Similarity: {chunk.similarity_score:.2f})
Source: {chunk.source_url} - {chunk.page_title}
Sections: {section_path}
Text: {chunk.text}
"""
            formatted_chunks.append(formatted)

        return "\n".join(formatted_chunks)


# ============================================================================
# FastAPI Application
# ============================================================================

# Initialize FastAPI app (T010)
app = FastAPI(
    title="RAG Agent API",
    description="Retrieval-Augmented Generation agent for answering questions about textbook content",
    version="0.1.0",
    docs_url="/docs",
    redoc_url="/redoc",
)

# Configure CORS middleware to allow cross-origin requests (T010)
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # In production, restrict to specific origins
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

logger.info("FastAPI app initialized with CORS middleware configured")

# Initialize config lazily (on first use)
_app_config: Optional[AgentConfig] = None


def get_app_config() -> AgentConfig:
    """Get or initialize application configuration (lazy initialization).

    This allows the config to be initialized either:
    1. On app startup (production with uvicorn)
    2. On first request (testing with TestClient)
    3. Explicitly by the user

    Returns:
        AgentConfig instance with credentials and RetrieverClient

    Raises:
        ValueError: If required environment variables are missing
    """
    global _app_config
    if _app_config is None:
        _app_config = AgentConfig()
        logger.info("[OK] AgentConfig initialized (lazy) with RetrieverClient")
    return _app_config


# ============================================================================
# App Startup/Shutdown Events
# ============================================================================


@app.on_event("startup")
async def startup_event():
    """Initialize app configuration on startup."""
    try:
        config = get_app_config()
        app.state.config = config
        logger.info("[OK] Application startup complete")
    except Exception as e:
        logger.error(f"[ERROR] Application startup failed: {e}")
        raise


@app.on_event("shutdown")
async def shutdown_event():
    """Cleanup on shutdown."""
    logger.info("Application shutting down")


@app.get("/health", tags=["health"])
async def health_check():
    """Health check endpoint."""
    return {
        "status": "healthy",
        "timestamp": datetime.utcnow().isoformat(),
        "service": "rag-agent-api",
    }


@app.post("/chat", response_model=ChatResponse, tags=["chat"])
async def chat(request: ChatRequest, response: Response) -> ChatResponse:
    """Chat endpoint - Query the RAG agent with comprehensive error handling.

    This endpoint implements the complete RAG (Retrieval-Augmented Generation)
    pipeline, orchestrating semantic search, context formatting, and LLM
    invocation with response grounding constraints.

    **RAG Pipeline Flow**:
        1. **Validation** (Phase 3): Pydantic validates ChatRequest schema
           - Query: 1-10,000 characters, non-empty
           - top_k: 1-100 range
           - retrieval_scope: "text_only" or "full_collection"
        2. **Retrieval** (Phase 4): Semantic search for relevant chunks
           - Full-collection: Cohere embedding + Qdrant search
           - Text-only: Process user-provided context
           - Returns up to top_k chunks with metadata
        3. **Context Formatting** (Phase 4): Format chunks for prompt injection
           - Includes similarity scores, source URLs, section hierarchy
        4. **Generation** (Phase 5): OpenAI GPT-4 with grounded system prompt
           - System prompt enforces: answer ONLY from provided context
           - Prevents hallucinations about textbook content
           - Requests citations of sources
        5. **Metrics** (Phase 5): Track latency for SLA monitoring
           - retrieval_time_ms: Cohere embedding + Qdrant search
           - generation_time_ms: OpenAI API call
           - total_time_ms: End-to-end latency (target: <5s)
        6. **Error Handling** (Phase 7): Graceful failure with actionable messages
           - HTTP status codes reflect error severity
           - All responses include retrieval_scope for auditability

    **User Stories Implemented**:
        - US1 (Phase 3): Query via chat endpoint
        - US2 (Phase 4): Retrieve context from Qdrant (or text-only)
        - US3 (Phase 5): Ground responses in book content only
        - US4 (Phase 6): Text-only and full-collection query modes
        - US5 (Phase 7): Comprehensive error handling with proper status codes

    Args:
        request (ChatRequest): User query and retrieval configuration
            - query: Natural language question (required)
            - retrieval_scope: "text_only" or "full_collection" (default: "full_collection")
            - top_k: Number of chunks to retrieve (default: 5, range: 1-100)
            - context_text: User-provided text for text_only mode (optional)
        response (Response): FastAPI response object for dynamic status codes

    Returns:
        ChatResponse: Structured response with answer, context, metrics
            - query: Original user query
            - answer: AI-generated response grounded in context
            - retrieved_chunks: Relevant textbook chunks used as context
            - execution_metrics: Latency breakdown for monitoring
            - retrieval_scope: Which retrieval mode was used
            - status: "success" or "error"
            - error: Error details (only if status="error")

    Raises:
        No exceptions raised; all errors returned in ChatResponse with status="error"

    **HTTP Status Codes**:
        - 200 OK: Request succeeded (check response.status field)
        - 400 Bad Request: Validation error
            * Empty or whitespace query
            * Query exceeds 10,000 characters
            * Invalid top_k (not in 1-100 range)
            * Malformed request JSON
        - 429 Too Many Requests: Rate limit from OpenAI API
            * Client should retry with exponential backoff
        - 500 Internal Server Error: Server-side failure
            * Qdrant connection failure
            * OpenAI API error (other than timeout/rate limit)
            * Unexpected exception
        - 503 Service Unavailable: OpenAI API timeout
            * Generation service timeout (>30 seconds)
            * Client should retry

    **Performance SLA**:
        - Target: <5 seconds end-to-end (p95)
        - Retrieval: <500ms (Cohere embedding + Qdrant search)
        - Generation: <3.5 seconds (OpenAI GPT-4)
        - Overhead: <100ms (validation, formatting)
        See execution_metrics in response for actual timing.

    **Example Request**:
        POST /chat HTTP/1.1
        Content-Type: application/json

        {
            "query": "What is ROS2?",
            "retrieval_scope": "full_collection",
            "top_k": 5,
            "context_text": null
        }

    **Example Success Response**:
        HTTP/1.1 200 OK
        Content-Type: application/json

        {
            "query": "What is ROS2?",
            "answer": "ROS2 is a flexible middleware...",
            "retrieved_chunks": [...],
            "execution_metrics": {
                "retrieval_time_ms": 245.3,
                "generation_time_ms": 2150.7,
                "total_time_ms": 2396.0
            },
            "retrieval_scope": "full_collection",
            "status": "success",
            "error": null
        }

    **Example Error Response**:
        HTTP/1.1 500 Internal Server Error
        Content-Type: application/json

        {
            "query": "What is ROS2?",
            "answer": null,
            "retrieved_chunks": [],
            "execution_metrics": null,
            "retrieval_scope": "full_collection",
            "status": "error",
            "error": {
                "code": "RETRIEVAL_FAILED",
                "message": "Retrieval service unavailable, please retry"
            }
        }
    """
    request_start_time = time.time()

    try:
        # T014: Request validation is handled by Pydantic schema
        logger.info(
            f"Chat request: query='{request.query[:80]}...', "
            f"scope={request.retrieval_scope}, top_k={request.top_k}"
        )

        # ====================================================================
        # T019-T026: Retrieval Integration (User Story 2 - Phase 4)
        # Phase 6: Text-only and Full-Collection Modes (T035-T038)
        # ====================================================================

        retrieved_chunks: List[RetrievedChunk] = []
        retrieval_start_time = time.time()

        # T035: Implement retrieval scope logic based on ChatRequest.retrieval_scope parameter
        if request.retrieval_scope == "full_collection":
            # T037: Implement full-collection mode (default) - Agent autonomously retrieves via tool
            # With Agents SDK: Agent will call retrieve_textbook_chunks tool when needed
            # Endpoint just passes query to agent, skips manual Qdrant search
            logger.info(
                f"Full-collection mode: Agent will autonomously retrieve via retrieve_textbook_chunks tool"
            )
            # retrieved_chunks stays empty - agent will retrieve and handle grounding
            retrieved_chunks = []

        elif request.retrieval_scope == "text_only":
            # T036: Implement text-only mode - process only provided context_text (skip Qdrant)
            logger.info("text_only retrieval scope requested (Phase 6 - User Story 4)")

            # T038: For text-only mode, embed/process provided context_text (don't call Qdrant)
            if not request.context_text:
                logger.warning("text_only mode requested but context_text is empty")
                # Continue with empty context - agent will respond "not in provided context"
                retrieved_chunks = []
            else:
                try:
                    # Use helper function to convert provided text to RetrievedChunk
                    retrieved_chunks = _process_text_only_mode(request.context_text)
                    logger.info(f"Processed text-only mode: {len(retrieved_chunks)} chunk(s)")
                except Exception as e:
                    logger.error(f"Error processing text-only mode: {e}", exc_info=True)
                    return ChatResponse(
                        query=request.query,
                        answer=None,
                        retrieved_chunks=[],
                        execution_metrics=None,
                        retrieval_scope=request.retrieval_scope,
                        status="error",
                        error=ErrorInfo(
                            code="RETRIEVAL_FAILED",
                            message=f"Failed to process provided context: {str(e)}",
                        ),
                    )

        # T024: Handle no-results case
        if not retrieved_chunks:
            logger.warning(f"No relevant chunks found for query: {request.query[:80]}")
            # Continue with empty context - agent will respond "not in textbook"

        retrieval_time_ms = (time.time() - retrieval_start_time) * 1000

        # ====================================================================
        # Phase 5: Agent Invocation and Response Grounding (T027-T034)
        # ====================================================================

        # T015: Wire up agent invocation with GroundedAgent
        # Initialize agent if needed
        agent = None
        agent_answer = None
        generation_time_ms = 0.0

        if OPENAI_AVAILABLE:
            try:
                generation_start_time = time.time()

                # Initialize agent instance
                config = get_app_config()
                agent = GroundedAgent(config)

                # T027-T033: Invoke agent with Agents SDK
                # For full-collection mode: Agent autonomously calls retrieve_textbook_chunks tool
                # For text-only mode: Pre-retrieved chunks are passed to agent
                # Agent uses Agents SDK for automatic tool orchestration and grounding
                agent_answer = await agent.generate_response_async(
                    query=request.query,
                    context_chunks=retrieved_chunks if retrieved_chunks else None,
                )

                generation_time_ms = (time.time() - generation_start_time) * 1000

                # T029: Grounding validation - check if response cites sources
                # (Manual validation in Phase 8, logged for debugging)
                if retrieved_chunks:
                    source_urls = [chunk.source_url for chunk in retrieved_chunks]
                    citations_found = sum(1 for url in source_urls if url in agent_answer)
                    logger.info(
                        f"Grounding validation: {citations_found}/{len(retrieved_chunks)} "
                        f"source URLs found in response (citations)"
                    )

            except Exception as e:
                # T032: Log agent errors with proper HTTP status codes (T047-T048)
                generation_time_ms = (time.time() - generation_start_time) * 1000

                # T047: Handle OpenAI API timeout errors (503 Service Unavailable)
                if OPENAI_AVAILABLE and APITimeoutError and isinstance(e, APITimeoutError):
                    logger.error(f"OpenAI API timeout: {str(e)}", exc_info=True)
                    response.status_code = 503
                    return ChatResponse(
                        query=request.query,
                        answer=None,
                        retrieved_chunks=retrieved_chunks,
                        execution_metrics=None,
                        retrieval_scope=request.retrieval_scope,
                        status="error",
                        error=ErrorInfo(
                            code="OPENAI_TIMEOUT",
                            message="Generation service timeout, please retry",
                        ),
                    )

                # T048: Handle OpenAI rate limit errors (429 Too Many Requests)
                elif OPENAI_AVAILABLE and RateLimitError and isinstance(e, RateLimitError):
                    logger.error(f"OpenAI rate limit exceeded: {str(e)}", exc_info=True)
                    response.status_code = 429
                    return ChatResponse(
                        query=request.query,
                        answer=None,
                        retrieved_chunks=retrieved_chunks,
                        execution_metrics=None,
                        retrieval_scope=request.retrieval_scope,
                        status="error",
                        error=ErrorInfo(
                            code="RATE_LIMITED",
                            message="Rate limited by generation service, please retry after a moment",
                        ),
                    )

                # T046, T047: Handle other OpenAI API errors (500 Internal Server Error)
                elif OPENAI_AVAILABLE and APIConnectionError and isinstance(e, APIConnectionError):
                    logger.error(f"OpenAI API connection error: {str(e)}", exc_info=True)
                    response.status_code = 500
                    return ChatResponse(
                        query=request.query,
                        answer=None,
                        retrieved_chunks=retrieved_chunks,
                        execution_metrics=None,
                        retrieval_scope=request.retrieval_scope,
                        status="error",
                        error=ErrorInfo(
                            code="GENERATION_FAILED",
                            message="Generation service unavailable, please retry",
                        ),
                    )

                # T053: Generic agent error - log with full details
                else:
                    logger.error(
                        f"Agent generation failed: {type(e).__name__}: {str(e)}", exc_info=True
                    )
                    response.status_code = 500
                    return ChatResponse(
                        query=request.query,
                        answer=None,
                        retrieved_chunks=retrieved_chunks,
                        execution_metrics=None,
                        retrieval_scope=request.retrieval_scope,
                        status="error",
                        error=ErrorInfo(
                            code="AGENT_FAILED",
                            message=f"Agent response generation failed: {str(e)}",
                        ),
                    )
        else:
            # OpenAI SDK not available
            logger.error("OpenAI SDK not available for agent invocation")
            return ChatResponse(
                query=request.query,
                answer=None,
                retrieved_chunks=retrieved_chunks,
                execution_metrics=None,
                retrieval_scope=request.retrieval_scope,
                status="error",
                error=ErrorInfo(
                    code="OPENAI_UNAVAILABLE",
                    message="OpenAI SDK not available. Install with: pip install openai",
                ),
            )

        # T016: Format and return ChatResponse with execution metrics
        # T033-T034: Include generation time and verify <5s SLA
        total_time_ms = (time.time() - request_start_time) * 1000

        # T039: Update ChatResponse to indicate which retrieval_scope was used
        response = ChatResponse(
            query=request.query,
            answer=agent_answer,
            retrieved_chunks=retrieved_chunks,
            execution_metrics=ExecutionMetrics(
                retrieval_time_ms=retrieval_time_ms,
                generation_time_ms=generation_time_ms,
                total_time_ms=total_time_ms,
            ),
            retrieval_scope=request.retrieval_scope,
            status="success",
            error=None,
        )

        # T034: Log latency and check SLA
        latency_ok = "[OK]" if total_time_ms < 5000 else "[WARN]"
        logger.info(
            f"Chat response sent: status={response.status}, "
            f"chunks={len(retrieved_chunks)}, "
            f"retrieval_ms={retrieval_time_ms:.1f}, "
            f"generation_ms={generation_time_ms:.1f}, "
            f"total_ms={total_time_ms:.1f} {latency_ok}"
        )

        # Warn if SLA exceeded
        if total_time_ms >= 5000:
            logger.warning(
                f"[WARN] SLA Warning: Response time {total_time_ms:.0f}ms exceeds 5000ms target "
                f"(retrieval: {retrieval_time_ms:.0f}ms, generation: {generation_time_ms:.0f}ms)"
            )

        return response

    except ValueError as e:
        # T043-T045, T049: Handle validation errors -> 400 Bad Request
        # Includes: empty query, invalid top_k, query too long, malformed request, etc.
        logger.error(f"Validation error in chat endpoint: {e}")  # T053: Logging
        response.status_code = 400  # T051: Validation errors → 400 Bad Request
        return ChatResponse(
            query=request.query if request else "",
            answer=None,
            retrieved_chunks=[],
            execution_metrics=None,
            retrieval_scope=request.retrieval_scope if request else None,
            status="error",
            error=ErrorInfo(code="VALIDATION_ERROR", message=str(e)),
        )

    except Exception as e:
        # T053: Handle unexpected errors -> 500 Internal Server Error
        logger.error(
            f"Unexpected error in chat endpoint: {type(e).__name__}: {str(e)}", exc_info=True
        )
        response.status_code = 500  # T051: Unexpected errors → 500 Internal Server Error
        return ChatResponse(
            query=request.query if request else "",
            answer=None,
            retrieved_chunks=[],
            execution_metrics=None,
            retrieval_scope=request.retrieval_scope if request else None,
            status="error",
            error=ErrorInfo(code="INTERNAL_ERROR", message="Internal server error"),
        )


def _process_text_only_mode(context_text: Optional[str]) -> List[RetrievedChunk]:
    """Process text-only mode context (T036, T038).

    When retrieval_scope="text_only", the user provides context_text directly.
    This function converts the provided text into RetrievedChunk objects.

    Args:
        context_text: Text provided by user for text-only mode

    Returns:
        List of RetrievedChunk objects (single chunk from provided text)

    Note: For text-only mode, we treat the entire provided text as a single chunk
    with no semantic search required. The user has already selected their context.
    """
    if not context_text or not context_text.strip():
        logger.warning("text_only mode requested but context_text is empty")
        return []

    logger.info(f"Processing text-only mode with {len(context_text)} chars of provided context")

    # Create a single RetrievedChunk from the provided text (T038)
    chunk = RetrievedChunk(
        chunk_id="user_provided_1",
        text=context_text.strip(),
        similarity_score=1.0,  # User-provided content is 100% relevant
        source_url="<user_provided>",
        page_title="User-Provided Context",
        section_headers=["User Input"],
    )

    return [chunk]


def _format_context_for_prompt(chunks: List[RetrievedChunk]) -> str:
    """Format retrieved chunks as context string for prompt injection (T023).

    Args:
        chunks: List of RetrievedChunk objects from Qdrant

    Returns:
        Formatted context string for inclusion in agent system prompt

    Example output:
        [Chunk 1] (Score: 0.90)
        Source: https://example.com/chapter1 - Introduction to ROS2
        Sections: Robotics > ROS2 Basics
        Text: ROS2 is a flexible middleware...

        [Chunk 2] (Score: 0.85)
        ...
    """
    if not chunks:
        return "(No relevant content found in textbook)"

    formatted_chunks = []
    for i, chunk in enumerate(chunks, 1):
        section_path = " > ".join(chunk.section_headers) if chunk.section_headers else "General"
        formatted = f"""[Chunk {i}] (Similarity: {chunk.similarity_score:.2f})
Source: {chunk.source_url} - {chunk.page_title}
Sections: {section_path}
Text: {chunk.text}
"""
        formatted_chunks.append(formatted)

    return "\n".join(formatted_chunks)


@app.exception_handler(ValueError)
async def value_error_handler(request_obj, exc):
    """Handle validation errors (T043-T049, T053)."""
    logger.error(f"Validation error: {exc}")
    return ChatResponse(
        query="",
        answer=None,
        retrieved_chunks=[],
        execution_metrics=None,
        retrieval_scope=None,
        status="error",
        error=ErrorInfo(code="VALIDATION_ERROR", message=str(exc)),
    )


@app.exception_handler(Exception)
async def general_exception_handler(request_obj, exc):
    """Handle unexpected exceptions with logging (T053)."""
    logger.error(f"Unhandled exception: {type(exc).__name__}: {str(exc)}", exc_info=True)
    return ChatResponse(
        query="",
        answer=None,
        retrieved_chunks=[],
        execution_metrics=None,
        retrieval_scope=None,
        status="error",
        error=ErrorInfo(code="INTERNAL_ERROR", message="Internal server error"),
    )


if __name__ == "__main__":
    import uvicorn

    try:
        config = AgentConfig()
        logger.info("AgentConfig loaded successfully")
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        exit(1)

    # Run: uvicorn backend.agent:app --reload --host 0.0.0.0 --port 8000
    uvicorn.run(app, host="0.0.0.0", port=8000, log_level=config.log_level.lower())
