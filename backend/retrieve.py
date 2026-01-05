"""Retrieval pipeline for validating RAG embeddings and semantic search.

This module provides a client for querying the Qdrant vector database with
natural language queries, using Cohere embeddings to find relevant textbook content.
"""

import argparse
import json
import logging
import sys
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import cohere
from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams

from ingestion.config import Config


def setup_logging(level: str = "INFO", log_file: str = "retrieval_validation.log") -> logging.Logger:
    """Configure logging with file and console output.

    Args:
        level: Logging level as string (DEBUG, INFO, WARNING, ERROR)
        log_file: Path to log file (relative to backend dir)

    Returns:
        Configured logger instance
    """
    numeric_level = getattr(logging, level.upper(), logging.INFO)

    # Create log file path
    log_path = Path(__file__).parent / log_file

    # Configure logging
    logging.basicConfig(
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        level=numeric_level,
        force=True,
        handlers=[
            logging.FileHandler(log_path),
            logging.StreamHandler(sys.stdout)
        ]
    )

    logger = logging.getLogger(__name__)
    logger.info(f"Logging initialized at level {level} to {log_path}")
    return logger


logger = setup_logging()


class QueryEmbedder:
    """Embed search queries using Cohere API."""

    def __init__(
        self,
        api_key: str,
        model: str = "embed-english-v3.0",
        max_retries: int = 3,
        timeout: int = 60
    ):
        """Initialize query embedder.

        Args:
            api_key: Cohere API key
            model: Embedding model name
            max_retries: Max retry attempts for API calls
            timeout: Request timeout in seconds
        """
        self.client = cohere.ClientV2(api_key=api_key)
        self.model = model
        self.max_retries = max_retries
        self.timeout = timeout
        logger.info(f"QueryEmbedder initialized with model: {model}")

    def embed_query(self, query: str) -> List[float]:
        """Embed a search query using Cohere.

        Args:
            query: Natural language query text

        Returns:
            Embedding vector (1024-dimensional for embed-english-v3.0)

        Raises:
            ValueError: If query is empty
            RuntimeError: If Cohere API fails after retries
        """
        if not query or not query.strip():
            raise ValueError("Query cannot be empty")

        for attempt in range(self.max_retries):
            try:
                logger.debug(f"Embedding query (attempt {attempt + 1}): {query[:100]}...")

                response = self.client.embed(
                    model=self.model,
                    texts=[query],
                    input_type="search_query",
                    embedding_types=["float"],
                    truncate="END"
                )

                embedding = response.embeddings.float[0]
                logger.info(f"Query embedded successfully: {len(embedding)} dimensions")
                return embedding

            except Exception as e:
                logger.warning(f"Attempt {attempt + 1} failed: {e}")
                if attempt == self.max_retries - 1:
                    logger.error(f"Failed to embed query after {self.max_retries} attempts")
                    raise RuntimeError(f"Failed to embed query: {e}")
                time.sleep(2 ** attempt)  # Exponential backoff

        raise RuntimeError("Unexpected error in embed_query")


class RetrieverClient:
    """Query Qdrant vector database for relevant content chunks."""

    def __init__(
        self,
        qdrant_url: str,
        qdrant_api_key: str,
        cohere_api_key: str,
        collection_name: str = "textbook_embeddings",
        model: str = "embed-english-v3.0"
    ):
        """Initialize retriever client.

        Args:
            qdrant_url: Qdrant Cloud URL
            qdrant_api_key: Qdrant API key
            cohere_api_key: Cohere API key
            collection_name: Qdrant collection name
            model: Embedding model to use
        """
        self.qdrant_client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)
        self.embedder = QueryEmbedder(cohere_api_key, model=model)
        self.collection_name = collection_name
        self.model = model

        logger.info(f"RetrieverClient initialized for collection: {collection_name}")

    def _validate_config(self) -> None:
        """Validate configuration and Qdrant connectivity.

        Raises:
            RuntimeError: If collection doesn't exist or API keys are invalid
        """
        try:
            if not self.qdrant_client.collection_exists(self.collection_name):
                raise RuntimeError(f"Collection '{self.collection_name}' not found in Qdrant")

            collection_info = self.qdrant_client.get_collection(self.collection_name)
            logger.info(
                f"Collection validated: {collection_info.points_count} points, "
                f"vector size: {collection_info.config.params.vectors.size}"
            )
        except Exception as e:
            logger.error(f"Configuration validation failed: {e}")
            raise

    def search(
        self,
        query: str,
        top_k: int = 5,
        similarity_threshold: float = 0.0
    ) -> Dict[str, Any]:
        """Search for relevant chunks using natural language query.

        Args:
            query: Natural language search query
            top_k: Number of results to return (1-100)
            similarity_threshold: Minimum similarity score (0.0-1.0)

        Returns:
            Dictionary with search results and metadata

        Raises:
            ValueError: If parameters are invalid
            RuntimeError: If search fails
        """
        if not 1 <= top_k <= 100:
            raise ValueError("top_k must be between 1 and 100")
        if not 0.0 <= similarity_threshold <= 1.0:
            raise ValueError("similarity_threshold must be between 0.0 and 1.0")

        try:
            start_time = time.time()

            # Step 1: Embed the query
            logger.info(f"Searching for: {query}")
            embedding_start = time.time()
            query_embedding = self.embedder.embed_query(query)
            embedding_time = time.time() - embedding_start

            # Step 2: Search Qdrant collection
            search_start = time.time()
            search_results = self.qdrant_client.query_points(
                collection_name=self.collection_name,
                query=query_embedding,
                limit=top_k,
                with_payload=True
            )
            search_time = time.time() - search_start

            # Step 3: Format results
            results = []
            rank = 0
            for point in search_results.points:
                if point.score >= similarity_threshold:
                    rank += 1
                    result = {
                        "chunk_id": str(point.id),
                        "chunk_text": point.payload.get("chunk_text", ""),
                        "similarity_score": point.score,
                        "rank": rank,
                        "metadata": {
                            "source_url": point.payload.get("source_url", ""),
                            "page_title": point.payload.get("page_title", ""),
                            "section_headers": point.payload.get("section_headers", []),
                            "chunk_index": point.payload.get("chunk_index", -1)
                        }
                    }
                    results.append(result)
                    logger.debug(f"Result {rank}: score={point.score:.3f}, url={result['metadata']['source_url']}")

            total_time = time.time() - start_time

            response = {
                "status": "success",
                "query": {"text": query},
                "results": results,
                "total_results": len(results),
                "requested_top_k": top_k,
                "execution_metrics": {
                    "query_embedding_time_ms": round(embedding_time * 1000, 2),
                    "vector_search_time_ms": round(search_time * 1000, 2),
                    "total_execution_time_ms": round(total_time * 1000, 2),
                    "embedding_model": self.model,
                    "collection_name": self.collection_name
                }
            }

            logger.info(
                f"Search complete: {len(results)} results in {total_time:.3f}s "
                f"(embed: {embedding_time:.3f}s, search: {search_time:.3f}s)"
            )
            return response

        except Exception as e:
            logger.error(f"Search failed: {e}")
            return {
                "status": "error",
                "query": {"text": query},
                "results": [],
                "total_results": 0,
                "requested_top_k": top_k,
                "execution_metrics": {
                    "total_execution_time_ms": round((time.time() - start_time) * 1000, 2),
                    "embedding_model": self.model,
                    "collection_name": self.collection_name
                },
                "error": {
                    "code": "SEARCH_FAILED",
                    "message": str(e)
                }
            }

    def validate_metadata(self, results: List[Dict[str, Any]]) -> Dict[str, Any]:
        """Validate metadata integrity in search results.

        Args:
            results: List of search results

        Returns:
            Validation report
        """
        report = {
            "total_results": len(results),
            "valid_results": 0,
            "invalid_results": 0,
            "issues": []
        }

        required_metadata_fields = ["source_url", "page_title", "section_headers", "chunk_index"]

        for idx, result in enumerate(results):
            metadata = result.get("metadata", {})
            has_issues = False

            for field in required_metadata_fields:
                if field not in metadata:
                    report["issues"].append(f"Result {idx}: missing field {field}")
                    has_issues = True
                elif field == "chunk_index":
                    # chunk_index can be 0, so check explicitly for None
                    if metadata[field] is None:
                        report["issues"].append(f"Result {idx}: empty {field}")
                        has_issues = True
                elif not metadata[field]:
                    # For other fields, empty/falsy is a problem
                    report["issues"].append(f"Result {idx}: empty {field}")
                    has_issues = True

            if has_issues:
                report["invalid_results"] += 1
            else:
                report["valid_results"] += 1

        logger.info(f"Metadata validation: {report['valid_results']}/{report['total_results']} valid")
        return report


class ValidationRunner:
    """Execute retrieval validation with sample queries."""

    def __init__(self, config: Optional[Config] = None):
        """Initialize validation runner.

        Args:
            config: Optional Config object; loads from environment if not provided
        """
        self.config = config or Config()
        self.config.validate()

        self.client = RetrieverClient(
            qdrant_url=self.config.qdrant_url,
            qdrant_api_key=self.config.qdrant_api_key,
            cohere_api_key=self.config.cohere_api_key,
            collection_name=self.config.collection_name,
            model=self.config.cohere_model
        )
        self.client._validate_config()

        logger.info("ValidationRunner initialized and configuration validated")

    def validate_queries(
        self,
        queries: Optional[List[str]] = None,
        top_k: int = 5
    ) -> List[Dict[str, Any]]:
        """Run validation queries and collect results.

        Args:
            queries: List of queries to validate (uses defaults if None)
            top_k: Number of results per query

        Returns:
            List of search responses
        """
        if queries is None:
            queries = [
                "What is ROS2?",
                "Explain simulation in Gazebo",
                "How do I use Isaac Sim?",
                "What is reinforcement learning?"
            ]

        logger.info(f"Running validation with {len(queries)} queries, top_k={top_k}")

        responses = []
        for query in queries:
            logger.info(f"\n{'='*60}")
            logger.info(f"Query: {query}")
            response = self.client.search(query, top_k=top_k)
            responses.append(response)

            # Print results
            if response["status"] == "success":
                logger.info(f"Found {response['total_results']} results")
                for result in response["results"]:
                    logger.info(
                        f"  [{result['rank']}] Score: {result['similarity_score']:.3f} | "
                        f"{result['metadata']['page_title']}"
                    )
            else:
                logger.error(f"Error: {response['error']['message']}")

        logger.info(f"\n{'='*60}")
        logger.info("Validation complete")
        return responses


def main():
    """CLI entry point for retrieval validation."""
    parser = argparse.ArgumentParser(description="Validate RAG retrieval pipeline")
    parser.add_argument(
        "--validate",
        action="store_true",
        help="Run validation with sample queries"
    )
    parser.add_argument(
        "--query",
        type=str,
        help="Run a single custom query"
    )
    parser.add_argument(
        "--top-k",
        type=int,
        default=5,
        help="Number of results to return (1-100)"
    )
    parser.add_argument(
        "--log-level",
        type=str,
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level"
    )
    parser.add_argument(
        "--output",
        type=str,
        help="Save results to JSON file"
    )

    args = parser.parse_args()

    # Reconfigure logging if needed
    if args.log_level != "INFO":
        logger.setLevel(getattr(logging, args.log_level))

    try:
        if args.validate:
            runner = ValidationRunner()
            responses = runner.validate_queries(top_k=args.top_k)

            if args.output:
                with open(args.output, "w") as f:
                    json.dump(responses, f, indent=2)
                logger.info(f"Results saved to {args.output}")

        elif args.query:
            runner = ValidationRunner()
            response = runner.client.search(args.query, top_k=args.top_k)

            if response["status"] == "success":
                print(json.dumps(response, indent=2))
            else:
                print(f"Error: {response['error']['message']}")
                sys.exit(1)

            if args.output:
                with open(args.output, "w") as f:
                    json.dump(response, f, indent=2)

        else:
            parser.print_help()

    except Exception as e:
        logger.error(f"Fatal error: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
