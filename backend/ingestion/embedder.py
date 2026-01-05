"""Cohere embedding generation with batching and retry logic."""

import logging
import time
from typing import List

import cohere


logger = logging.getLogger(__name__)


class CohereEmbedder:
    """Generate embeddings using Cohere API with error handling."""

    def __init__(
        self,
        api_key: str,
        model: str = "embed-english-v3.0",
        batch_size: int = 96,
        max_retries: int = 3,
        timeout: int = 60
    ):
        """Initialize embedder.

        Args:
            api_key: Cohere API key
            model: Embedding model name
            batch_size: Texts per request (max 96)
            max_retries: Max retry attempts
            timeout: Request timeout in seconds
        """
        self.client = cohere.ClientV2(api_key=api_key)
        self.model = model
        self.batch_size = min(batch_size, 96)
        self.max_retries = max_retries
        self.timeout = timeout

        self.stats = {
            "total_chunks": 0,
            "successful": 0,
            "failed": 0,
            "failed_chunks": []
        }

    def embed_chunks(
        self,
        chunks: List[str],
        input_type: str = "search_document"
    ) -> List[List[float]]:
        """Generate embeddings for chunks with batching and retry.

        Args:
            chunks: List of text chunks
            input_type: "search_document" or "search_query"

        Returns:
            List of embedding vectors
        """
        self.stats["total_chunks"] = len(chunks)
        all_embeddings = []

        for batch_idx in range(0, len(chunks), self.batch_size):
            batch = chunks[batch_idx:batch_idx + self.batch_size]

            for attempt in range(self.max_retries):
                try:
                    response = self.client.embed(
                        model=self.model,
                        texts=batch,
                        input_type=input_type,
                        embedding_types=["float"],
                        truncate="END"
                    )

                    all_embeddings.extend(response.embeddings.float)
                    self.stats["successful"] += len(batch)
                    logger.info(f"Batch {batch_idx // self.batch_size + 1}: {len(batch)} chunks embedded")
                    break  # Success - exit retry loop

                except cohere.errors.TooManyRequestsError:
                    wait = 2 ** attempt
                    logger.warning(f"Rate limit on batch {batch_idx // self.batch_size + 1}. Retry in {wait}s (attempt {attempt + 1}/{self.max_retries})")
                    time.sleep(wait)

                except Exception as e:
                    if attempt == self.max_retries - 1:
                        logger.error(f"Batch failed after {self.max_retries} retries: {e}")
                        self.stats["failed"] += len(batch)
                        self.stats["failed_chunks"].extend(batch[:5])  # Log first 5
                        break
                    wait = 2 ** attempt
                    logger.error(f"Embedding error (attempt {attempt + 1}): {e}. Retrying in {wait}s")
                    time.sleep(wait)

        return all_embeddings

    def get_stats(self) -> dict:
        """Get embedding statistics.

        Returns:
            Stats dict with totals and failure info
        """
        return {
            "total_chunks": self.stats["total_chunks"],
            "successful": self.stats["successful"],
            "failed": self.stats["failed"],
            "success_rate": (
                self.stats["successful"] / self.stats["total_chunks"]
                if self.stats["total_chunks"] > 0
                else 0
            )
        }
