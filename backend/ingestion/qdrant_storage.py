"""Qdrant Cloud vector storage management."""

import hashlib
import logging
import uuid
from typing import List, Dict

from qdrant_client import QdrantClient
from qdrant_client.models import Distance, VectorParams, PointStruct, Filter, FieldCondition, MatchValue


logger = logging.getLogger(__name__)


class QdrantManager:
    """Manage Qdrant Cloud collections and vector insertion."""

    def __init__(self, url: str, api_key: str, collection_name: str, vector_size: int = 1024):
        """Initialize Qdrant manager.

        Args:
            url: Qdrant Cloud URL
            api_key: Qdrant API key
            collection_name: Collection name
            vector_size: Vector dimensionality (default: 1024 for Cohere v3.0)
        """
        self.client = QdrantClient(url=url, api_key=api_key)
        self.collection_name = collection_name
        self.vector_size = vector_size

    def initialize_collection(self) -> None:
        """Create collection if it doesn't exist."""
        try:
            if not self.client.collection_exists(self.collection_name):
                self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(
                        size=self.vector_size,
                        distance=Distance.COSINE
                    )
                )
                logger.info(f"Created collection: {self.collection_name}")
            else:
                logger.info(f"Collection exists: {self.collection_name}")

            # Create payload indexes
            self._create_indexes()
        except Exception as e:
            logger.error(f"Failed to initialize collection: {e}")
            raise

    def _create_indexes(self) -> None:
        """Create payload indexes for efficient filtering."""
        try:
            # Index for URL filtering
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="source_url",
                field_schema="keyword",
            )
        except Exception:
            pass  # Index may already exist

        try:
            # Index for content hash (deduplication)
            self.client.create_payload_index(
                collection_name=self.collection_name,
                field_name="chunk_id",
                field_schema="keyword",
            )
        except Exception:
            pass

    def generate_point_id(self, text: str, url: str) -> str:
        """Generate deterministic UUID from content hash.

        Args:
            text: Chunk text
            url: Source URL

        Returns:
            UUID string based on content hash
        """
        content = f"{url}:{text}"
        hash_hex = hashlib.sha256(content.encode('utf-8')).hexdigest()
        return str(uuid.UUID(hash_hex[:32]))

    def upsert_embeddings(
        self,
        chunks: List[Dict],
        embeddings: List[List[float]]
    ) -> int:
        """Upsert embeddings and metadata to Qdrant.

        Args:
            chunks: List of chunk dicts with text and metadata
            embeddings: List of embedding vectors

        Returns:
            Number of points inserted
        """
        points = []

        for chunk, embedding in zip(chunks, embeddings):
            # Generate deterministic point ID from content
            point_id = self.generate_point_id(chunk["text"], chunk["metadata"]["source_url"])

            points.append(
                PointStruct(
                    id=point_id,
                    vector=embedding,
                    payload={
                        "source_url": chunk["metadata"]["source_url"],
                        "page_title": chunk["metadata"]["page_title"],
                        "section_headers": chunk["metadata"]["section_headers"],
                        "chunk_id": chunk["metadata"]["chunk_id"],
                        "chunk_index": chunk["metadata"]["chunk_index"],
                        "chunk_text": chunk["text"],
                        "timestamp": chunk["metadata"]["timestamp"],
                        "chunk_size_tokens": chunk["metadata"]["chunk_size_tokens"]
                    }
                )
            )

        try:
            self.client.upsert(
                collection_name=self.collection_name,
                points=points,
                wait=True
            )
            logger.info(f"Inserted {len(points)} points to {self.collection_name}")
            return len(points)
        except Exception as e:
            logger.error(f"Failed to insert points: {e}")
            raise

    def get_collection_stats(self) -> Dict:
        """Get collection statistics.

        Returns:
            Collection stats dict
        """
        try:
            info = self.client.get_collection(self.collection_name)
            return {
                "name": self.collection_name,
                "points_count": info.points_count,
                "indexed_vectors_count": info.indexed_vectors_count,
                "status": str(info.status),
            }
        except Exception as e:
            logger.error(f"Failed to get collection stats: {e}")
            raise

    def verify_url_chunks(self, url: str) -> int:
        """Count chunks for a specific URL.

        Args:
            url: Source URL

        Returns:
            Number of chunks for that URL
        """
        try:
            results, _ = self.client.scroll(
                collection_name=self.collection_name,
                scroll_filter=Filter(
                    must=[
                        FieldCondition(
                            key="source_url",
                            match=MatchValue(value=url)
                        )
                    ]
                ),
                limit=10000
            )
            return len(results)
        except Exception as e:
            logger.error(f"Failed to verify URL chunks: {e}")
            return 0

    def similarity_search(
        self,
        query_vector: List[float],
        limit: int = 5
    ) -> List[Dict]:
        """Perform similarity search.

        Args:
            query_vector: Query embedding
            limit: Number of results

        Returns:
            List of similar chunks with metadata
        """
        try:
            results = self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=limit,
                with_payload=True
            )

            return [
                {
                    "score": point.score,
                    "url": point.payload.get("source_url"),
                    "title": point.payload.get("page_title"),
                    "text": point.payload.get("chunk_text", "")[:200]
                }
                for point in results.points
            ]
        except Exception as e:
            logger.error(f"Similarity search failed: {e}")
            return []
