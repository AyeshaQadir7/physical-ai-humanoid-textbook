"""Main orchestrator for ingestion pipeline."""

import argparse
import json
import logging
import sys
from datetime import datetime
from pathlib import Path
from typing import Dict, Any

from .config import Config
from .crawler import URLCrawler
from .chunker import TextChunker
from .embedder import CohereEmbedder
from .qdrant_storage import QdrantManager
from .checkpoint import CheckpointManager


def setup_logging(level: str = "INFO") -> logging.Logger:
    """Configure logging with specified level.

    Args:
        level: Logging level as string (DEBUG, INFO, WARNING, ERROR)

    Returns:
        Configured logger instance
    """
    numeric_level = getattr(logging, level.upper(), logging.INFO)
    logging.basicConfig(
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
        level=numeric_level,
        force=True  # Override any previous configuration
    )
    return logging.getLogger(__name__)


logger = setup_logging()


class IngestionPipeline:
    """Complete ingestion pipeline orchestrator."""

    def __init__(self, config: Config):
        """Initialize pipeline with configuration.

        Args:
            config: Configuration object
        """
        self.config = config
        self.crawler = URLCrawler(config.textbook_base_url)
        self.chunker = TextChunker(config.chunk_size, config.chunk_overlap)
        self.embedder = CohereEmbedder(
            config.cohere_api_key,
            model=config.cohere_model,
            batch_size=config.batch_size,
            max_retries=config.max_retries
        )
        self.storage = QdrantManager(
            config.qdrant_url,
            config.qdrant_api_key,
            config.collection_name
        )
        self.checkpoint = CheckpointManager(config.checkpoint_file)

    def run(self, clear_checkpoint: bool = False) -> Dict[str, Any]:
        """Execute complete ingestion pipeline.

        Args:
            clear_checkpoint: Whether to clear checkpoint for fresh run

        Returns:
            Final ingestion report dictionary with status, stats, errors
        """
        start_time = datetime.utcnow()

        if clear_checkpoint:
            self.checkpoint.clear()

        # Stage 1: Crawl
        logger.info("=" * 60)
        logger.info("STAGE 1: Crawling URLs")
        logger.info("=" * 60)

        pages = self.crawler.crawl()
        crawl_stats = self.crawler.get_stats()

        # Stage 2: Chunk
        logger.info("\n" + "=" * 60)
        logger.info("STAGE 2: Chunking content")
        logger.info("=" * 60)

        all_chunks = []
        for page in pages:
            chunks = self.chunker.chunk_text(
                page["extracted_text"],
                page["url"],
                page["page_title"],
                page["section_headers"]
            )
            all_chunks.extend(chunks)

        chunk_validation = self.chunker.validate_chunk_sizes(all_chunks)
        logger.info(f"Chunk validation: {chunk_validation}")

        # Stage 3: Filter processed chunks (resume capability)
        logger.info("\n" + "=" * 60)
        logger.info("STAGE 3: Filtering processed chunks")
        logger.info("=" * 60)

        new_chunks = [
            c for c in all_chunks
            if not self.checkpoint.is_processed(c["hash"])
        ]
        logger.info(f"New chunks: {len(new_chunks)} (skipped: {len(all_chunks) - len(new_chunks)})")

        # Stage 4: Embed
        logger.info("\n" + "=" * 60)
        logger.info("STAGE 4: Generating embeddings")
        logger.info("=" * 60)

        if new_chunks:
            chunk_texts = [c["text"] for c in new_chunks]
            embeddings = self.embedder.embed_chunks(chunk_texts)
            embed_stats = self.embedder.get_stats()
            logger.info(f"Embedding stats: {embed_stats}")
        else:
            embeddings = []
            embed_stats = {"total_chunks": 0, "successful": 0, "failed": 0, "success_rate": 1.0}

        # Stage 5: Initialize Qdrant collection
        logger.info("\n" + "=" * 60)
        logger.info("STAGE 5: Initializing Qdrant collection")
        logger.info("=" * 60)

        self.storage.initialize_collection()

        # Stage 6: Insert to Qdrant
        logger.info("\n" + "=" * 60)
        logger.info("STAGE 6: Inserting embeddings to Qdrant")
        logger.info("=" * 60)

        insertion_count = 0
        if new_chunks and embeddings:
            insertion_count = self.storage.upsert_embeddings(new_chunks, embeddings)

            # Mark chunks as processed
            for chunk in new_chunks:
                self.checkpoint.mark_processed(chunk["hash"])

        # Stage 7: Verify
        logger.info("\n" + "=" * 60)
        logger.info("STAGE 7: Verifying data integrity")
        logger.info("=" * 60)

        collection_stats = self.storage.get_collection_stats()
        logger.info(f"Collection stats: {collection_stats}")

        # Build report
        end_time = datetime.utcnow()
        duration = (end_time - start_time).total_seconds()

        report = {
            "run_id": start_time.isoformat() + "Z",
            "status": "success",
            "summary": {
                "urls_crawled": crawl_stats["visited_urls"],
                "urls_failed": crawl_stats["failed_urls"],
                "total_chunks_created": len(all_chunks),
                "new_chunks": len(new_chunks),
                "total_embeddings_generated": embed_stats["successful"],
                "total_points_inserted": insertion_count,
                "insertion_success_rate": (
                    insertion_count / len(new_chunks)
                    if len(new_chunks) > 0
                    else 1.0
                )
            },
            "timings": {
                "total_duration_seconds": round(duration, 2)
            },
            "verification": {
                "vector_count": collection_stats.get("points_count", 0),
                "collection_status": collection_stats.get("status")
            },
            "errors": crawl_stats["failed_details"]
        }

        logger.info("\n" + "=" * 60)
        logger.info("INGESTION COMPLETE")
        logger.info("=" * 60)
        logger.info(json.dumps(report, indent=2))

        return report


def main():
    """CLI entry point."""
    parser = argparse.ArgumentParser(description="Website ingestion pipeline")
    parser.add_argument(
        "--base-url",
        default=None,
        help="Base URL to crawl (overrides TEXTBOOK_BASE_URL env var)"
    )
    parser.add_argument(
        "--collection-name",
        default=None,
        help="Qdrant collection name (overrides COLLECTION_NAME env var)"
    )
    parser.add_argument(
        "--clear-checkpoint",
        action="store_true",
        help="Clear checkpoint for fresh run"
    )
    parser.add_argument(
        "--log-level",
        default="INFO",
        choices=["DEBUG", "INFO", "WARNING", "ERROR"],
        help="Logging level"
    )

    args = parser.parse_args()

    # Set log level early
    global logger
    logger = setup_logging(args.log_level)

    # Load configuration
    try:
        config = Config()

        # Override with CLI args
        if args.base_url:
            config.textbook_base_url = args.base_url
        if args.collection_name:
            config.collection_name = args.collection_name

        config.validate()
    except ValueError as e:
        logger.error(f"Configuration error: {e}")
        sys.exit(1)

    # Run pipeline
    try:
        pipeline = IngestionPipeline(config)
        report = pipeline.run(clear_checkpoint=args.clear_checkpoint)

        # Exit with success
        sys.exit(0)
    except Exception as e:
        logger.error(f"Pipeline failed: {e}", exc_info=True)
        sys.exit(1)


if __name__ == "__main__":
    main()
