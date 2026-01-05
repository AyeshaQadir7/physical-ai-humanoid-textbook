"""Configuration loading from environment variables."""

import os
from dataclasses import dataclass
from dotenv import load_dotenv
from typing import Optional


load_dotenv()


@dataclass
class Config:
    """Application configuration from environment variables."""

    # Cohere API
    cohere_api_key: str = os.getenv("COHERE_API_KEY", "")
    cohere_model: str = os.getenv("COHERE_MODEL", "embed-english-v3.0")
    batch_size: int = int(os.getenv("BATCH_SIZE", "96"))
    max_retries: int = int(os.getenv("MAX_RETRIES", "3"))
    request_timeout: int = int(os.getenv("REQUEST_TIMEOUT", "60"))

    # Qdrant Cloud
    qdrant_url: str = os.getenv("QDRANT_URL", "")
    qdrant_api_key: str = os.getenv("QDRANT_API_KEY", "")
    collection_name: str = os.getenv("COLLECTION_NAME", "textbook_embeddings")

    # Chunking
    chunk_size: int = int(os.getenv("CHUNK_SIZE", "512"))
    chunk_overlap: int = int(os.getenv("CHUNK_OVERLAP", "50"))

    # Pipeline
    checkpoint_file: str = os.getenv("CHECKPOINT_FILE", "ingestion_checkpoint.json")
    log_level: str = os.getenv("LOG_LEVEL", "INFO")

    # Target
    textbook_base_url: str = os.getenv("TEXTBOOK_BASE_URL", "")

    def validate(self) -> None:
        """Validate required configuration.

        Raises:
            ValueError: If required configuration is missing
        """
        required: list[str] = ["cohere_api_key", "qdrant_url", "qdrant_api_key", "textbook_base_url"]
        missing: list[str] = [key for key in required if not getattr(self, key)]
        if missing:
            raise ValueError(f"Missing required config: {', '.join(missing)}")
