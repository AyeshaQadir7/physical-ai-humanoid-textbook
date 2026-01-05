"""Checkpoint management for resume capability."""

import json
import logging
from pathlib import Path
from typing import Set


logger = logging.getLogger(__name__)


class CheckpointManager:
    """Track and persist ingestion progress for resume capability."""

    def __init__(self, checkpoint_file: str):
        """Initialize checkpoint manager.

        Args:
            checkpoint_file: Path to JSON checkpoint file
        """
        self.checkpoint_file = Path(checkpoint_file)
        self.processed_hashes: Set[str] = self._load_checkpoint()

    def _load_checkpoint(self) -> Set[str]:
        """Load processed hashes from checkpoint file."""
        if self.checkpoint_file.exists():
            try:
                with open(self.checkpoint_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                    logger.info(f"Loaded checkpoint: {len(data.get('processed_hashes', []))} chunks processed")
                    return set(data.get('processed_hashes', []))
            except Exception as e:
                logger.warning(f"Failed to load checkpoint: {e}. Starting fresh.")
                return set()
        return set()

    def mark_processed(self, chunk_hash: str) -> None:
        """Mark a chunk as processed and save checkpoint.

        Args:
            chunk_hash: SHA256 hash of chunk content
        """
        self.processed_hashes.add(chunk_hash)
        self._save_checkpoint()

    def get_processed_hashes(self) -> Set[str]:
        """Get all processed chunk hashes.

        Returns:
            Set of processed hashes
        """
        return self.processed_hashes.copy()

    def is_processed(self, chunk_hash: str) -> bool:
        """Check if chunk was already processed.

        Args:
            chunk_hash: SHA256 hash of chunk content

        Returns:
            True if chunk was processed
        """
        return chunk_hash in self.processed_hashes

    def _save_checkpoint(self) -> None:
        """Save checkpoint to file."""
        try:
            with open(self.checkpoint_file, 'w', encoding='utf-8') as f:
                json.dump({'processed_hashes': list(self.processed_hashes)}, f)
        except Exception as e:
            logger.error(f"Failed to save checkpoint: {e}")

    def clear(self) -> None:
        """Clear checkpoint for fresh run."""
        self.processed_hashes.clear()
        if self.checkpoint_file.exists():
            self.checkpoint_file.unlink()
        logger.info("Checkpoint cleared")
