"""Integration test for complete pipeline."""

import pytest
from unittest.mock import patch, MagicMock

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "ingestion"))

from main import IngestionPipeline
from config import Config


@pytest.mark.skip(reason="Requires live APIs")
def test_full_pipeline_with_live_apis():
    """Test complete pipeline with real Cohere and Qdrant (requires API keys)."""
    config = Config()
    config.validate()

    pipeline = IngestionPipeline(config)
    report = pipeline.run()

    assert report["status"] == "success"
    assert report["summary"]["total_chunks_created"] > 0
    assert report["summary"]["total_embeddings_generated"] >= 0


def test_pipeline_handles_config_error():
    """Test pipeline handles missing configuration."""
    config = Config()
    config.cohere_api_key = ""  # Missing required key

    with pytest.raises(ValueError):
        config.validate()
