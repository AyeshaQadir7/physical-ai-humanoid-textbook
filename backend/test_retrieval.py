"""Test RAG retrieval pipeline - diagnose issues with Qdrant and Cohere."""

import os
import sys
import logging
from pathlib import Path
from dotenv import load_dotenv

# Setup logging
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s - %(levelname)s - %(message)s"
)
logger = logging.getLogger(__name__)

# Load environment
env_path = Path(__file__).parent / ".env"
load_dotenv(env_path)

def test_qdrant():
    """Test Qdrant connection and collection."""
    logger.info("\n=== Testing Qdrant ===")

    try:
        from qdrant_client import QdrantClient

        qdrant_url = os.getenv("QDRANT_URL")
        qdrant_api_key = os.getenv("QDRANT_API_KEY")
        collection_name = os.getenv("QDRANT_COLLECTION_NAME", "textbook_embeddings")

        logger.info(f"Connecting to: {qdrant_url}")
        client = QdrantClient(url=qdrant_url, api_key=qdrant_api_key)

        # Check if collection exists
        if client.collection_exists(collection_name):
            logger.info(f"✓ Collection '{collection_name}' exists")

            # Get collection info
            info = client.get_collection(collection_name)
            logger.info(f"  - Points: {info.points_count}")
            logger.info(f"  - Vector size: {info.config.params.vectors.size}")

            if info.points_count == 0:
                logger.error("✗ Collection is EMPTY! No embeddings found.")
                logger.info("  → Run the embedding pipeline first")
                return False
            return True
        else:
            logger.error(f"✗ Collection '{collection_name}' does NOT exist")
            logger.info("  Available collections:")
            collections = client.get_collections()
            for col in collections.collections:
                logger.info(f"    - {col.name}")
            return False

    except Exception as e:
        logger.error(f"✗ Qdrant error: {type(e).__name__}: {e}")
        return False

def test_cohere():
    """Test Cohere embedding API."""
    logger.info("\n=== Testing Cohere API ===")

    try:
        import cohere

        api_key = os.getenv("COHERE_API_KEY")
        logger.info("Testing Cohere embedding...")

        client = cohere.ClientV2(api_key=api_key)
        response = client.embed(
            model="embed-english-v3.0",
            texts=["robotics"],
            input_type="search_query"
        )

        embedding_dim = len(response.embeddings.float[0])
        logger.info(f"✓ Cohere API working - generated {embedding_dim} dimensional embedding")
        return True

    except Exception as e:
        logger.error(f"✗ Cohere error: {type(e).__name__}: {e}")
        return False

def test_retriever_client():
    """Test RetrieverClient."""
    logger.info("\n=== Testing RetrieverClient ===")

    try:
        from retrieve import RetrieverClient

        logger.info("Initializing RetrieverClient...")
        client = RetrieverClient(
            qdrant_url=os.getenv("QDRANT_URL"),
            qdrant_api_key=os.getenv("QDRANT_API_KEY"),
            cohere_api_key=os.getenv("COHERE_API_KEY"),
            collection_name=os.getenv("QDRANT_COLLECTION_NAME", "textbook_embeddings")
        )

        logger.info("✓ RetrieverClient initialized")

        # Try validation
        logger.info("Running configuration validation...")
        client._validate_config()
        logger.info("✓ Configuration validated")

        return True

    except Exception as e:
        logger.error(f"✗ RetrieverClient error: {type(e).__name__}: {e}")
        return False

def test_search():
    """Test actual search."""
    logger.info("\n=== Testing Search Query ===")

    try:
        from retrieve import RetrieverClient

        client = RetrieverClient(
            qdrant_url=os.getenv("QDRANT_URL"),
            qdrant_api_key=os.getenv("QDRANT_API_KEY"),
            cohere_api_key=os.getenv("COHERE_API_KEY"),
            collection_name=os.getenv("QDRANT_COLLECTION_NAME", "textbook_embeddings")
        )

        queries = [
            "What is ROS2?",
            "Explain robotics",
            "How do I use simulation?"
        ]

        for query in queries:
            logger.info(f"\nSearching for: '{query}'")
            response = client.search(query, top_k=3)

            if response.get("status") == "error":
                logger.error(f"  ✗ Search error: {response['error']['message']}")
                continue

            results = response.get("results", [])
            logger.info(f"  Found {len(results)} results:")

            for i, result in enumerate(results, 1):
                score = result.get("similarity_score", 0)
                title = result.get("metadata", {}).get("page_title", "Unknown")
                logger.info(f"    [{i}] Score: {score:.3f} | {title}")
                if score < 0.5:
                    logger.warning(f"        ⚠ Low similarity score!")

        return len(results) > 0

    except Exception as e:
        logger.error(f"✗ Search error: {type(e).__name__}: {e}")
        return False

def main():
    """Run all diagnostics."""
    logger.info("=" * 60)
    logger.info("RAG RETRIEVAL DIAGNOSTICS")
    logger.info("=" * 60)

    results = {
        "Qdrant": test_qdrant(),
        "Cohere": test_cohere(),
        "RetrieverClient": test_retriever_client(),
        "Search": test_search(),
    }

    logger.info("\n" + "=" * 60)
    logger.info("SUMMARY")
    logger.info("=" * 60)

    for component, passed in results.items():
        status = "✓ PASS" if passed else "✗ FAIL"
        logger.info(f"{component}: {status}")

    all_passed = all(results.values())

    if not all_passed:
        logger.error("\n⚠ ISSUES DETECTED:")
        if not results["Qdrant"]:
            logger.error("  - Qdrant collection is empty or missing")
            logger.error("    → Run the embedding/ingestion pipeline first")
        if not results["Cohere"]:
            logger.error("  - Cohere API is not accessible")
            logger.error("    → Check API key and internet connection")
        if not results["Search"]:
            logger.error("  - Search is failing or returning no results")
            logger.error("    → Check embeddings or Qdrant data")

    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())
