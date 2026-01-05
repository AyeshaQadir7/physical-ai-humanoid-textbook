"""Integration tests for crawl and chunk pipeline."""

import pytest
from unittest.mock import patch, MagicMock

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "ingestion"))

from crawler import URLCrawler
from chunker import TextChunker


def test_crawl_and_chunk_pipeline():
    """Test complete crawl and chunk workflow."""
    html = """
    <html><body>
        <article>
            <h1>Title</h1>
            <p>Content paragraph. </p> * 100
        </article>
    </body></html>
    """

    with patch('crawler.requests.get') as mock_get:
        mock_response = MagicMock()
        mock_response.text = html
        mock_response.encoding = 'utf-8'
        mock_response.status_code = 200
        mock_get.return_value = mock_response

        # Crawl
        crawler = URLCrawler("https://example.com")
        pages = crawler.crawl()

        assert len(pages) > 0

        # Chunk
        chunker = TextChunker()
        all_chunks = []
        for page in pages:
            chunks = chunker.chunk_text(
                page["extracted_text"],
                page["url"],
                page["page_title"],
                page["section_headers"]
            )
            all_chunks.extend(chunks)

        assert len(all_chunks) > 0
        assert all("metadata" in c for c in all_chunks)
        assert all("hash" in c for c in all_chunks)


@pytest.mark.skip(reason="Requires live URL")
def test_with_real_vercel_url():
    """Test with real Vercel URL (manual test)."""
    crawler = URLCrawler("https://physical-ai-robotics.vercel.app")
    pages = crawler.crawl()

    assert len(pages) > 0

    chunker = TextChunker()
    for page in pages:
        chunks = chunker.chunk_text(
            page["extracted_text"],
            page["url"],
            page["page_title"],
            page["section_headers"]
        )
        assert len(chunks) > 0
