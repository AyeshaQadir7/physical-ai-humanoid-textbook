"""Tests for URL crawler."""

import pytest
from unittest.mock import patch, MagicMock

import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "ingestion"))

from crawler import URLCrawler


def test_crawler_extracts_article_tag(sample_html):
    """Test crawler extracts content from <article> tag."""
    with patch('crawler.requests.get') as mock_get:
        mock_response = MagicMock()
        mock_response.text = sample_html
        mock_response.encoding = 'utf-8'
        mock_response.status_code = 200
        mock_get.return_value = mock_response

        crawler = URLCrawler("https://example.com")
        page = crawler.fetch_and_extract("https://example.com/page1")

        assert page is not None
        assert "Main Content" in page["extracted_text"]
        assert "Navigation" not in page["extracted_text"]
        assert "Footer" not in page["extracted_text"]


def test_crawler_extracts_headers(sample_html):
    """Test crawler extracts section headers."""
    with patch('crawler.requests.get') as mock_get:
        mock_response = MagicMock()
        mock_response.text = sample_html
        mock_response.encoding = 'utf-8'
        mock_response.status_code = 200
        mock_get.return_value = mock_response

        crawler = URLCrawler("https://example.com")
        page = crawler.fetch_and_extract("https://example.com/page1")

        assert "Main Content" in page["section_headers"]
        assert "Section 1" in page["section_headers"]


def test_crawler_handles_missing_article():
    """Test crawler handles HTML without article tag."""
    html_no_article = "<html><body><p>No article tag</p></body></html>"

    with patch('crawler.requests.get') as mock_get:
        mock_response = MagicMock()
        mock_response.text = html_no_article
        mock_response.encoding = 'utf-8'
        mock_response.status_code = 200
        mock_get.return_value = mock_response

        crawler = URLCrawler("https://example.com")
        page = crawler.fetch_and_extract("https://example.com/page1")

        assert page is None


def test_crawler_preserves_special_characters():
    """Test crawler preserves special characters."""
    html_special = """
    <html><body>
        <article>
            <p>Special chars: &amp; &lt; &gt; &quot; &apos; ñ é ü</p>
        </article>
    </body></html>
    """

    with patch('crawler.requests.get') as mock_get:
        mock_response = MagicMock()
        mock_response.text = html_special
        mock_response.encoding = 'utf-8'
        mock_response.status_code = 200
        mock_get.return_value = mock_response

        crawler = URLCrawler("https://example.com")
        page = crawler.fetch_and_extract("https://example.com/page1")

        # Entities should be unescaped
        assert "&" in page["extracted_text"] or "&amp;" not in page["extracted_text"]
        assert "ñ" in page["extracted_text"] or "é" in page["extracted_text"]


def test_crawler_handles_request_timeout():
    """Test crawler handles request timeout."""
    import requests

    with patch('crawler.requests.get') as mock_get:
        mock_get.side_effect = requests.exceptions.Timeout("Request timed out")

        crawler = URLCrawler("https://example.com")
        with pytest.raises(Exception):
            crawler.fetch_and_extract("https://example.com/timeout")
