"""URL crawling and HTML text extraction."""

import logging
from typing import List, Tuple
from urllib.parse import urljoin, urlparse
from html import unescape

import requests
from bs4 import BeautifulSoup


logger = logging.getLogger(__name__)


class URLCrawler:
    """Crawl Vercel-hosted documentation and extract clean text."""

    def __init__(self, base_url: str, timeout: int = 30):
        """Initialize crawler.

        Args:
            base_url: Base URL to crawl
            timeout: Request timeout in seconds
        """
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout
        self.visited_urls = set()
        self.failed_urls = []

    def crawl(self) -> List[dict]:
        """Crawl base URL and discover all internal links.

        Returns:
            List of page dicts with extracted content
        """
        pages = []
        to_visit = [self.base_url]

        while to_visit:
            url = to_visit.pop(0)

            if url in self.visited_urls:
                continue

            self.visited_urls.add(url)

            try:
                page = self.fetch_and_extract(url)
                if page:
                    pages.append(page)

                    # Discover new links
                    discovered_urls = self._discover_links(page.get("raw_html", ""))
                    for discovered_url in discovered_urls:
                        if discovered_url not in self.visited_urls and discovered_url not in to_visit:
                            to_visit.append(discovered_url)

            except Exception as e:
                logger.error(f"Failed to crawl {url}: {e}")
                self.failed_urls.append({"url": url, "error": str(e)})

        logger.info(f"Crawled {len(pages)} pages. Failed: {len(self.failed_urls)}")
        return pages

    def fetch_and_extract(self, url: str) -> dict:
        """Fetch URL and extract clean text.

        Args:
            url: URL to fetch

        Returns:
            Dict with extracted content and metadata
        """
        try:
            response = requests.get(url, timeout=self.timeout)
            response.raise_for_status()
            response.encoding = 'utf-8'

            # Parse HTML
            soup = BeautifulSoup(response.text, 'html.parser')

            # Extract title
            title = soup.find('title')
            page_title = title.get_text(strip=True) if title else "Untitled"

            # Extract main content
            article = soup.find('article') or soup.find('main')
            if not article:
                logger.warning(f"No article/main tag found in {url}")
                return None

            # Remove boilerplate
            for tag in article(['script', 'style', 'nav', 'header', 'footer']):
                tag.decompose()

            # Extract text
            text = article.get_text(separator=' ', strip=True)
            text = unescape(text)  # Unescape HTML entities

            # Extract section headers
            section_headers = []
            for header in article.find_all(['h1', 'h2', 'h3']):
                header_text = header.get_text(strip=True)
                if header_text:
                    section_headers.append(header_text)

            return {
                "url": url,
                "page_title": page_title,
                "section_headers": section_headers[:5],  # Keep first 5 headers
                "extracted_text": text,
                "raw_html": response.text,
                "status_code": response.status_code
            }

        except requests.exceptions.Timeout:
            logger.error(f"Timeout fetching {url}")
            raise
        except requests.exceptions.RequestException as e:
            logger.error(f"Request failed for {url}: {e}")
            raise
        except Exception as e:
            logger.error(f"Extraction failed for {url}: {e}")
            raise

    def _discover_links(self, html: str) -> List[str]:
        """Discover internal links from HTML.

        Args:
            html: HTML content

        Returns:
            List of discovered internal URLs
        """
        soup = BeautifulSoup(html, 'html.parser')
        discovered = []

        for link in soup.find_all('a', href=True):
            href = link['href']

            # Skip anchors, external links
            if href.startswith('#') or href.startswith('http') and not href.startswith(self.base_url):
                continue

            # Convert relative to absolute
            if href.startswith('/'):
                url = urljoin(self.base_url, href)
            else:
                url = urljoin(self.base_url + '/', href)

            # Only include URLs under base domain
            if urlparse(url).netloc == urlparse(self.base_url).netloc:
                # Remove fragments and query params for consistency
                url = url.split('#')[0]
                if url not in self.visited_urls:
                    discovered.append(url)

        return discovered

    def get_stats(self) -> dict:
        """Get crawl statistics.

        Returns:
            Stats dict
        """
        return {
            "visited_urls": len(self.visited_urls),
            "failed_urls": len(self.failed_urls),
            "failed_details": self.failed_urls
        }
