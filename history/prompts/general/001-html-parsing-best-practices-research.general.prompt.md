---
id: 001
title: HTML Parsing Best Practices Research
stage: general
date: 2025-12-25
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: 001-website-ingestion
user: Ayeha Qadir7
command: research
labels: ["research", "html-parsing", "beautifulsoup", "docusaurus", "utf-8"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
tests:
---

## Prompt

Research best practices for HTML parsing and clean text extraction in Python for a Docusaurus website. Focus on:

1. **BeautifulSoup vs other libraries**: Compare BeautifulSoup, Selenium, Playwright for extracting clean text from HTML
2. **Docusaurus structure**: What HTML structure/tags does Docusaurus typically use for main content (article, main, semantic tags)?
3. **Boilerplate filtering**: Best practices for removing navigation, footers, headers from extracted content while preserving article body
4. **UTF-8 and special characters**: How to handle mixed character sets and preserve them without data loss

Return findings in structured format with decision and rationale for each.

## Response snapshot

Research completed on HTML parsing best practices for Docusaurus websites. Key findings:

1. **Library Selection**: BeautifulSoup recommended for static content extraction due to performance and simplicity. Selenium/Playwright needed only for JavaScript-heavy dynamic content.

2. **Docusaurus Structure**: Uses semantic `<article>` tag for main content by default, with `<main>`, `<nav>`, `<aside>`, and `<footer>` for page structure.

3. **Boilerplate Filtering**: Use `find()` to target `<article>` or `<main>` tags directly, combined with element blacklist filtering for navigation, scripts, and headers.

4. **UTF-8 Handling**: Python 3 defaults to UTF-8. Use explicit encoding parameters and `html.unescape()` for HTML entities to preserve special characters.

Detailed findings with sources provided in structured decision format with rationale for each recommendation.

## Outcome

- ✅ Impact: Comprehensive research completed with actionable recommendations for HTML parsing implementation
- 🧪 Tests: No tests run (research task)
- 📁 Files: PHR created
- 🔁 Next prompts: Implement HTML parser for Docusaurus based on findings
- 🧠 Reflection: Research covered all requested areas with multiple authoritative sources

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Implement parser prototype using BeautifulSoup with article tag extraction
