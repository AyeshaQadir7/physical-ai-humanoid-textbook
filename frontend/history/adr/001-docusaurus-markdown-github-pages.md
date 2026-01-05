# ADR-001: Documentation Stack - Docusaurus v2+ + Markdown + GitHub Pages

**Status**: Accepted
**Date**: 2025-12-10
**Decision Cluster**: Publishing & Documentation Platform
**References**: plan.md (Phase 0, 4), constitution.md (III. Docusaurus Markdown Compatibility)

---

## Context

The Physical AI & Humanoid Robotics textbook requires a publishing platform that:
- Hosts 20+ chapters across 5 modules (Foundations, ROS 2, Simulation, Isaac, VLA)
- Supports version control (Git) and continuous deployment
- Enables collaborative authoring with clear quality gates
- Renders consistently across devices (desktop, tablet, mobile)
- Maintains technical accuracy through code example testing and validation
- Provides search functionality and cross-references
- Requires minimal maintenance and hosting costs
- Supports student offline access and printing

The textbook is educational content for a 13-week capstone course, not a live SaaS application. Publication cadence is once per quarter, with minor updates between cohorts.

---

## Decision

**Adopt Docusaurus v2+ as the static site generator**, using:

1. **Documentation Platform**: Docusaurus v2 (Meta-maintained, React-based)
   - Markdown input with Docusaurus-compatible frontmatter (id, title, sidebar_position, description, keywords)
   - Auto-generated sidebar from sidebar_position metadata
   - Built-in search (powered by Algolia or local search)
   - KaTeX for mathematical notation support
   - Admonition support (:::note, :::warning, :::tip, :::danger)

2. **Content Format**: Markdown (standard CommonMark + Docusaurus extensions)
   - All chapters authored in Markdown in `docs/` folder
   - Inline code examples (fenced blocks with language tags)
   - Relative paths for internal links (e.g., `../module-1/basics.md`)
   - Full URLs for external links (e.g., `https://docs.ros.org/...`)
   - Images stored in `docs/assets/` with relative references

3. **Hosting & Deployment**: GitHub Pages
   - Static site generated via `npm run build`
   - Deployed to `gh-pages` branch via `npm run deploy` or GitHub Actions
   - Live at `https://[org].github.io/physical-ai-robotics-textbook/`
   - Custom domain optional (e.g., `textbook.organization.edu`)

4. **Version Control**: Git + GitHub
   - All Markdown and config in main repository
   - Build artifacts (static site) in `gh-pages` branch (auto-generated, not committed)
   - PHRs and ADRs stored in `history/` for traceability
   - Chapter reviews via pull requests with constitution checklist

---

## Rationale

### Why Docusaurus?

- **Docusaurus is optimized for technical documentation**: Designed for multi-page, interconnected technical content (vs. blog platforms like Gatsby or Hugo)
- **Low barrier to entry**: Markdown authoring requires no HTML/CSS knowledge; contributors can focus on content
- **Sidebar auto-generation**: Eliminates manual navigation maintenance; sidebar_position metadata keeps content and nav in sync
- **Math support (KaTeX)**: Essential for robotics/physics notation without raw LaTeX compilation overhead
- **Ecosystem**: Plugins for analytics, search, versioning (if needed post-launch)
- **Maturity**: Meta-maintained since 2020; widely adopted by React, Babel, Jest communities

### Why Markdown?

- **Version-control-friendly**: Plain text, diffs are human-readable; no proprietary formats
- **Low friction for collaboration**: Non-technical reviewers can read/edit in GitHub; no special software required
- **Portable**: Easy to migrate to other platforms later if needed (e.g., LaTeX for PDF book, MkDocs, Hugo)
- **Docusaurus compatibility**: Markdown is primary input; zero translation step

### Why GitHub Pages?

- **Cost**: Free static hosting; no infrastructure management
- **Integration**: Seamless with GitHub repository; one-command deploy
- **Reliability**: Backed by GitHub/Microsoft infrastructure; 99.9% uptime SLA
- **Simplicity**: No CI/CD pipeline complexity; `gh-pages` branch is automatic
- **Workflow**: Students can contribute via GitHub (fork, PRs, issues) using skills they likely have

### Tradeoffs

| Aspect | Upside | Downside |
|--------|--------|----------|
| **Static site** | Fast, cheap, no servers | No dynamic interactivity (polls, real-time updates) |
| **GitHub Pages** | Free, integrated | Minor latency for international users; limited customization |
| **Markdown** | Simple, portable | Less expressive than HTML/JSX; no advanced layouts |
| **Docusaurus** | Great for docs, actively maintained | Tied to React/Node.js ecosystem; not suitable for non-documentation sites |

### Long-term Maintainability

- **Upstream stability**: Docusaurus v2 API stable since 2021; breaking changes infrequent
- **Dependency management**: Version-pin `package.json`; use renovate or dependabot for updates
- **Content independence**: Content (Markdown) is independent of Docusaurus version; migration possible if ever needed
- **Team skills**: Markdown + Git are durable skills; any team can maintain

---

## Alternatives Considered

### Alternative A: WordPress + Custom Theme
- **Pros**: WYSIWYG editor; easy for non-technical users
- **Cons**: Security maintenance burden; database required; overkill for static content; plugin sprawl; hosting costs
- **Decision**: Rejected (too much overhead for a static textbook)

### Alternative B: LaTeX → PDF + MkDocs HTML
- **Pros**: Beautiful print output; professional typesetting
- **Cons**: Steep learning curve (LaTeX syntax); fragmented tooling; slower feedback loop for content authors; PDF and HTML are separate maintenance burdens
- **Decision**: Rejected (prioritize authoring velocity and web-first experience; PDF can be generated from Docusaurus later if needed)

### Alternative C: Notion or Google Docs (Shared workspace)
- **Pros**: Collaborative editing; real-time comments
- **Cons**: Not version-controlled; proprietary format; no Git integration; limited customization; offline access poor
- **Decision**: Rejected (inconsistent with Spec-Kit Plus philosophy of version-controlled, auditable content)

### Alternative D: Custom Next.js + Vercel
- **Pros**: Maximum customization; trendy
- **Cons**: Overkill for static content; requires full-stack engineering; higher hosting costs; more maintenance; slower time-to-market
- **Decision**: Rejected (Docusaurus solves 95% of the need with 10% of the complexity)

---

## Consequences

### Positive
- **Fast iteration**: Authors can create chapters in Markdown, submit PRs, and see live preview within hours
- **Quality gates**: Constitution checklist becomes a PR template; reviews are tied to version control
- **Searchability**: Students can find concepts across chapters; built-in search reduces support burden
- **Offline access**: Entire site can be downloaded/printed; students not dependent on internet
- **Accessibility**: Docusaurus generates accessible HTML; screen reader compatible
- **Analytics**: Integrate Docusaurus analytics plugin to track which chapters are most viewed (feedback signal)

### Negative
- **Learning curve for HTML/JSX**: If advanced customization needed beyond Markdown (e.g., interactive components), requires JavaScript/React knowledge
- **Limited interactivity**: No embedded simulations, quizzes, or live coding environments in the textbook itself (those remain external, linked)
- **GitHub Pages limitations**: Custom domain setup required for branded URL; subdomains have some limitations with HTTPS
- **Static search**: Algolia is free for open-source but has indexing limits; self-hosted search lighter-weight but requires more setup

### Risk Mitigation
- **API changes**: Docusaurus v2 is stable; if major version break occurs, Markdown content migrates easily (e.g., to Hugo)
- **Hosting outage**: GitHub Pages has 99.9% SLA; backup: export static site + host on AWS S3 if needed
- **Complexity creep**: Avoid custom Docusaurus plugins unless absolutely necessary; keep site "vanilla"

---

## Implementation

### Phase 0 Steps
1. Bootstrap Docusaurus: `npx create-docusaurus@latest textbook classic`
2. Install math support: `npm install katex remark-math rehype-katex`
3. Create folder structure (see plan.md, Phase 0, Step 0.1)
4. Configure `docusaurus.config.js`: KaTeX plugin, baseUrl for GitHub Pages
5. Configure `sidebars.js`: Auto-generate from `sidebar_position` frontmatter
6. First chapter (intro.md) as proof-of-concept

### Chapter Authoring
- Every chapter must include frontmatter: `id`, `title`, `sidebar_position`, `description`, `keywords`
- Use relative paths for internal links (e.g., `../module-1/basics.md`)
- Use full URLs for external links
- Images: store in `docs/assets/`, reference with relative paths

### Deployment
- Local testing: `npm run build && npm run serve`
- Push to main repository (Git)
- Deploy: `npm run deploy` (or GitHub Actions)
- Monitor: Check GitHub Pages live site after each deployment

---

## Related Decisions

- **ADR-002**: Simulation-First Hardware Strategy (independent; complements Docusaurus publishing)
- **ADR-003**: Modular Progressive Architecture (independent; content structure, independent of platform choice)

---

## Sign-Off

**Decision Maker**: Architect (on behalf of project team)
**Consensus**: Ready for Phase 0 bootstrap

---

**Next Action**: Begin Phase 0 Docusaurus setup; test with first chapter (intro.md)
