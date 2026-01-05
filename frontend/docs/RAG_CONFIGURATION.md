# RAG Configuration Plan

**Document**: Configuration strategy for SPEC-4 RAG Chatbot
**Phase**: Phase 1 - Setup (T002)
**Status**: Planning
**Date**: 2025-12-26

---

## Overview

This document describes how the `REACT_APP_RAG_AGENT_URL` environment variable will be injected into the Docusaurus frontend for the chatbot feature (SPEC-4).

---

## Configuration Injection Strategy

### Entry Point: docusaurus.config.ts

The Docusaurus config file runs in Node.js and has access to `process.env` at build time. This is where we will read the `REACT_APP_RAG_AGENT_URL` environment variable and make it available to the frontend client code.

**Current Config File**: `frontend/docusaurus.config.ts` (lines 1-166)

**Injection Point**: Add a new configuration object after line 5 (after the imports, before the config const) to read and validate the environment variable.

### Implementation Plan

#### Step 1: Add Environment Variable Reader (To be done in T025)

In `docusaurus.config.ts`, add after imports:

```typescript
// Read RAG Agent API URL from environment
const RAG_AGENT_URL = process.env.REACT_APP_RAG_AGENT_URL || 'http://localhost:8000/chat';

// Log configuration for debugging (production should not log URLs for security)
if (process.env.NODE_ENV !== 'production') {
  console.log(`[RAG Config] Using RAG Agent URL: ${RAG_AGENT_URL}`);
}
```

#### Step 2: Export Configuration via clientModules (To be done in T025)

Add a `clientModules` array to the Docusaurus config to expose the URL to client-side code:

```typescript
const config: Config = {
  // ... existing config ...

  // Expose RAG configuration to client-side
  clientModules: [
    './src/config/ragConfig.ts',  // Module will read from __RAG_AGENT_URL__ global
  ],
};
```

#### Step 3: Create Configuration Module (To be done in T004 - Phase 2)

Create `frontend/src/config/ragConfig.ts` that reads the injected configuration:

```typescript
/**
 * RAG Agent API Configuration
 * Reads the RAG_AGENT_URL injected at build time and provides it to the chatbot.
 */

// This will be injected by Docusaurus at build time
declare global {
  var __RAG_AGENT_URL__: string;
}

export const getRagAgentUrl = (): string => {
  // Try to read from injected global variable first (production build)
  if (typeof __RAG_AGENT_URL__ !== 'undefined') {
    return __RAG_AGENT_URL__;
  }

  // Fallback to environment variable (development)
  return (
    process.env.REACT_APP_RAG_AGENT_URL ||
    'http://localhost:8000/chat'
  );
};

export const ragConfig = {
  apiUrl: getRagAgentUrl(),
};
```

---

## Environment Variables

### Development (.env.local)

Create `frontend/.env.local`:

```env
REACT_APP_RAG_AGENT_URL=http://localhost:8000/chat
```

### Production (.env.production)

Set via CI/CD environment or deployment platform:

```env
REACT_APP_RAG_AGENT_URL=https://api.robotics-textbook.example.com/chat
```

### Default Value

If `REACT_APP_RAG_AGENT_URL` is not set:
- **Development**: `http://localhost:8000/chat` (localhost dev server)
- **Production**: `http://localhost:8000/chat` (fallback, but should always be set in production)

---

## Build-Time vs. Runtime Configuration

### Approach: Build-Time Injection

**Why**: Docusaurus is a static site generator. Configuration must be baked into the bundle at build time, not read at runtime from the browser.

**How**:
1. At build time (`npm run build`), read `REACT_APP_RAG_AGENT_URL` from `process.env`
2. Inject the URL into the JavaScript bundle as a global variable or constant
3. Client-side code reads this injected value

**Advantages**:
- Configuration is immutable once deployed
- Works with static hosting (Vercel, Netlify, GitHub Pages)
- No runtime API calls needed to fetch configuration

**Disadvantages**:
- Cannot change configuration without rebuilding
- Different builds needed for different environments

---

## Validation Checklist

### During Build
- [ ] Environment variable `REACT_APP_RAG_AGENT_URL` is present
- [ ] Variable is a valid URL (starts with http:// or https://)
- [ ] Variable ends with `/chat` endpoint (or handle in code)
- [ ] Default fallback works if variable is missing

### During Development (`npm run build`)
- [ ] Configuration is read from `.env.local`
- [ ] Default value `http://localhost:8000/chat` is used if not set
- [ ] Built JavaScript includes the URL

### During Deployment (T027)
- [ ] CI/CD sets `REACT_APP_RAG_AGENT_URL` before build
- [ ] Production URL is production domain (not localhost)
- [ ] Built bundle contains production URL

---

## File Modifications Summary

| File | Change | Phase | Task |
|------|--------|-------|------|
| `frontend/docusaurus.config.ts` | Add env var reader + clientModules export | Phase 2 | T025 |
| `frontend/src/config/ragConfig.ts` | Create configuration module | Phase 2 | T004 |
| `frontend/.env.example` | Add example env vars | Phase 2 | T026 |
| `frontend/.env.local` | (Dev only, not committed) Create for development | Phase 2 | T026 |
| `specs/004-rag-chat-ui/DEPLOYMENT.md` | Document deployment configuration | Phase 5 | T027 |

---

## Related Documents

- **spec.md**: Feature specification (US3-P3: Configure Backend URL)
- **plan.md**: Architecture and design decisions
- **tasks.md**: Implementation tasks
- **.env.example**: Environment variable template (T026)
- **DEPLOYMENT.md**: Production deployment guide (T027)

---

## Next Steps

1. **Phase 2 (T004)**: Create `frontend/src/config/ragConfig.ts` configuration module
2. **Phase 2 (T025)**: Update `frontend/docusaurus.config.ts` to read and inject environment variable
3. **Phase 2 (T026)**: Create `.env.example` and `.env.local` files
4. **Phase 5 (T027)**: Create deployment guide documenting the configuration for production

---

## Questions & Decisions

**Q**: Should the configuration be validated at build time or runtime?
**A**: Validated at runtime to avoid breaking builds. If URL is invalid, chatbot shows error message to user.

**Q**: Should the URL be logged?
**A**: Yes, in development only (non-production). Log at info level for debugging.

**Q**: What if REACT_APP_RAG_AGENT_URL is not set?
**A**: Use default `http://localhost:8000/chat`. Works for local development; must be set for production deployments.

**Q**: Can users change the URL after deployment?
**A**: No. Configuration is baked into the bundle at build time. Would need to rebuild to change.
