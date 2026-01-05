# Deployment Guide: RAG Chatbot (SPEC-4)

**Feature**: Integrate RAG Backend with Docusaurus Frontend
**Status**: Complete
**Last Updated**: 2025-12-26

---

## Overview

This guide covers deploying the RAG chatbot frontend (Docusaurus with integrated ChatBot component) to different environments (development, staging, production).

---

## Prerequisites

- Node.js >= 20.0
- npm or yarn package manager
- FastAPI backend running (Spec 1-3)
- Qdrant vector database with indexed content
- OpenAI API key configured on backend

---

## Development Deployment

### Local Setup

1. **Clone and Install Dependencies**
   ```bash
   cd frontend
   npm install
   ```

2. **Configure Environment**
   ```bash
   cp .env.example .env.local
   # Edit .env.local:
   REACT_APP_RAG_AGENT_URL=http://localhost:8000/chat
   ```

3. **Start Development Server**
   ```bash
   npm run start
   ```
   - Opens http://localhost:3000
   - Hot-reload enabled
   - Browser opens automatically

4. **Start FastAPI Backend** (in another terminal)
   ```bash
   cd backend
   python -m uvicorn ingestion.agent:app --reload --port 8000
   ```

### Testing Locally

```bash
# Run TypeScript type-checking
npm run typecheck

# Run tests (if configured)
npm run test

# Build for production
npm run build
```

---

## Staging Deployment

### Build Configuration

1. **Build for Staging**
   ```bash
   REACT_APP_RAG_AGENT_URL=https://staging-api.robotics-textbook.example.com/chat npm run build
   ```

2. **Environment Variables**
   - `REACT_APP_RAG_AGENT_URL`: Staging API endpoint
   - `REACT_APP_REQUEST_TIMEOUT`: 30000 (30 seconds)
   - `REACT_APP_MAX_CHUNKS`: 10

3. **Verify Build**
   ```bash
   npm run build
   # Check build/ directory was created
   ls -la build/
   ```

### Deployment Steps

1. **Deploy to Staging Server**
   ```bash
   # Option 1: Deploy to Vercel
   vercel --prod

   # Option 2: Deploy to Netlify
   netlify deploy --prod --dir=build

   # Option 3: Manual deployment
   scp -r build/* user@staging.example.com:/var/www/robotics-textbook/
   ```

2. **Verify Deployment**
   ```bash
   curl https://staging-robotics-textbook.example.com/
   # Should return Docusaurus HTML
   ```

3. **Test Chatbot Integration**
   - Open staging URL in browser
   - Verify ChatBot appears (bottom-right sidebar)
   - Submit test query: "What is ROS2?"
   - Verify response appears with sources
   - Test on mobile (resize to <768px)

---

## Production Deployment

### Pre-Deployment Checklist

- [ ] All tests pass locally (`npm test`)
- [ ] TypeScript compilation succeeds (`npm run typecheck`)
- [ ] Production API endpoint is accessible
- [ ] CORS is configured on backend for production domain
- [ ] Qdrant database is populated with latest content
- [ ] OpenAI API key is configured
- [ ] Error logging is configured
- [ ] Analytics tracking is configured

### Build for Production

1. **Set Production Environment**
   ```bash
   export REACT_APP_RAG_AGENT_URL=https://api.robotics-textbook.org/chat
   export NODE_ENV=production
   ```

2. **Build**
   ```bash
   npm run build
   # Optimized, minified bundle created in build/
   ```

3. **Build Size Check**
   ```bash
   npm run build -- --analyze
   # Should be < 200KB gzipped
   ```

### Deployment Options

#### Option 1: Vercel (Recommended)

```bash
# Install Vercel CLI
npm install -g vercel

# Login
vercel login

# Deploy
vercel --prod -e REACT_APP_RAG_AGENT_URL=https://api.robotics-textbook.org/chat

# Verify
vercel ls
```

**Vercel Configuration** (`vercel.json`):
```json
{
  "buildCommand": "npm run build",
  "outputDirectory": "build",
  "env": {
    "REACT_APP_RAG_AGENT_URL": "@rag_agent_url"
  }
}
```

#### Option 2: Netlify

```bash
# Install Netlify CLI
npm install -g netlify-cli

# Login
netlify login

# Deploy
netlify deploy --prod --dir=build -e REACT_APP_RAG_AGENT_URL=https://api.robotics-textbook.org/chat

# Verify
netlify sites
```

**Netlify Configuration** (`netlify.toml`):
```toml
[build]
  command = "npm run build"
  publish = "build"

[build.environment]
  REACT_APP_RAG_AGENT_URL = "https://api.robotics-textbook.org/chat"
```

#### Option 3: Docker Deployment

1. **Create Dockerfile**
   ```dockerfile
   FROM node:20-alpine AS builder
   WORKDIR /app
   COPY . .
   RUN npm install
   ARG REACT_APP_RAG_AGENT_URL
   ENV REACT_APP_RAG_AGENT_URL=$REACT_APP_RAG_AGENT_URL
   RUN npm run build

   FROM nginx:alpine
   COPY --from=builder /app/build /usr/share/nginx/html
   COPY nginx.conf /etc/nginx/conf.d/default.conf
   EXPOSE 80
   CMD ["nginx", "-g", "daemon off;"]
   ```

2. **Build and Push**
   ```bash
   docker build \
     --build-arg REACT_APP_RAG_AGENT_URL=https://api.robotics-textbook.org/chat \
     -t robotics-textbook-frontend:latest .

   docker push robotics-textbook-frontend:latest
   ```

3. **Deploy to Kubernetes**
   ```bash
   kubectl apply -f deployment.yaml
   kubectl set env deployment/frontend REACT_APP_RAG_AGENT_URL=https://api.robotics-textbook.org/chat
   ```

### Post-Deployment Verification

1. **Health Check**
   ```bash
   curl https://robotics-textbook.org/
   # Should return 200 with HTML
   ```

2. **Test Chatbot**
   - Open https://robotics-textbook.org in browser
   - Verify ChatBot appears on every page
   - Submit test query: "Explain ROS2"
   - Verify sources display correctly
   - Test on mobile device

3. **Performance Check**
   ```bash
   # Test page load time
   curl -w "@curl-format.txt" -o /dev/null -s https://robotics-textbook.org/

   # Check bundle size
   curl -I https://robotics-textbook.org/static/js/main.*.js | grep Content-Length
   ```

4. **Error Logging**
   - Monitor browser console for errors
   - Check server logs for API failures
   - Verify error messages display correctly

---

## Environment Variables

### Development
```env
REACT_APP_RAG_AGENT_URL=http://localhost:8000/chat
REACT_APP_REQUEST_TIMEOUT=30000
```

### Staging
```env
REACT_APP_RAG_AGENT_URL=https://staging-api.robotics-textbook.example.com/chat
REACT_APP_REQUEST_TIMEOUT=30000
```

### Production
```env
REACT_APP_RAG_AGENT_URL=https://api.robotics-textbook.org/chat
REACT_APP_REQUEST_TIMEOUT=30000
```

---

## Rollback Procedure

If production deployment has issues:

### Immediate Rollback (< 5 minutes downtime)

```bash
# Vercel
vercel rollback

# Netlify
netlify deploy --prod --dir=build # Deploy previous version

# Manual
git log --oneline # Find previous commit
git checkout <commit-hash>
npm run build && deploy
```

### Quick Fix

1. **Fix Issue Locally**
   ```bash
   git checkout main
   # Make fix
   git commit -m "Fix: API connection issue"
   git push origin main
   ```

2. **Redeploy**
   ```bash
   npm run build && npm run deploy
   ```

---

## Monitoring & Observability

### Key Metrics

- **API Response Time**: Target < 5 seconds
- **Page Load Time**: Target < 3 seconds
- **Error Rate**: Target < 1%
- **Availability**: Target 99.9%

### Monitoring Tools

- **Frontend**: Sentry for error tracking
- **Performance**: Google PageSpeed Insights
- **Analytics**: Google Analytics
- **API**: Application Performance Monitoring (APM)

### Setup Error Tracking

```typescript
// In frontend main.tsx
import * as Sentry from "@sentry/react";

Sentry.init({
  dsn: process.env.REACT_APP_SENTRY_DSN,
  environment: process.env.NODE_ENV,
  tracesSampleRate: 1.0,
});
```

---

## Troubleshooting

### ChatBot Not Appearing

**Problem**: ChatBot component not visible

**Solutions**:
1. Check Root.tsx is in `frontend/src/theme/`
2. Verify ChatBot.tsx exists and has no TypeScript errors
3. Check browser console for component load errors
4. Clear browser cache and reload

### API Connection Failed

**Problem**: "Unable to connect" error when submitting query

**Solutions**:
1. Verify `REACT_APP_RAG_AGENT_URL` is correct
2. Check backend is running at that URL
3. Verify CORS is configured on backend
4. Test with curl: `curl https://api.example.com/health`
5. Check firewall/network rules

### Slow Page Load

**Problem**: Docusaurus site takes > 5 seconds to load

**Solutions**:
1. Check bundle size: `npm run build -- --analyze`
2. Enable gzip compression on server
3. Use CDN for static assets
4. Optimize images in docusaurus content
5. Check API response times

### High Error Rate

**Problem**: ChatBot shows errors frequently

**Solutions**:
1. Check API server logs
2. Verify Qdrant database is accessible
3. Check OpenAI API rate limits
4. Review error logs in Sentry
5. Check network connectivity

---

## CI/CD Pipeline

### GitHub Actions Example

```yaml
name: Deploy to Production

on:
  push:
    branches: [ main ]

jobs:
  deploy:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v3

      - uses: actions/setup-node@v3
        with:
          node-version: '20'

      - run: npm install
      - run: npm run typecheck
      - run: npm run test

      - run: npm run build
        env:
          REACT_APP_RAG_AGENT_URL: ${{ secrets.PROD_API_URL }}

      - uses: vercel/action@main
        with:
          vercel-token: ${{ secrets.VERCEL_TOKEN }}
          vercel-org-id: ${{ secrets.VERCEL_ORG_ID }}
          vercel-project-id: ${{ secrets.VERCEL_PROJECT_ID }}
```

---

## Support & Escalation

- **Issues**: Report in GitHub issues or email support@robotics-textbook.org
- **Emergency**: Page on-call engineer (PagerDuty)
- **Escalation**: Contact DevOps team lead

---

## References

- **Specification**: `specs/004-rag-chat-ui/spec.md`
- **Architecture**: `specs/004-rag-chat-ui/plan.md`
- **Integration Guide**: `frontend/docs/CHATBOT_INTEGRATION.md`
- **Configuration**: `frontend/.env.example`

