# ChatBot Integration Guide

**Date**: 2025-12-26
**Feature**: SPEC-4 RAG Chatbot Integration with Docusaurus
**Status**: Complete
**Task**: T014

---

## Overview

The ChatBot has been integrated into the Docusaurus frontend as a global UI overlay that appears on every page of the textbook. Users can ask questions about the book content at any time, from any page.

---

## How It Works

### Integration Method: Root Theme Component

Docusaurus v3 provides a `/src/theme/Root.tsx` component that wraps the entire site. This is the standard extension point for adding global UI elements.

**File**: `frontend/src/theme/Root.tsx`

```typescript
export default function Root({ children }: RootProps): React.ReactElement {
  return (
    <>
      {/* Site content from Docusaurus */}
      {children}

      {/* ChatBot overlay - appears on top of all pages */}
      <ChatBot />
    </>
  );
}
```

### What This Means

- ✅ ChatBot appears on **every page** of the textbook
- ✅ ChatBot is **persistent** across navigation
- ✅ ChatBot **conversation history** persists during the session
- ✅ ChatBot **doesn't break** existing Docusaurus functionality
- ✅ ChatBot UI is **responsive**: sidebar (desktop) or bottom sheet (mobile)

---

## User Experience

### Desktop (> 768px)
- ChatBot appears as a **fixed sidebar on the right edge**
- Width: 384px (Tailwind's w-96)
- Height: Up to 600px
- Always visible, doesn't scroll with page content
- User can read book content while chatting

### Mobile (< 768px)
- ChatBot appears as a **full-width bottom sheet modal**
- Height: 80vh (leaves some page content visible above)
- Slides up from bottom with smooth animation
- User can still see parts of the page while chatting
- Closes when user taps outside or uses dismiss button

---

## Component Hierarchy

```
Docusaurus Site
  ↓
Root.tsx (Theme wrapper)
  ├── Site Content ({children})
  │   ├── Navbar
  │   ├── Sidebar
  │   └── Page Content
  └── ChatBot Component (Global overlay)
      ├── ChatBot Header
      ├── Messages Container
      ├── Input Form
      └── CSS: Fixed positioning on desktop/mobile
```

---

## Testing the Integration

### Manual Testing Steps

1. **Start Development Server**
   ```bash
   cd frontend
   npm run start
   ```

2. **Verify ChatBot Appears**
   - Open http://localhost:3000 in browser
   - Look for "Textbook Assistant" header in bottom-right (desktop) or bottom (mobile)
   - ChatBot should be visible on every page

3. **Test Desktop Layout**
   - Resize browser to > 768px width
   - ChatBot should appear as fixed sidebar on right
   - Should not scroll with page content
   - Should not overlap with book content

4. **Test Mobile Layout**
   - Resize browser to < 768px width
   - ChatBot should appear as full-width bottom sheet
   - Should be dismissible

5. **Test Functionality**
   - Type a question: "What is ROS2?"
   - ChatBot should show loading spinner
   - Note: **API call will fail** if FastAPI backend is not running
   - This is expected for frontend testing only

6. **Test Persistence**
   - Send a question
   - Click a link to navigate to another page
   - Conversation history should still be visible
   - ChatBot state should persist across navigation

7. **Test Error Handling**
   - Backend offline → Should show "Unable to connect" error
   - Timeout → Should show "Request took too long" error
   - Validation error → Should show "Please enter a question" error

---

## Backend Integration

For **end-to-end testing**, you need the FastAPI RAG Agent API running:

### Prerequisites

1. **Backend Running**
   ```bash
   cd backend
   python -m uvicorn ingestion.agent:app --reload --port 8000
   ```

2. **CORS Configured**
   - FastAPI must have CORS middleware configured
   - Must allow requests from `http://localhost:3000` (frontend)
   - See: `specs/004-rag-chat-ui/research.md` for CORS configuration

3. **Qdrant Vector DB**
   - Must be running with textbook content indexed
   - Spec 1 (Website Ingestion) handles this

4. **OpenAI API Key**
   - Spec 3 (RAG Agent API) requires OPENAI_API_KEY
   - Set in environment before running backend

### Testing Backend Integration

1. **Verify Backend is Accessible**
   ```bash
   curl http://localhost:8000/health
   # Should return: {"status":"ok"}
   ```

2. **Send a Chat Request**
   ```bash
   curl -X POST http://localhost:8000/chat \
     -H "Content-Type: application/json" \
     -d '{"query": "What is ROS2?", "retrieval_scope": "full_collection", "top_k": 5}'
   ```

3. **Test Frontend-Backend Integration**
   - Open frontend at http://localhost:3000
   - Type question in chatbot
   - Should receive response with sources
   - Should display sources as clickable links

---

## Configuration

### Environment Variables

The ChatBot uses the `REACT_APP_RAG_AGENT_URL` environment variable to determine the backend API endpoint.

**Development** (`.env.local`):
```env
REACT_APP_RAG_AGENT_URL=http://localhost:8000/chat
```

**Production** (Set via CI/CD):
```env
REACT_APP_RAG_AGENT_URL=https://api.robotics-textbook.example.com/chat
```

**Default** (If not set):
```
http://localhost:8000/chat
```

See: `frontend/docs/RAG_CONFIGURATION.md` for detailed configuration guide.

---

## File Structure

```
frontend/
├── src/
│   ├── theme/
│   │   └── Root.tsx                 ← ENTRY POINT (NEW)
│   ├── components/ChatBot/
│   │   ├── ChatBot.tsx              (Main component)
│   │   ├── ChatMessage.tsx          (Display)
│   │   ├── SourceCitation.tsx       (Display)
│   │   ├── LoadingSpinner.tsx       (UI)
│   │   ├── ErrorDisplay.tsx         (UI)
│   │   ├── useChat.ts               (Hook)
│   │   ├── useConversation.ts       (Hook)
│   │   └── ChatBot.module.css       (Styles)
│   ├── services/
│   │   └── ragClient.ts             (HTTP client)
│   ├── config/
│   │   └── ragConfig.ts             (Configuration)
│   └── types/
│       └── chat.ts                  (Types)
└── docusaurus.config.ts             (Build config)
```

---

## Known Limitations

### Current (MVP - Phase 3)

| Limitation | Impact | Workaround | Phase |
|-----------|--------|-----------|-------|
| No text selection | Can't ask about highlighted text | Ask full question manually | P2 |
| Session-only history | Reloading page clears chat | Copy important Q&A before reload | P2 |
| No production URL config | Must rebuild for different backend | Set REACT_APP_RAG_AGENT_URL in CI/CD | P5 |
| No unit tests | Limited regression testing | Manual testing before release | P3+ |

---

## Next Steps

### Before Production Release

1. **T015: Unit Tests**
   - Test ChatBot component submission
   - Test message display with sources
   - Test error handling

2. **T016: Integration Tests**
   - End-to-end: submit query → get response → display
   - Error scenarios: network, timeout, validation
   - Retry logic verification

3. **Manual Testing Checklist**
   - [ ] Test on desktop Chrome, Firefox, Safari
   - [ ] Test on mobile iPhone, Android
   - [ ] Test with FastAPI backend running
   - [ ] Test error scenarios
   - [ ] Test conversation persistence
   - [ ] Verify no breaking changes to existing site

### Phase 4 (P2 Features)

- T017-T024: Text selection feature for scoped queries

### Phase 5 (P3 Features)

- T025-T028: Production URL configuration and deployment

---

## Troubleshooting

### ChatBot Not Visible

**Problem**: ChatBot doesn't appear on page

**Solutions**:
1. Check TypeScript compilation: `npm run typecheck`
2. Check browser console for errors: Open DevTools → Console
3. Verify Root.tsx exists: `ls src/theme/Root.tsx`
4. Restart dev server: `npm run start`

### API Requests Fail

**Problem**: ChatBot shows "Unable to connect" error

**Solutions**:
1. Start backend: `python -m uvicorn ingestion.agent:app --port 8000`
2. Check backend is accessible: `curl http://localhost:8000/health`
3. Verify CORS is configured on backend
4. Check browser console for CORS errors
5. Verify REACT_APP_RAG_AGENT_URL is correct

### TypeScript Errors

**Problem**: `npm run typecheck` fails

**Solutions**:
1. Clear cache: `rm -rf node_modules tsconfig.tsbuildinfo`
2. Reinstall: `npm install`
3. Check for syntax errors in recent changes
4. Run typecheck again: `npm run typecheck`

---

## References

- **Component Guide**: `frontend/src/components/ChatBot/README.md` (if created)
- **Configuration Guide**: `frontend/docs/RAG_CONFIGURATION.md`
- **API Contract**: `specs/004-rag-chat-ui/contracts/chatbot-api.openapi.yaml`
- **Data Model**: `specs/004-rag-chat-ui/data-model.md`
- **Implementation Progress**: `specs/004-rag-chat-ui/IMPLEMENTATION_PROGRESS.md`

---

## Summary

✅ **ChatBot is now fully integrated** into the Docusaurus frontend
✅ **Appears on every page** of the textbook
✅ **Responsive design** for desktop and mobile
✅ **Ready for end-to-end testing** with FastAPI backend
✅ **Type-safe** with full TypeScript compilation

Next: Run manual tests with FastAPI backend, then proceed to Phase 4-6.
