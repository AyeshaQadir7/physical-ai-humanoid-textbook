# Research & Design Findings: RAG Chatbot Frontend Integration

**Date**: 2025-12-26
**Feature**: SPEC-4 Integrate RAG Backend with Docusaurus Frontend
**Status**: Phase 0 Complete - All design decisions resolved

---

## Research Summary

This document captures findings from Phase 0 research that resolved open questions and established best practices for frontend-backend integration, HTTP communication, state management, and mobile UX patterns.

---

## 1. CORS Configuration for FastAPI + Docusaurus

### Decision
**Use FastAPI's CORSMiddleware with environment-based configuration**

### Implementation Details

```python
# backend/agent.py - CORS setup for Docusaurus frontend
from fastapi.middleware.cors import CORSMiddleware

# Read allowed origins from environment
CORS_ORIGINS = os.getenv("CORS_ORIGINS", "http://localhost:3000").split(",")

app.add_middleware(
    CORSMiddleware,
    allow_origins=CORS_ORIGINS,
    allow_credentials=False,  # No auth, so false is fine
    allow_methods=["GET", "POST", "OPTIONS"],
    allow_headers=["Content-Type"],
)
```

### Configuration by Environment

**Development (.env.local)**:
```env
CORS_ORIGINS=http://localhost:3000
```

**Production (.env.production)**:
```env
CORS_ORIGINS=https://robotics-textbook.example.com
```

### Why This Approach

| Option | Pros | Cons | Selected? |
|--------|------|------|-----------|
| `allow_origins=["*"]` | Simple, works anywhere | Insecure for production | ❌ No |
| Environment-based list | Secure, flexible, specific | Requires config management | ✅ **Yes** |
| Wildcard with allowlist | Flexible but explicit | Complex regex | ❌ Overkill |

### Testing

```bash
# Development: from http://localhost:3000
curl -H "Origin: http://localhost:3000" http://localhost:8000/health
# Expected: 200 OK with CORS headers

# Production: from https://robotics-textbook.example.com
curl -H "Origin: https://robotics-textbook.example.com" https://api.example.com/health
# Expected: 200 OK with CORS headers
```

### Related Issue

**Spec 3 (RAG Agent API)** currently has no CORS configuration. This research identifies that **CORS must be added to Spec 3** before Spec 4 deployment.

**Action**: Add CORS setup to Spec 3 backend code (separate task in implementation phase)

---

## 2. HTTP Client Library: Fetch API vs. Axios

### Decision
**Use native Fetch API + React hooks; defer axios to Phase 2 if needed**

### Rationale

| Criteria | Fetch API | Axios |
|----------|-----------|-------|
| **Bundle Size** | 0 KB (native) | ~12 KB | ✅ **Fetch wins** |
| **Dependencies** | None | 1 additional | ✅ **Fetch wins** |
| **Error Handling** | Requires explicit logic | Built-in | ❌ Axios better |
| **Timeout Support** | AbortController | Built-in | ❌ Axios better |
| **Request Cancellation** | AbortController | Built-in | ❌ Axios better |
| **Standard** | Web standard | Community lib | ✅ **Fetch wins** |
| **Learning Curve** | Low | Low | 🟰 Tie |

### Implementation: Custom Hook Pattern

```typescript
// frontend/src/services/useChat.ts
import { useState, useCallback } from 'react';

interface ChatRequest {
  query: string;
  top_k?: number;
  retrieval_scope?: 'full_collection' | 'text_only';
}

interface ChatResponse {
  query: string;
  answer: string;
  retrieved_chunks: Array<{
    chunk_id: string;
    text: string;
    similarity_score: number;
    source_url: string;
    page_title: string;
    section_headers: string[];
  }>;
  execution_metrics: {
    retrieval_time_ms: number;
    generation_time_ms: number;
    total_time_ms: number;
  };
  status: 'success' | 'error';
  error?: {
    code: string;
    message: string;
  };
}

export function useChat() {
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const sendQuery = useCallback(
    async (request: ChatRequest, timeout = 30000): Promise<ChatResponse | null> => {
      setLoading(true);
      setError(null);

      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), timeout);

      try {
        const apiUrl = process.env.REACT_APP_RAG_AGENT_URL || 'http://localhost:8000/chat';

        const response = await fetch(apiUrl, {
          method: 'POST',
          headers: {
            'Content-Type': 'application/json',
          },
          body: JSON.stringify(request),
          signal: controller.signal,
        });

        if (!response.ok) {
          const errorData = await response.json();
          throw new Error(errorData.error?.message || `HTTP ${response.status}`);
        }

        const data: ChatResponse = await response.json();
        return data;
      } catch (err) {
        const message = err instanceof Error ? err.message : 'Unknown error';
        setError(message);
        return null;
      } finally {
        clearTimeout(timeoutId);
        setLoading(false);
      }
    },
    []
  );

  return { sendQuery, loading, error };
}
```

### When to Upgrade to Axios (Phase 2)

- If implementing request cancellation for multiple concurrent queries
- If adding request/response interceptors (logging, retry logic)
- If building SDKs for third-party integrations

**Decision**: MVP uses Fetch + hooks. Revisit in Phase 2 if complexity increases.

---

## 3. Conversation State Management

### Decision
**Use React.useState + Context API for MVP; optional useReducer for Phase 2**

### State Structure

```typescript
// frontend/src/types/chat.ts
interface ChatMessage {
  id: string;                    // UUID
  query: string;                 // User question
  answer: string;                // AI response
  chunks: RetrievedChunk[];      // Source references
  timestamp: Date;               // When sent
  status: 'pending' | 'success' | 'error';
  error?: string;                // Error message if failed
}

interface RetrievedChunk {
  chunk_id: string;
  text: string;
  similarity_score: number;
  source_url: string;
  page_title: string;
  section_headers: string[];
}
```

### MVP Implementation (useState)

```typescript
// ChatBot.tsx
import { useState } from 'react';
import { ChatMessage, RetrievedChunk } from '../types/chat';
import { useChat } from '../services/useChat';

export function ChatBot() {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const { sendQuery, loading } = useChat();

  const handleSubmit = async (query: string) => {
    const messageId = crypto.randomUUID();

    // Add pending message
    setMessages(prev => [...prev, {
      id: messageId,
      query,
      answer: '',
      chunks: [],
      timestamp: new Date(),
      status: 'pending',
    }]);

    // Send query
    const response = await sendQuery({ query, top_k: 5 });

    // Update message with response
    if (response) {
      setMessages(prev =>
        prev.map(msg =>
          msg.id === messageId
            ? {
                ...msg,
                answer: response.answer,
                chunks: response.retrieved_chunks,
                status: 'success',
              }
            : msg
        )
      );
    } else {
      setMessages(prev =>
        prev.map(msg =>
          msg.id === messageId
            ? { ...msg, status: 'error', error: 'Failed to get response' }
            : msg
        )
      );
    }
  };

  return (
    <div className="chatbot">
      <div className="messages">
        {messages.map(msg => (
          <ChatBubble key={msg.id} message={msg} />
        ))}
      </div>
      <ChatInput onSubmit={handleSubmit} disabled={loading} />
    </div>
  );
}
```

### Phase 2: Context + useReducer (if needed)

If conversation state becomes complex (undo/redo, message editing, multi-turn logic):

```typescript
// NOT for MVP, but documented for future:
type MessageAction =
  | { type: 'ADD_MESSAGE'; payload: ChatMessage }
  | { type: 'UPDATE_MESSAGE'; payload: { id: string; updates: Partial<ChatMessage> } }
  | { type: 'DELETE_MESSAGE'; payload: string }
  | { type: 'CLEAR_HISTORY' };

function messageReducer(state: ChatMessage[], action: MessageAction) {
  switch (action.type) {
    case 'ADD_MESSAGE':
      return [...state, action.payload];
    case 'UPDATE_MESSAGE':
      return state.map(msg =>
        msg.id === action.payload.id ? { ...msg, ...action.payload.updates } : msg
      );
    case 'DELETE_MESSAGE':
      return state.filter(msg => msg.id !== action.payload);
    case 'CLEAR_HISTORY':
      return [];
  }
}
```

**Decision**: MVP uses simple useState. If conversation management becomes complex in Phase 2, refactor to Context + useReducer.

---

## 4. Text Selection UX: Context Menu vs. Floating Toolbar

### Decision
**Use context menu (simpler); defer floating toolbar to Phase 2 if needed**

### MVP Approach: Context Menu

```typescript
// frontend/src/hooks/useTextSelection.ts
import { useEffect, useState } from 'react';

interface SelectionEvent {
  text: string;
  x: number;  // Mouse X coordinate
  y: number;  // Mouse Y coordinate
}

export function useTextSelection(
  ref: React.RefObject<HTMLDivElement>,
  minLength = 5
) {
  const [selection, setSelection] = useState<SelectionEvent | null>(null);

  useEffect(() => {
    const handleMouseUp = () => {
      const selected = window.getSelection();
      if (!selected || !selected.toString().trim()) {
        setSelection(null);
        return;
      }

      const text = selected.toString().trim();
      if (text.length < minLength) {
        setSelection(null);
        return;
      }

      // Only show if selection is within our component
      const range = selected.getRangeAt(0);
      if (!ref.current?.contains(range.commonAncestorContainer)) {
        setSelection(null);
        return;
      }

      // Get mouse position
      const event = new MouseEvent('mouseup');
      setSelection({
        text,
        x: (event as any).clientX || 0,
        y: (event as any).clientY || 0,
      });
    };

    document.addEventListener('mouseup', handleMouseUp);
    return () => document.removeEventListener('mouseup', handleMouseUp);
  }, [ref]);

  const clearSelection = () => setSelection(null);

  return { selection, clearSelection };
}
```

### UI Implementation

```tsx
// ChatBot.tsx - with text selection context menu
const { selection, clearSelection } = useTextSelection(contentRef);

{selection && (
  <ContextMenu x={selection.x} y={selection.y}>
    <button onClick={() => {
      handleQueryWithContext(selection.text);
      clearSelection();
    }}>
      Ask about this selection
    </button>
  </ContextMenu>
)}
```

### Why Context Menu for MVP

| Approach | Simplicity | UX Quality | Implementation Time | Selected? |
|----------|-----------|-----------|---------------------|-----------|
| Context Menu | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | 2 hours | ✅ **Yes** |
| Floating Toolbar | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | 8 hours | ❌ Phase 2 |
| Tooltip + Click | ⭐⭐ | ⭐⭐ | 4 hours | ❌ Complex |

### Phase 2: Enhanced Text Selection

If P2 roadmap includes UX improvement:
- Floating toolbar that appears on selection (appears automatically, no right-click)
- Smooth animations
- More action options (copy, highlight, share)
- Uses library like `Popper.js` or `Floating UI`

**Decision**: MVP uses simple context menu. Upgrade in Phase 2 if user feedback shows demand for floating toolbar.

---

## 5. Mobile Experience: Bottom Sheet vs. Sidebar

### Decision
**Responsive component with breakpoints: sidebar (desktop) → bottom sheet (mobile)**

### Responsive Strategy

```typescript
// frontend/src/components/ChatBot/ChatBot.tsx
import { useWindowSize } from '../hooks/useWindowSize';

export function ChatBot() {
  const { width } = useWindowSize();
  const isMobile = width < 768;  // Tailwind's md breakpoint

  return (
    <div className={isMobile ? 'chatbot-mobile' : 'chatbot-desktop'}>
      {/* Conditionally render as modal (mobile) or sidebar (desktop) */}
      {isMobile ? <ChatBotMobile /> : <ChatBotSidebar />}
    </div>
  );
}

// Desktop: Fixed sidebar
const ChatBotSidebar = () => (
  <aside className="fixed right-0 top-0 w-96 h-screen bg-white border-l shadow-lg">
    {/* Chat UI */}
  </aside>
);

// Mobile: Slide-up modal (via CSS transform)
const ChatBotMobile = () => (
  <div className="chatbot-modal-backdrop">
    <div className="chatbot-modal-sheet">
      {/* Chat UI - drag to dismiss, full width */}
    </div>
  </div>
);
```

### CSS Implementation (Tailwind-friendly)

```css
/* Desktop: Sidebar */
@media (min-width: 768px) {
  .chatbot-desktop {
    position: fixed;
    right: 0;
    width: 384px;  /* 96 in Tailwind units */
    height: 100vh;
  }
}

/* Mobile: Bottom sheet */
@media (max-width: 767px) {
  .chatbot-mobile {
    position: fixed;
    bottom: 0;
    width: 100%;
    height: 80vh;  /* Leaves room for user to see page content */
    border-radius: 16px 16px 0 0;
    animation: slideUp 0.3s ease-out;
  }

  @keyframes slideUp {
    from {
      transform: translateY(100%);
      opacity: 0;
    }
    to {
      transform: translateY(0);
      opacity: 1;
    }
  }
}
```

### Why No External Library

| Option | Pros | Cons | Selected? |
|--------|------|------|-----------|
| Custom CSS | Lightweight, full control | Requires CSS skills | ✅ **Yes** |
| Headless UI | Accessible, battle-tested | Adds dependency | ❌ P2 option |
| Radix UI | Excellent a11y, flexible | ~50 KB | ❌ P2 option |

**Decision**: MVP uses custom CSS with Tailwind utilities. In Phase 2, if mobile UX needs enhancement (drag-to-dismiss, snap points), consider Headless UI or Radix Dialog.

---

## Summary of Technical Decisions

| Decision | Choice | Rationale | Risk Level |
|----------|--------|-----------|-----------|
| **CORS** | Environment-based allowlist | Secure, flexible | Low |
| **HTTP Client** | Fetch API + hooks | No deps, standard, simple | Low |
| **State Management** | useState | MVP simplicity | Low; Phase 2 ready |
| **Text Selection** | Context menu | Quick to implement | Low |
| **Mobile UX** | Responsive (sidebar→sheet) | CSS-only, no deps | Low |

---

## Dependencies & Prerequisites

1. **Spec 3 (RAG Agent API)** must be deployed and accessible
   - Must have `/chat` endpoint responding to POST requests
   - Must have CORS configured for Docusaurus domain
   - Status: Spec 3 deployed ✅

2. **Docusaurus v2+** with React 18+
   - Status: Docusaurus bootstrapped ✅

3. **Modern browsers** with:
   - Fetch API
   - window.getSelection()
   - CSS Grid/Flexbox
   - ES2020+ support
   - Status: Standard in 2023+ browsers ✅

---

## Next Steps

1. **Phase 1**: Create `data-model.md`, API contracts, and `quickstart.md`
2. **Phase 2 (Tasks)**: Execute `/sp.tasks` to generate executable work items
3. **Implementation**: Follow task breakdown for MVP development

---

## Appendix: Browser Compatibility

| Browser | Version | Support |
|---------|---------|---------|
| Chrome | 80+ | ✅ Full |
| Firefox | 75+ | ✅ Full |
| Safari | 13+ | ✅ Full |
| Edge | 80+ | ✅ Full |
| IE 11 | - | ❌ Unsupported |

All modern browsers have Fetch API, getSelection(), and ES2020+ support. No polyfills needed.
