# Data Model: RAG Chatbot Frontend

**Date**: 2025-12-26
**Feature**: SPEC-4 Integrate RAG Backend with Docusaurus Frontend
**Status**: Phase 1 Design

---

## Overview

The data model defines the structure of chat messages, retrieved chunks, and conversation state. All data is stored client-side in React state; no server persistence is required for MVP.

---

## Core Entities

### 1. ChatMessage

Represents a single query-response pair in the conversation.

**Entity Definition**:

```typescript
interface ChatMessage {
  // Identification
  id: string;                         // UUID, unique per message

  // User Query
  query: string;                      // User's natural language question

  // AI Response
  answer: string | null;              // LLM-generated response (null while loading)

  // Retrieved Context
  retrieved_chunks: RetrievedChunk[]; // Source passages used to generate response

  // Metadata
  timestamp: Date;                    // When message was created
  status: 'pending' | 'success' | 'error';  // Current state
  error?: string;                     // Error message if status='error'

  // Optional: Text selection context
  selected_text?: string;             // User-selected text (if P2: text selection feature)
}
```

**Validation Rules**:

| Field | Rules |
|-------|-------|
| `id` | Must be non-empty UUID |
| `query` | Must be 1-10,000 characters; non-empty after trim |
| `answer` | Max 50,000 characters; can be empty (null while loading) |
| `retrieved_chunks` | Array, max 10 items (backend constraint) |
| `timestamp` | Valid ISO 8601 date |
| `status` | One of: 'pending', 'success', 'error' |
| `error` | Only populated if status='error'; non-empty string |
| `selected_text` | Optional; max 5,000 characters if present |

**Lifecycle**:

```
1. CREATE:
   {
     id: UUID(),
     query: "What is ROS2?",
     answer: null,
     retrieved_chunks: [],
     timestamp: now(),
     status: 'pending'
   }

2. LOADING: (waiting for API response)
   - status = 'pending'
   - answer = null
   - Show spinner in UI

3. SUCCESS: (API returned response)
   {
     ...same as create...
     status: 'success',
     answer: "ROS2 is a flexible middleware...",
     retrieved_chunks: [...]  // Populated from API
   }

4. ERROR: (API failed or timed out)
   {
     ...same as create...
     status: 'error',
     answer: null,
     error: "Request timeout after 30 seconds"
   }

5. END OF LIFE: Message cleared when session ends (page reload or user clears history)
```

**Example Instance**:

```typescript
{
  id: "550e8400-e29b-41d4-a716-446655440000",
  query: "What is ROS2?",
  answer: "ROS2 is a flexible middleware for robotics applications, providing improved real-time performance and security compared to ROS1. It uses a pub/sub architecture for inter-process communication.",
  retrieved_chunks: [
    {
      chunk_id: "chunk_001",
      text: "ROS2 is a flexible middleware...",
      similarity_score: 0.94,
      source_url: "https://robotics-textbook.example.com/module-1-ros2",
      page_title: "Introduction to ROS2",
      section_headers: ["Module 1: ROS2", "Foundations", "What is ROS2?"]
    }
  ],
  timestamp: "2025-12-26T13:47:00.000Z",
  status: 'success'
}
```

---

### 2. RetrievedChunk

Represents a single piece of textbook content retrieved by semantic search.

**Entity Definition**:

```typescript
interface RetrievedChunk {
  // Identification
  chunk_id: string;                    // Unique identifier from Qdrant

  // Content
  text: string;                        // Actual textbook passage (excerpt, not full page)

  // Relevance & Metadata
  similarity_score: number;            // Semantic relevance (0.0 to 1.0)
  source_url: string;                  // URL to textbook page
  page_title: string;                  // Title of the page/chapter
  section_headers: string[];           // Breadcrumb path (e.g., ["Module 1", "Chapter 2", "Section 3"])
}
```

**Validation Rules**:

| Field | Rules |
|-------|-------|
| `chunk_id` | Non-empty string; alphanumeric + underscores |
| `text` | Non-empty string; max 5,000 characters |
| `similarity_score` | Float in range [0.0, 1.0] |
| `source_url` | Valid URL; must start with http:// or https:// |
| `page_title` | Non-empty string; max 200 characters |
| `section_headers` | Array of non-empty strings; max 5 levels deep |

**Source**: This entity is returned directly from the FastAPI `/chat` endpoint (Spec 3). No transformation needed beyond type mapping.

**Example Instance**:

```typescript
{
  chunk_id: "chunk_001",
  text: "ROS2 is a flexible middleware for robotics applications. It provides improved real-time performance and security compared to ROS1 through a pub/sub architecture.",
  similarity_score: 0.94,
  source_url: "https://robotics-textbook.example.com/module-1-ros2/chapter-1-intro",
  page_title: "Introduction to ROS2",
  section_headers: ["Module 1: ROS2 Foundations", "Chapter 1: Overview", "What is ROS2?"]
}
```

---

### 3. ConversationState (React Context/Hook)

Container for all chat messages during the session.

**Entity Definition**:

```typescript
interface ConversationState {
  // Message history
  messages: ChatMessage[];

  // UI state
  isLoading: boolean;                  // True while awaiting API response
  error: string | null;                // Global error message (not message-specific)

  // Configuration
  selectedText?: string;               // Text selected on page (for P2 feature)
}
```

**State Transitions**:

| Trigger | Action | New State |
|---------|--------|-----------|
| User sends query | Add pending message | messages += pending msg |
| API responds | Update message with response | message.status = 'success' |
| API fails | Update message with error | message.status = 'error' |
| User clears history | Remove all messages | messages = [] |
| User selects text | Store selection | selectedText = selected |

---

## API Response Mapping

The frontend receives responses from the FastAPI `/chat` endpoint (Spec 3) with this structure:

```typescript
interface APIResponse {
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
  retrieval_scope: 'full_collection' | 'text_only';
  status: 'success' | 'error';
  error?: {
    code: string;
    message: string;
  };
}
```

**Mapping to ChatMessage**:

```typescript
// APIResponse → ChatMessage
function apiResponseToChatMessage(
  messageId: string,
  apiResponse: APIResponse
): ChatMessage {
  return {
    id: messageId,
    query: apiResponse.query,
    answer: apiResponse.answer,
    retrieved_chunks: apiResponse.retrieved_chunks.map(chunk => ({
      chunk_id: chunk.chunk_id,
      text: chunk.text,
      similarity_score: chunk.similarity_score,
      source_url: chunk.source_url,
      page_title: chunk.page_title,
      section_headers: chunk.section_headers,
    })),
    timestamp: new Date(),
    status: apiResponse.status === 'success' ? 'success' : 'error',
    error: apiResponse.error?.message,
  };
}
```

---

## Relationships

### ChatMessage → RetrievedChunk

- **Cardinality**: One-to-Many (1:N)
- **Direction**: ChatMessage has multiple RetrievedChunk items
- **Constraint**: Each RetrievedChunk belongs to exactly one ChatMessage
- **Lifecycle**: RetrievedChunk is created with ChatMessage; deleted when ChatMessage is deleted

### ConversationState → ChatMessage

- **Cardinality**: One-to-Many (1:N)
- **Direction**: ConversationState contains multiple ChatMessage items
- **Constraint**: Messages are ordered by timestamp (ascending)
- **Lifecycle**: ChatMessage added to ConversationState when created; removed when session ends or user clears history

---

## Storage & Persistence

**MVP Scope**: Session-only (no persistence)

| Data | Storage | Duration | Cleared On |
|------|---------|----------|-----------|
| ChatMessage[] | React state | Current session | Page reload / navigation away / manual clear |
| error message | React state | Current session | Next user action |
| selectedText | React state | Until user deselects | Deselection / click elsewhere |

**No localStorage**: Conversation history not persisted across sessions (simplifies MVP)

**Future Enhancement (Phase 2)**:
- localStorage or IndexedDB for persistence
- Server-side conversation logs (requires backend changes)

---

## Type Definitions (TypeScript)

**File**: `frontend/src/types/chat.ts`

```typescript
/**
 * Types for RAG Chatbot conversation data
 */

// Chat Message Entity
export interface ChatMessage {
  id: string;
  query: string;
  answer: string | null;
  retrieved_chunks: RetrievedChunk[];
  timestamp: Date;
  status: 'pending' | 'success' | 'error';
  error?: string;
  selected_text?: string;
}

// Retrieved Chunk Entity
export interface RetrievedChunk {
  chunk_id: string;
  text: string;
  similarity_score: number;
  source_url: string;
  page_title: string;
  section_headers: string[];
}

// Conversation State
export interface ConversationState {
  messages: ChatMessage[];
  isLoading: boolean;
  error: string | null;
  selectedText?: string;
}

// API Request
export interface ChatRequest {
  query: string;
  retrieval_scope?: 'full_collection' | 'text_only';
  top_k?: number;
  context_text?: string;  // For text_only mode
}

// API Response
export interface ChatResponse {
  query: string;
  answer: string;
  retrieved_chunks: RetrievedChunk[];
  execution_metrics: {
    retrieval_time_ms: number;
    generation_time_ms: number;
    total_time_ms: number;
  };
  retrieval_scope: 'full_collection' | 'text_only';
  status: 'success' | 'error';
  error?: {
    code: string;
    message: string;
  };
}
```

---

## Edge Cases & Validation

### Invalid Inputs

| Case | Validation | Behavior |
|------|-----------|----------|
| Empty query | query.trim().length === 0 | Show error: "Please enter a question" |
| Query > 10k chars | query.length > 10000 | Truncate to 10k chars or show error |
| No network connection | fetch() throws | Show error: "No internet connection" |
| API timeout > 30s | AbortController fires | Show error: "Request timeout" |
| API returns 5xx | HTTP status >= 500 | Show error: "Server error, try again" |
| Malformed JSON response | JSON.parse() throws | Log error, show generic message |
| Empty retrieved_chunks | array.length === 0 | Display message: "No relevant content found" |
| Very long answer | answer.length > 50k | Show in scrollable area; prevent UI overflow |

### State Consistency Rules

1. **Pending State**: If status='pending', answer must be null
2. **Success State**: If status='success', answer must be non-empty
3. **Error State**: If status='error', error field must be non-empty
4. **Chunk Count**: Max 10 retrieved_chunks per message (backend limit)
5. **Timestamp Order**: Messages sorted ascending by timestamp

---

## Future Enhancements (Phase 2+)

### P2: Message Persistence
- Add `localStorage.setItem('chatHistory', JSON.stringify(messages))`
- Reload on app start: `localStorage.getItem('chatHistory')`
- Add "Clear History" button

### P2: Message Editing
- Add `edited_at?: Date` to ChatMessage
- Add `is_edited: boolean` flag
- Show "(edited)" indicator in UI

### P2: Ratings & Feedback
- Add `user_rating?: 1|2|3|4|5` to ChatMessage
- Add `user_feedback?: string` field
- Send to backend for learning

### P3: Multi-turn Conversation Context
- Add `parent_message_id?: string` for conversation threading
- Send conversation history to API for context
- API returns multi-turn aware responses

---

## Summary

- **ChatMessage**: Query-response pair with status and metadata
- **RetrievedChunk**: Source textbook passage with URL and breadcrumb
- **ConversationState**: Container for messages and UI state
- **Storage**: Session-only in React state; no persistence for MVP
- **Validation**: Comprehensive rules prevent invalid state
- **Extensible**: Clear design for Phase 2 persistence and feedback
