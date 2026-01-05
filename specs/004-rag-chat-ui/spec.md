# Feature Specification: Integrate RAG Backend with Docusaurus Frontend

**Feature Branch**: `004-rag-chat-ui`
**Created**: 2025-12-26
**Status**: Draft
**Target Audience**: Full-stack engineers

## Overview

Create an embedded chatbot component for the Docusaurus robotics textbook that allows readers to ask questions about book content. The chatbot queries the FastAPI RAG Agent API (Spec 3) and displays responses inline with the book.

**Key Constraints**:
- Frontend: Docusaurus (React/TypeScript)
- Backend: FastAPI (Spec 3: RAG Agent API)
- Communication: HTTP/REST JSON
- No authentication required
- Works in both local development and production environments

---

## User Scenarios & Testing

### User Story 1 - Ask Question About Book Content (Priority: P1)

**User Journey**: A reader is learning about ROS2 in the robotics textbook. While reading a section, they have a specific question about a concept. They open the chatbot sidebar, type their question, and receive an answer grounded in the textbook content with sources cited.

**Why this priority**: This is the core value proposition - it's the primary interaction users will have with the chatbot. Without this, the feature is incomplete.

**Independent Test**: Can be fully tested by typing a question in the chatbot UI and verifying that:
1. Question is sent to the backend API
2. Response is received and displayed
3. Sources are cited
4. Textbook content is embedded in the response

**Acceptance Scenarios**:

1. **Given** a user is reading a textbook page, **When** they open the chatbot and enter a question like "What is ROS2?", **Then** the system sends the query to the RAG Agent API and displays the response with citations within 5 seconds
2. **Given** the chatbot receives a response, **When** the response is displayed, **Then** source URLs and page titles are visible and clickable
3. **Given** a user submits a query, **When** the API returns a response, **Then** the response text is formatted and readable in the UI (no JSON visible)
4. **Given** a user submits multiple questions, **When** they interact with the chatbot, **Then** each query-response pair is maintained in conversation history within the session

---

### User Story 2 - Scoped Query on Selected Text (Priority: P2)

**User Journey**: While reading about actuators, a reader selects a sentence they don't understand. A context menu appears allowing them to "Ask about this". The chatbot opens with the selected text pre-filled and provides a scoped answer specific to that content.

**Why this priority**: This enhances usability by allowing users to ask about specific parts of the text rather than the entire book. It's a nice-to-have that improves user experience but isn't required for MVP.

**Independent Test**: Can be fully tested by selecting text on a page and verifying that:
1. Text selection triggers context menu
2. "Ask about this" action opens chatbot with selected text
3. Query is scoped to that text in the prompt

**Acceptance Scenarios**:

1. **Given** a user selects text on a page, **When** they right-click or see a context menu, **Then** "Ask about this selection" option appears
2. **Given** the user clicks "Ask about this selection", **When** the chatbot opens, **Then** the selected text is pre-populated in the query field with a message like "Explain this:" or "What does this mean?"
3. **Given** a user sends a scoped question, **When** the API responds, **Then** the response is tailored to explain that specific text (uses selected text as context)

---

### User Story 3 - Configure Backend URL (Priority: P3)

**User Journey**: A developer deploys the textbook to production and needs to configure the chatbot to connect to the production RAG API instead of the local development server. They set an environment variable or configuration file that specifies the backend URL.

**Why this priority**: This is necessary for production deployment but can be done as a configuration step before shipping. It's not a user-facing feature but is required for operational readiness.

**Independent Test**: Can be tested by changing the backend URL configuration and verifying the chatbot connects to the correct API endpoint.

**Acceptance Scenarios**:

1. **Given** the Docusaurus build process, **When** environment variables are read, **Then** the chatbot uses the configured RAG_AGENT_API_URL to connect to the backend
2. **Given** a local development environment, **When** no URL is specified, **Then** the chatbot defaults to `http://localhost:8000/chat`
3. **Given** a production deployment, **When** REACT_APP_RAG_AGENT_URL is set to a production URL, **Then** the chatbot makes requests to that URL instead

---

### Edge Cases

- What happens when the backend API is unreachable? (User sees error message with retry button)
- What happens when a query times out after 30+ seconds? (User sees timeout error, can retry)
- What happens if the selected text is empty? (Context menu doesn't appear or shows disabled state)
- What happens if a user sends an empty query? (Frontend validation prevents submission, shows error)
- What happens if the response is longer than can fit on screen? (Response is scrollable or paginated)
- What happens on mobile devices with small screens? (Chatbot UI adapts - sidebar becomes modal or bottom sheet)

---

## Requirements

### Functional Requirements

- **FR-001**: System MUST accept user queries via a text input field in the chatbot UI
- **FR-002**: System MUST send queries to the FastAPI RAG Agent API's `/chat` endpoint via HTTP POST
- **FR-003**: System MUST parse JSON responses from the API and extract the `answer` field
- **FR-004**: System MUST display retrieved sources (source_url and page_title) with responses
- **FR-005**: System MUST support making sources clickable to navigate to book pages or external links
- **FR-006**: System MUST maintain conversation history during the user's session (list of queries and responses)
- **FR-007**: System MUST validate that queries are non-empty before sending to the API
- **FR-008**: System MUST display error messages when API requests fail (timeouts, connection errors, HTTP errors)
- **FR-009**: System MUST capture selected text on the page when user initiates a scoped query
- **FR-010**: System MUST pre-populate the chatbot query field with selected text when a scoped query is initiated
- **FR-011**: System MUST read the RAG agent API URL from environment variables during build time
- **FR-012**: System MUST default to `http://localhost:8000/chat` when no environment variable is set
- **FR-013**: System MUST send requests with `Content-Type: application/json`
- **FR-014**: System MUST handle API responses with the structure: `{ query, answer, retrieved_chunks, execution_metrics, status, error }`
- **FR-015**: System MUST display "loading" state while waiting for API response (spinner or skeleton)

### Key Entities

- **ChatMessage**: Represents a single query-response pair
  - `id`: Unique identifier
  - `query`: User's question text
  - `answer`: AI-generated response
  - `retrieved_chunks`: Array of source references
  - `timestamp`: When the message was created
  - `error`: Optional error information if request failed

- **RetrievedChunk**: Source reference metadata
  - `chunk_id`: Unique identifier
  - `text`: Content excerpt
  - `similarity_score`: Relevance score (0.0-1.0)
  - `source_url`: URL to source page
  - `page_title`: Title of page
  - `section_headers`: Breadcrumb path in document

---

## Success Criteria

### Measurable Outcomes

- **SC-001**: Users can successfully ask a question and receive a response from the chatbot within 5 seconds of submission (includes API latency)
- **SC-002**: 95% of API requests from the frontend succeed (status code 200)
- **SC-003**: Chatbot UI is accessible and usable on desktop browsers (Chrome, Firefox, Safari, Edge)
- **SC-004**: Selected text capture works on 90% of common text selection scenarios
- **SC-005**: Backend URL configuration works for both local (`http://localhost:8000`) and production (`https://api.example.com`) URLs
- **SC-006**: Conversation history persists for the duration of a page session (survives page navigation within Docusaurus, cleared on full page reload)
- **SC-007**: Error messages appear within 2 seconds of an API failure
- **SC-008**: At least 80% of test users can find and use the chatbot without instruction

### Deployment Readiness

- **SC-009**: Chatbot component integrates without breaking existing Docusaurus site functionality
- **SC-010**: Build time does not increase by more than 10% with new chatbot component included
- **SC-011**: Chatbot works with Docusaurus in both development (`docusaurus start`) and production (`docusaurus build`) modes

---

## Assumptions

1. **API Availability**: The RAG Agent API (Spec 3) is deployed and accessible at the configured URL
2. **Response Format**: API responses follow the structure defined in Spec 3 (AGENT_API.md)
3. **No Authentication**: Chatbot calls to the API don't require authentication (no JWT, API keys, etc.)
4. **Session-Based Storage**: Conversation history only needs to persist during the current session (no database needed)
5. **Synchronous Requests**: All API calls are synchronous (await/Promise-based) and don't require WebSocket
6. **Browser Environment**: Runs in modern browsers with ES2020+ support and fetch API
7. **Text Selection**: Browser provides standard text selection API (window.getSelection)

---

## Out of Scope

- User authentication or multi-user conversation history
- Persistent storage of conversations across sessions
- Advanced UI styling or custom design system (uses Docusaurus defaults)
- Analytics or usage tracking
- Rate limiting on the frontend (backend handles this)
- Voice input/output
- Multi-language support beyond English
- Accessibility features beyond WCAG 2.1 AA standards

---

## Questions for Clarification

The following aspects have reasonable defaults but can be clarified if needed:

1. **Text Selection Behavior**: Should selecting text on a page show a context menu, or should there be a separate button? (Default: Context menu or popover near selection)
2. **Mobile Experience**: Should the chatbot be a sidebar (desktop) or modal/bottom-sheet (mobile)? (Default: Responsive - sidebar on desktop, bottom sheet on mobile)
3. **Conversation Persistence**: Should conversations be saved to localStorage so returning users see history? (Default: Session-only, no localStorage)

---

## Related Work

- **Spec 1**: Website Ingestion Pipeline (provides textbook content)
- **Spec 2**: RAG Retrieval Validation (validates semantic search)
- **Spec 3**: RAG Agent API (provides the `/chat` endpoint this feature calls)
- **Frontend**: Docusaurus robotics textbook site

---

## Success Definition

This feature is **COMPLETE** when:

1. ✅ Users can ask questions via chatbot UI and receive responses from RAG API
2. ✅ Responses include source citations (URLs and page titles)
3. ✅ Conversation history is maintained during session
4. ✅ Error handling provides meaningful feedback
5. ✅ Backend URL is configurable via environment variables
6. ✅ Text selection capture works (P2 feature)
7. ✅ Component integrates with Docusaurus without breaking existing functionality
8. ✅ Works in both development and production environments
