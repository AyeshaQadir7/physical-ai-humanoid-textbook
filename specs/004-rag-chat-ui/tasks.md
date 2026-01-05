# Implementation Tasks: Integrate RAG Backend with Docusaurus Frontend

**Feature**: SPEC-4 Integrate RAG Backend with Docusaurus Frontend
**Branch**: `004-rag-chat-ui`
**Date**: 2025-12-26
**Status**: Ready for Implementation
**Target Audience**: Full-stack engineers

---

## Summary

Executable task breakdown for building a chatbot React component in Docusaurus that queries the FastAPI RAG Agent API. Tasks are organized by user story with clear dependencies and parallel execution opportunities.

**MVP Scope**: User Story 1 (P1) - Core chatbot functionality for asking questions about book content.

**Total Tasks**: 31
**Setup & Foundational**: 5 tasks
**User Story 1 (P1)**: 11 tasks
**User Story 2 (P2)**: 8 tasks
**User Story 3 (P3)**: 4 tasks
**Polish & Cross-Cutting**: 3 tasks

---

## Implementation Strategy

### Phase Execution Order

1. **Phase 1 - Setup** (2 tasks)
   - Create project structure and TypeScript types
   - Install dependencies (if needed beyond Docusaurus)

2. **Phase 2 - Foundational** (3 tasks)
   - API client service layer (RAG communication)
   - Configuration and environment setup
   - Test infrastructure

3. **Phase 3 - User Story 1 (P1)** (11 tasks)
   - Core chatbot component: UI, state management, API integration
   - Message display and source citation
   - Error handling and loading states
   - MVP validation

4. **Phase 4 - User Story 2 (P2)** (8 tasks)
   - Text selection detection hook
   - Context menu or popover component
   - Integration with chatbot for scoped queries

5. **Phase 5 - User Story 3 (P3)** (4 tasks)
   - Environment variable configuration
   - Configuration documentation
   - Production URL validation

6. **Phase 6 - Polish & Cross-Cutting** (3 tasks)
   - UI refinement and responsive design
   - Comprehensive integration testing
   - Documentation completion

### Parallel Opportunities

**In Phase 3 (US1)**: T016, T017, T018, T019 can run in parallel (different files, no dependencies)
**In Phase 4 (US2)**: T025, T026 can run in parallel (independent hooks)
**In Phase 6**: T029, T030 can run in parallel (different concerns)

### Independent Testing Strategy

**User Story 1 (P1)** is independently testable:
- Can query with hardcoded API URL
- Can validate response display without text selection feature
- Can test conversation history independently
- MVP can ship with just US1 completed

**User Story 2 (P2)** is independently testable:
- Requires US1 ChatBot component to exist
- Text selection feature can be tested in isolation
- Can defer to Phase 2 without blocking US1

**User Story 3 (P3)** is independently testable:
- Configuration can be validated by integration tests
- Environment variables can be mocked in tests
- Can be implemented independently of US1/US2 UX

---

## Phase 1: Setup

Project initialization and type definitions.

### Goal
Establish project structure, TypeScript types, and configuration baseline for chatbot development.

### Test Criteria
- [ ] TypeScript types compile without errors
- [ ] Frontend build succeeds
- [ ] Type checker reports no errors in new files

### Tasks

- [x] T001 Create TypeScript types file at `frontend/src/types/chat.ts` with ChatMessage, RetrievedChunk, ConversationState, ChatRequest, ChatResponse interfaces per data-model.md

- [x] T002 Create Docusaurus configuration update plan: document where RAG_AGENT_URL will be injected in `frontend/docusaurus.config.ts` (to be completed in Phase 2)

---

## Phase 2: Foundational

Blocking prerequisites that all user stories depend on.

### Goal
Build API communication layer, configuration system, and test infrastructure required by all features.

### Test Criteria
- [ ] RAG API client makes requests with correct schema
- [ ] Configuration loads from environment variables
- [ ] Test framework and mocks are functional

### Tasks

- [x] T003 [P] Create RAG API HTTP client service at `frontend/src/services/ragClient.ts` with `sendQuery(request: ChatRequest): Promise<ChatResponse>` using Fetch API, including timeout (30s) via AbortController, error handling for 400/429/500/503 status codes

- [x] T004 [P] Create environment configuration module at `frontend/src/config/ragConfig.ts` that reads REACT_APP_RAG_AGENT_URL from build-time environment, defaults to `http://localhost:8000/chat` if not set, and exports configured API endpoint URL

- [x] T005 Create test mocks and utilities at `frontend/src/__tests__/mocks/ragApi.mock.ts` with mock ChatResponse factory and MSW handlers for /chat endpoint

---

## Phase 3: User Story 1 - Ask Question About Book Content (P1)

Core chatbot functionality: ask questions, receive responses, view sources.

### Goal
Build minimum viable chatbot component that accepts queries, calls RAG API, displays responses with source citations, and maintains conversation history during the session.

### Independent Test Criteria (US1)
- [ ] User can type a question and submit it
- [ ] Question is sent to the backend /chat endpoint with correct request format
- [ ] API response is received and parsed correctly
- [ ] Answer text is displayed in the UI
- [ ] Retrieved chunks (sources) are displayed with URL and page title clickable
- [ ] Multiple questions in a session are tracked in conversation history
- [ ] Loading spinner shows while waiting for API response
- [ ] Error messages appear when API request fails
- [ ] Empty query validation prevents submission

### Tasks

- [x] T006 [US1] Create main ChatBot component at `frontend/src/components/ChatBot/ChatBot.tsx` with:
  - State: messages[], loading state, error state
  - Input field for user query with validation (non-empty, max 10k chars)
  - Submit button with loading disabled state
  - Message list display area
  - Error message display area with retry capability

- [x] T007 [P] [US1] Create useChat hook at `frontend/src/components/ChatBot/useChat.ts` that:
  - Manages API communication state (loading, error)
  - Calls ragClient.sendQuery() with proper error handling
  - Returns { sendQuery, loading, error } interface
  - Implements 30s timeout abort logic
  - Handles 400/429/500/503 status codes with user-friendly messages

- [x] T008 [P] [US1] Create ChatMessage display component at `frontend/src/components/ChatBot/ChatMessage.tsx` that:
  - Displays user query in one style (right-aligned or highlighted)
  - Displays AI answer in another style (left-aligned, different background)
  - Shows loading spinner while answer is pending
  - Renders retrieved_chunks[] as clickable source links
  - Each chunk shows: text snippet, similarity_score, page_title, source_url (clickable)

- [x] T009 [P] [US1] Create RetrievedChunk display component at `frontend/src/components/ChatBot/SourceCitation.tsx` showing:
  - Source text (excerpt with max 200 chars, truncated if longer)
  - Similarity score as visual indicator (percentage or colored bar)
  - Page title and source URL as clickable link
  - Section breadcrumb path (section_headers[])

- [x] T010 [US1] Create chat state management hook at `frontend/src/components/ChatBot/useConversation.ts` that:
  - Maintains messages[] array with type ChatMessage
  - Implements add message, update message status, clear history methods
  - Preserves session state across navigation
  - Clears on page reload or manual clear

- [x] T011 [P] [US1] Create ChatBot CSS module at `frontend/src/components/ChatBot/ChatBot.module.css` with:
  - Responsive layout (sidebar on desktop >768px, full-width on mobile)
  - Input field styling (text area with submit button)
  - Message list scrollable area (max-height with overflow)
  - Loading spinner animation
  - Error message styling (red background, dismiss button)
  - Source citation styling (bordered, left-indent, clickable links)

- [x] T012 [P] [US1] Create LoadingSpinner component at `frontend/src/components/ChatBot/LoadingSpinner.tsx` with:
  - Animated spinner icon or skeleton loader
  - Shows while status='pending'
  - Includes "Waiting for response..." text

- [x] T013 [US1] Create error boundary and error display component at `frontend/src/components/ChatBot/ErrorDisplay.tsx` showing:
  - Error message text
  - Retry button that re-sends the failed request
  - Dismiss button to clear the error
  - Common error messages mapped from API error codes

- [x] T014 [US1] Integrate ChatBot component into Docusaurus layout via `frontend/src/theme/Root.tsx`:
  - Created Root.tsx as Docusaurus theme wrapper
  - ChatBot positioned as fixed sidebar (desktop) or bottom sheet (mobile)
  - Verified no breaking changes to existing functionality
  - ChatBot visible on all book pages
  - Documentation created: CHATBOT_INTEGRATION.md

- [x] T015 [P] [US1] Create unit tests for core US1 components:
  - `frontend/src/components/ChatBot/__tests__/ChatBot.test.tsx`: Input submission, loading state, error handling
  - `frontend/src/components/ChatBot/__tests__/useChat.test.ts`: API call, timeout, error handling
  - `frontend/src/services/__tests__/ragClient.test.ts`: Request/response serialization, status codes
  - `frontend/src/components/ChatBot/__tests__/ChatMessage.test.tsx`: Message rendering, chunk display

- [x] T016 [US1] Create integration test: `frontend/src/__tests__/integration/chatbot-e2e.test.ts` that:
  - Mocks the /chat API endpoint
  - Simulates user typing a question
  - Verifies API is called with correct request
  - Verifies response is displayed with sources
  - Tests conversation history accumulation

---

## Phase 4: User Story 2 - Scoped Query on Selected Text (P2)

Text selection feature: detect selected text, show context menu, pre-fill chatbot query.

### Goal
Enable users to select text on the page and ask questions about that specific selection, with the chatbot pre-filled with the selected text and using it as context.

### Independent Test Criteria (US2)
- [ ] Text selection on page is detected (5+ chars triggers action)
- [ ] Context menu or popover appears at selection location
- [ ] "Ask about this selection" button is functional
- [ ] Chatbot opens and selected text is pre-filled
- [ ] Query is sent with selected text as context to API

### Tasks

- [x] T017 [P] [US2] Create useTextSelection hook at `frontend/src/components/ChatBot/useTextSelection.ts` that:
  - Listens to document mouseup/touchend events
  - Captures window.getSelection() text
  - Filters selections < 5 chars
  - Returns { text, x, y } for positioning
  - Only triggers on text within book content area

- [x] T018 [P] [US2] Create TextSelectionMenu component at `frontend/src/components/ChatBot/TextSelectionMenu.tsx` showing:
  - Context menu or floating button near selection
  - "Ask about this selection" button/option
  - "Ask: [preview of selected text]" label (max 50 chars)
  - Dismiss on click-away or escape key

- [x] T019 [US2] Update ChatBot component to accept selected text context:
  - Add optional `selectedText` prop
  - Pre-fill query field if selectedText is provided
  - Auto-focus query field when selection context is active
  - Show "Ask about: [text]" label in input placeholder

- [x] T020 [US2] Update useChat hook to include selected text in API request:
  - Modify ChatRequest payload to include selected_text field (or embed in query)
  - Ensure backend receives full context
  - Document query format for scoped queries

- [x] T021 [P] [US2] Integrate text selection into page layout at `frontend/src/components/TextSelectionProvider.tsx`:
  - Wrap book content area with selection detection
  - Expose selected text to ChatBot component via context or props
  - Handle selection clear on blur/dismiss

- [x] T022 [US2] Create unit tests for text selection:
  - `frontend/src/components/ChatBot/__tests__/useTextSelection.test.ts`: Selection detection, filtering, positioning
  - `frontend/src/components/ChatBot/__tests__/TextSelectionMenu.test.tsx`: Menu appearance, button functionality

- [x] T023 [P] [US2] Create integration test for scoped queries:
  - `frontend/src/__tests__/integration/text-selection-e2e.test.ts`: Select text, open menu, pre-fill query, verify API includes context

- [x] T024 [US2] Create documentation for text selection feature at `frontend/docs/TEXT_SELECTION.md`:
  - How to trigger (mouse selection on page)
  - What happens (context menu appears)
  - Flow to chatbot with pre-filled query
  - Browser compatibility notes (min 5 char selection)

---

## Phase 5: User Story 3 - Configure Backend URL (P3)

Configuration for local and production deployments.

### Goal
Allow developers to configure the chatbot to connect to different RAG API backends (localhost for dev, production URL for production).

### Independent Test Criteria (US3)
- [ ] REACT_APP_RAG_AGENT_URL environment variable is read during build
- [ ] Default URL (http://localhost:8000/chat) is used when env var is not set
- [ ] Production URL can be set via environment variable
- [ ] Requests are sent to the configured URL

### Tasks

- [x] T025 [US3] Update `frontend/docusaurus.config.ts` to:
  - Read REACT_APP_RAG_AGENT_URL from process.env at build time
  - Pass it to ragConfig module or expose globally
  - Document the environment variable requirement

- [x] T026 [P] [US3] Create environment documentation at `frontend/.env.example`:
  - Example entry: `REACT_APP_RAG_AGENT_URL=http://localhost:8000/chat`
  - Comment explaining local vs production values
  - Instructions for setting up .env.local for development

- [x] T027 [US3] Create deployment guide at `specs/004-rag-chat-ui/DEPLOYMENT.md`:
  - How to set REACT_APP_RAG_AGENT_URL for production
  - Example: GitHub Actions secrets, Docker build args
  - Verification steps: test chatbot connects to correct API
  - Rollback procedure if API URL is wrong

- [x] T028 [P] [US3] Create integration test for configuration:
  - `frontend/src/__tests__/integration/config-e2e.test.ts`: Mock different API URLs, verify requests go to correct endpoint

---

## Phase 6: Polish & Cross-Cutting Concerns

Final refinement, comprehensive testing, and documentation.

### Goal
Finalize UI/UX, ensure robust error handling, comprehensive documentation, and production readiness.

### Test Criteria
- [ ] All existing Docusaurus functionality works without regression
- [ ] Chatbot renders correctly on desktop and mobile
- [ ] Build time increase is < 10%
- [ ] Performance metrics meet SLAs (API response in 5s, UI opens in 200ms)
- [ ] Accessibility standards met (WCAG 2.1 AA)

### Tasks

- [x] T029 [P] Responsive design refinement:
  - `frontend/src/components/ChatBot/ChatBot.module.css`: Update media queries for tablet breakpoints (481-768px)
  - Mobile layout: Chatbot becomes bottom-sheet modal (80vh height, rounded corners, drag-to-dismiss)
  - Desktop layout: Fixed sidebar (384px width, right edge, scrollable message list)
  - Test on mobile devices (iOS Safari, Android Chrome)

- [x] T030 [P] Accessibility audit and fixes:
  - Add ARIA labels to input field, buttons, message list
  - Ensure keyboard navigation works (Tab through inputs, Enter to submit, Escape to close)
  - Test with screen reader (NVDA, VoiceOver)
  - Update color contrast ratios (WCAG AA: 4.5:1 for text)
  - Document accessibility features at `frontend/docs/ACCESSIBILITY.md`

- [x] T031 Create comprehensive test coverage summary:
  - Document test suite location and how to run tests
  - Verify unit test coverage > 80% for chatbot components
  - Verify integration tests cover happy path and error scenarios
  - Create test matrix: browsers (Chrome, Firefox, Safari), devices (desktop, tablet, mobile)
  - Update `frontend/TESTING.md` with full details

---

## Dependencies & Sequencing

### Execution Order

```
Phase 1 (Setup)
  ├─ T001 (types)
  └─ T002 (config plan)
      ↓
Phase 2 (Foundational)
  ├─ T003 (RAG client)
  ├─ T004 (config)
  └─ T005 (test mocks)
      ↓
Phase 3 (US1 - P1: Ask Question)
  ├─ T006 (ChatBot main component)
  ├─ T007 (useChat hook)
  ├─ T008 (ChatMessage component)
  ├─ T009 (SourceCitation component)
  ├─ T010 (useConversation hook)
  ├─ T011 (CSS styling)
  ├─ T012 (LoadingSpinner)
  ├─ T013 (ErrorDisplay)
  ├─ T014 (integrate into layout)
  ├─ T015 (unit tests)
  └─ T016 (integration tests)
      ↓
Phase 4 (US2 - P2: Text Selection) [Can start after T006, T007 complete]
  ├─ T017 (useTextSelection hook)
  ├─ T018 (TextSelectionMenu)
  ├─ T019 (update ChatBot for selected text)
  ├─ T020 (update useChat)
  ├─ T021 (TextSelectionProvider)
  ├─ T022 (unit tests)
  ├─ T023 (integration tests)
  └─ T024 (documentation)
      ↓
Phase 5 (US3 - P3: Backend Config) [Can start after T003, T004 complete]
  ├─ T025 (update docusaurus.config.ts)
  ├─ T026 (.env.example)
  ├─ T027 (DEPLOYMENT.md)
  └─ T028 (integration test)
      ↓
Phase 6 (Polish & Cross-Cutting)
  ├─ T029 (responsive design)
  ├─ T030 (accessibility)
  └─ T031 (test coverage)
```

### Parallel Execution Within Phases

**Phase 3 (US1)**: T007, T008, T009, T011, T012 can run in parallel (independent components)
**Phase 4 (US2)**: T017, T018 can run in parallel (independent features)
**Phase 5 (US3)**: T025, T026, T027, T028 are sequential (build on each other)
**Phase 6**: T029, T030 can run in parallel (UI polish vs accessibility)

---

## MVP Scope & Rollout

### MVP (Phase 3 Only)
Implement **User Story 1 (P1)** for initial release:
- Tasks T001-T016 (33 days at typical pace, 2 weeks with team)
- Core chatbot: ask questions, get responses, see sources
- No text selection feature
- Configuration defaults to localhost

**MVP Success Criteria**:
- ✅ Users can ask questions about book content
- ✅ Responses appear within 5 seconds
- ✅ Sources are cited and clickable
- ✅ Conversation history maintained during session
- ✅ Error messages on API failures
- ✅ Works on desktop (Chrome, Firefox, Safari)

### Phase 2 Enhancement (Phase 4)
Add **User Story 2 (P2)** to expand UX:
- Tasks T017-T024
- Text selection context menu
- Scoped queries on selected text

### Phase 3 Deployment (Phase 5)
Add **User Story 3 (P3)** for production readiness:
- Tasks T025-T028
- Environment-based backend URL
- Production deployment documentation

### Production Polish (Phase 6)
Final refinement:
- Tasks T029-T031
- Responsive design for mobile
- Accessibility compliance
- Test coverage validation

---

## File Structure Summary

### New Files Created

```
frontend/
├── src/
│   ├── types/
│   │   └── chat.ts                              # T001
│   ├── services/
│   │   ├── ragClient.ts                         # T003
│   │   └── __tests__/
│   │       └── ragClient.test.ts                # T015
│   ├── config/
│   │   └── ragConfig.ts                         # T004
│   ├── components/
│   │   ├── ChatBot/
│   │   │   ├── ChatBot.tsx                      # T006
│   │   │   ├── useChat.ts                       # T007
│   │   │   ├── ChatMessage.tsx                  # T008
│   │   │   ├── SourceCitation.tsx               # T009
│   │   │   ├── useConversation.ts               # T010
│   │   │   ├── ChatBot.module.css               # T011
│   │   │   ├── LoadingSpinner.tsx               # T012
│   │   │   ├── ErrorDisplay.tsx                 # T013
│   │   │   ├── useTextSelection.ts              # T017
│   │   │   ├── TextSelectionMenu.tsx            # T018
│   │   │   └── __tests__/
│   │   │       ├── ChatBot.test.tsx             # T015
│   │   │       ├── useChat.test.ts              # T015
│   │   │       ├── ChatMessage.test.tsx         # T015
│   │   │       ├── useTextSelection.test.ts     # T022
│   │   │       └── TextSelectionMenu.test.tsx   # T022
│   │   └── TextSelectionProvider.tsx            # T021
│   ├── theme/
│   │   └── DocLayout.tsx                        # T014 (MODIFIED)
│   └── __tests__/
│       ├── mocks/
│       │   └── ragApi.mock.ts                   # T005
│       └── integration/
│           ├── chatbot-e2e.test.ts              # T016
│           ├── text-selection-e2e.test.ts       # T023
│           └── config-e2e.test.ts               # T028
├── .env.example                                 # T026
├── docusaurus.config.ts                         # T025 (MODIFIED)
└── docs/
    ├── TEXT_SELECTION.md                        # T024
    ├── ACCESSIBILITY.md                         # T030
    └── TESTING.md                               # T031

specs/004-rag-chat-ui/
├── DEPLOYMENT.md                                # T027
└── (existing: spec.md, plan.md, research.md, data-model.md, contracts/)
```

### Modified Files

- `frontend/docusaurus.config.ts` (T025: add RAG_AGENT_URL)
- `frontend/src/theme/DocLayout.tsx` (T014: integrate ChatBot)

---

## Quality Checklist

Use this checklist to validate task completion:

### Task Acceptance
- [ ] Task code compiles without errors
- [ ] All file paths are correct and match spec
- [ ] Code follows project style guide
- [ ] Tests pass (unit or integration)
- [ ] No breaking changes to existing features
- [ ] Documentation is complete and accurate

### Phase Acceptance
- [ ] All tasks in the phase are marked [X]
- [ ] Phase test criteria are met
- [ ] No unresolved TODOs or FIXMEs in code
- [ ] Integration with previous phases works
- [ ] Build time is not negatively impacted

### Feature Acceptance (All Phases)
- [ ] Spec requirements are met (all FR, SC items)
- [ ] Edge cases handled (8 from spec.md)
- [ ] Performance SLAs met (5s response, 200ms UI load, 95%+ API success)
- [ ] Accessibility audit passed (WCAG 2.1 AA)
- [ ] Browser compatibility verified (Chrome, Firefox, Safari, Edge)
- [ ] No regressions in existing Docusaurus site functionality
- [ ] Documentation is complete (DEPLOYMENT.md, TEXT_SELECTION.md, ACCESSIBILITY.md)

---

## Commit Strategy

After completing each phase, create a commit:

```bash
# Phase 1
git commit -m "Phase 1: Setup types and configuration for SPEC-4 chatbot"

# Phase 2
git commit -m "Phase 2: Add RAG API client and foundational infrastructure for SPEC-4"

# Phase 3
git commit -m "Phase 3: Implement core chatbot (US1-P1) - Ask questions about book content"

# Phase 4
git commit -m "Phase 4: Add text selection feature (US2-P2) for scoped queries"

# Phase 5
git commit -m "Phase 5: Add backend URL configuration (US3-P3) for production deployment"

# Phase 6
git commit -m "Phase 6: Polish UI, accessibility, and comprehensive testing for SPEC-4"
```

---

## Next Steps

1. **Review & Approve**: Verify tasks match specification and implementation plan
2. **Begin Phase 1**: Create types file (T001) and configuration plan (T002)
3. **Progress Tracking**: Mark tasks [X] as completed; update branch with feature branch commits
4. **Testing**: Run test suite after each phase to validate acceptance criteria
5. **Documentation**: Ensure DEPLOYMENT.md and other docs are updated before production release

---

## Related Documents

- **spec.md**: Feature specification and requirements
- **plan.md**: Architecture and design decisions
- **research.md**: Technology research and best practices
- **data-model.md**: Entity definitions and data structures
- **contracts/chatbot-api.openapi.yaml**: API contract specification
- **checklists/requirements.md**: Quality validation checklist
