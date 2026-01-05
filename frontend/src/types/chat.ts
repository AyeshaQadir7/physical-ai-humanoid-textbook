/**
 * Types for RAG Chatbot conversation data
 *
 * These types define the structure of chat messages, retrieved chunks,
 * and conversation state for the chatbot feature (SPEC-4).
 * All data is stored client-side in React state; no server persistence.
 */

/**
 * A single query-response pair in the conversation.
 * Represents the complete lifecycle of a user interaction with the chatbot.
 */
export interface ChatMessage {
  // Identification
  id: string; // UUID, unique per message

  // User Query
  query: string; // User's natural language question (1-10,000 chars)

  // AI Response
  answer: string | null; // LLM-generated response (null while loading)

  // Retrieved Context
  retrieved_chunks: RetrievedChunk[]; // Source passages used to generate response

  // Metadata
  timestamp: Date; // When message was created
  status: "pending" | "success" | "error"; // Current state
  error?: string; // Error message if status='error'

  // Optional: Text selection context
  selected_text?: string; // User-selected text (if P2: text selection feature)
}

/**
 * A single piece of textbook content retrieved by semantic search.
 * Represents a source passage that the chatbot uses to ground its response.
 */
export interface RetrievedChunk {
  // Identification
  chunk_id: string; // Unique identifier from Qdrant

  // Content
  text: string; // Actual textbook passage (excerpt, not full page, max 5k chars)

  // Relevance & Metadata
  similarity_score: number; // Semantic relevance (0.0 to 1.0)
  source_url: string; // URL to textbook page
  page_title: string; // Title of the page/chapter
  section_headers: string[]; // Breadcrumb path (e.g., ["Module 1", "Chapter 2"])
}

/**
 * Container for all chat messages and conversation state during the session.
 * All data persists in React state for the current session only.
 */
export interface ConversationState {
  // Message history
  messages: ChatMessage[]; // Array of all messages in this session

  // UI state
  isLoading: boolean; // True while awaiting API response
  error: string | null; // Global error message (not message-specific)

  // Configuration
  selectedText?: string; // Text selected on page (for P2 feature)
}

/**
 * Request payload sent to the FastAPI RAG Agent API's /chat endpoint.
 * Defines what information the frontend sends to the backend.
 */
export interface ChatRequest {
  query: string; // User's natural language question (1-10k chars, required)

  retrieval_scope?: "full_collection" | "text_only"; // Which retrieval mode to use
  // - "full_collection": Search entire textbook via Qdrant (default)
  // - "text_only": Use only provided context_text (skip Qdrant)

  top_k?: number; // Number of relevant chunks to retrieve (1-100, default 5)

  context_text?: string; // User-provided context for "text_only" mode (max 50k chars)
  // Required if retrieval_scope="text_only"
}

/**
 * Response payload received from the FastAPI RAG Agent API's /chat endpoint.
 * Defines the structure of data returned by the backend.
 */
export interface ChatResponse {
  query: string; // Echo of the original user query

  answer: string | null; // The AI-generated response (null if error)

  retrieved_chunks: RetrievedChunk[]; // Source passages used to generate response

  execution_metrics: {
    retrieval_time_ms: number; // Time spent retrieving chunks from Qdrant
    generation_time_ms: number; // Time spent generating response via LLM
    total_time_ms: number; // Total end-to-end time
  };

  retrieval_scope: "full_collection" | "text_only"; // Which retrieval mode was used

  status: "success" | "error"; // Request status

  error?: {
    code: string; // Error code for programmatic handling (e.g., VALIDATION_ERROR)
    message: string; // Human-readable error message
  }; // Only present if status="error"
}

/**
 * Validation rules for ChatMessage fields
 * Used for client-side validation before sending to API
 */
export const ChatMessageValidation = {
  id: { type: "uuid" },
  query: { minLength: 1, maxLength: 10000 },
  answer: { maxLength: 50000 },
  retrieved_chunks: { maxLength: 10 }, // API constraint: max 10 chunks per message
  timestamp: { type: "iso8601" },
  status: { enum: ["pending", "success", "error"] },
  error: { minLength: 1 },
  selected_text: { maxLength: 5000 },
} as const;

/**
 * Validation rules for RetrievedChunk fields
 */
export const RetrievedChunkValidation = {
  chunk_id: { pattern: /^[a-zA-Z0-9_]+$/ }, // Alphanumeric + underscores
  text: { minLength: 1, maxLength: 5000 },
  similarity_score: { min: 0.0, max: 1.0 },
  source_url: { pattern: /^https?:\/\// }, // Must start with http:// or https://
  page_title: { minLength: 1, maxLength: 200 },
  section_headers: { maxLength: 5 }, // Max 5 levels deep
} as const;
