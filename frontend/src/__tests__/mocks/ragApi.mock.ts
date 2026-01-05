/**
 * RAG API Mocks and Test Fixtures
 *
 * Provides mock ChatResponse objects and utility functions for testing
 * chatbot components and the RAG API client.
 *
 * Usage in tests:
 * ```typescript
 * import { mockChatResponse, createMockChunk } from './ragApi.mock';
 *
 * const response = mockChatResponse({
 *   query: 'What is ROS2?',
 *   answer: 'ROS2 is a middleware...',
 * });
 * ```
 */

import type { ChatResponse, RetrievedChunk } from '../../types/chat';

/**
 * Create a mock RetrievedChunk for testing.
 *
 * @param overrides Partial values to override defaults
 * @returns A complete RetrievedChunk object
 */
export function createMockChunk(overrides?: Partial<RetrievedChunk>): RetrievedChunk {
  return {
    chunk_id: 'chunk_001',
    text: 'ROS2 is a flexible middleware for robotics applications. It provides improved real-time performance and security compared to ROS1.',
    similarity_score: 0.94,
    source_url: 'https://example.com/module-1-ros2/chapter-1-intro',
    page_title: 'Introduction to ROS2',
    section_headers: ['Module 1: ROS2 Foundations', 'Chapter 1: Overview'],
    ...overrides,
  };
}

/**
 * Create a mock ChatResponse for testing.
 *
 * @param overrides Partial values to override defaults
 * @returns A complete ChatResponse object (success status)
 */
export function mockChatResponse(overrides?: Partial<ChatResponse>): ChatResponse {
  return {
    query: 'What is ROS2?',
    answer: 'ROS2 is a flexible middleware for robotics applications, providing improved real-time performance and security compared to ROS1.',
    retrieved_chunks: [
      createMockChunk({
        chunk_id: 'chunk_001',
        similarity_score: 0.94,
      }),
      createMockChunk({
        chunk_id: 'chunk_002',
        text: 'ROS2 uses a pub/sub architecture for inter-process communication.',
        similarity_score: 0.88,
      }),
    ],
    execution_metrics: {
      retrieval_time_ms: 245.3,
      generation_time_ms: 2150.7,
      total_time_ms: 2396.0,
    },
    retrieval_scope: 'full_collection',
    status: 'success',
    error: undefined,
    ...overrides,
  };
}

/**
 * Create a mock ChatResponse with error status for testing error scenarios.
 *
 * @param errorCode The error code (e.g., OPENAI_TIMEOUT, RETRIEVAL_FAILED)
 * @param errorMessage The error message
 * @returns A ChatResponse object with error status
 */
export function mockChatErrorResponse(
  errorCode: string = 'AGENT_FAILED',
  errorMessage: string = 'Agent response generation failed'
): ChatResponse {
  return {
    query: 'What is ROS2?',
    answer: null,
    retrieved_chunks: [],
    execution_metrics: {
      retrieval_time_ms: 0,
      generation_time_ms: 0,
      total_time_ms: 0,
    },
    retrieval_scope: 'full_collection',
    status: 'error',
    error: {
      code: errorCode,
      message: errorMessage,
    },
  };
}

/**
 * Mock handlers for fetch API (for use with MSW or similar)
 *
 * These handlers mock the RAG API endpoints for testing without a real backend.
 */
export const mockRagApiHandlers = {
  /**
   * Handle successful /chat request
   */
  chatSuccess: (req: Request) => {
    return new Response(
      JSON.stringify(
        mockChatResponse({
          query: (req.body as any)?.query || 'What is ROS2?',
        })
      ),
      {
        status: 200,
        headers: { 'Content-Type': 'application/json' },
      }
    );
  },

  /**
   * Handle /chat request with validation error
   */
  chatValidationError: (req: Request) => {
    return new Response(
      JSON.stringify(
        mockChatErrorResponse('VALIDATION_ERROR', 'Query cannot be empty')
      ),
      {
        status: 400,
        headers: { 'Content-Type': 'application/json' },
      }
    );
  },

  /**
   * Handle /chat request with timeout error
   */
  chatTimeout: (req: Request) => {
    return new Response(
      JSON.stringify(
        mockChatErrorResponse('OPENAI_TIMEOUT', 'Generation service timeout, please retry')
      ),
      {
        status: 503,
        headers: { 'Content-Type': 'application/json' },
      }
    );
  },

  /**
   * Handle /chat request with rate limit error
   */
  chatRateLimited: (req: Request) => {
    return new Response(
      JSON.stringify(
        mockChatErrorResponse('RATE_LIMITED', 'Rate limited by generation service, please retry after a moment')
      ),
      {
        status: 429,
        headers: { 'Content-Type': 'application/json' },
      }
    );
  },

  /**
   * Handle /chat request with server error
   */
  chatServerError: (req: Request) => {
    return new Response(
      JSON.stringify(
        mockChatErrorResponse('AGENT_FAILED', 'Agent response generation failed')
      ),
      {
        status: 500,
        headers: { 'Content-Type': 'application/json' },
      }
    );
  },

  /**
   * Handle /health request
   */
  healthCheck: (req: Request) => {
    return new Response(
      JSON.stringify({ status: 'ok' }),
      {
        status: 200,
        headers: { 'Content-Type': 'application/json' },
      }
    );
  },
};

/**
 * Test data for various scenarios
 */
export const testData = {
  /**
   * Sample queries for testing
   */
  sampleQueries: [
    'What is ROS2?',
    'How do I install ROS2?',
    'What are the advantages of ROS2 over ROS1?',
    'How does ROS2 handle real-time communication?',
  ],

  /**
   * Sample error codes from the API
   */
  errorCodes: [
    'VALIDATION_ERROR',
    'RETRIEVAL_FAILED',
    'AGENT_FAILED',
    'OPENAI_TIMEOUT',
    'RATE_LIMITED',
    'GENERATION_FAILED',
    'OPENAI_UNAVAILABLE',
  ],

  /**
   * Edge case inputs for validation testing
   */
  edgeCases: {
    emptyQuery: '',
    veryLongQuery: 'a'.repeat(10001), // Exceeds 10k char limit
    validQuery: 'What is ROS2?',
    minimalQuery: 'ROS',
  },
};
