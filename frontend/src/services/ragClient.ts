/**
 * RAG API HTTP Client
 *
 * Provides HTTP communication with the FastAPI RAG Agent API (Spec 3).
 * Uses native Fetch API with timeout via AbortController.
 * Handles request/response serialization and error mapping.
 *
 * API Contract: /chat endpoint (POST)
 * See: specs/004-rag-chat-ui/contracts/chatbot-api.openapi.yaml
 */

import type { ChatRequest, ChatResponse } from '../types/chat';

/**
 * Error response from the RAG API.
 * Maps API error codes to user-friendly messages.
 */
export interface RagApiError {
  code: string; // Error code (e.g., VALIDATION_ERROR, OPENAI_TIMEOUT)
  message: string; // Human-readable error message
  status?: number; // HTTP status code
}

/**
 * Configuration for API client.
 */
interface RagClientConfig {
  apiUrl: string; // Base URL for RAG Agent API (should end with /chat endpoint)
  timeout?: number; // Request timeout in milliseconds (default: 30000)
}

/**
 * RAG API Client
 *
 * Handles all HTTP communication with the FastAPI RAG Agent API.
 * Implements timeout handling, error mapping, and response validation.
 */
export class RagClient {
  private apiUrl: string;
  private timeout: number;

  /**
   * Create a new RAG API client.
   *
   * @param config Configuration object with apiUrl (required) and timeout (optional)
   * @throws Error if apiUrl is not provided
   */
  constructor(config: RagClientConfig) {
    if (!config.apiUrl) {
      throw new Error('RagClient: apiUrl is required in configuration');
    }

    this.apiUrl = config.apiUrl;
    this.timeout = config.timeout || 30000; // Default 30 seconds
  }

  /**
   * Send a query to the RAG Agent API.
   *
   * @param request ChatRequest payload to send to /chat endpoint
   * @returns Promise<ChatResponse> The response from the API
   * @throws RagApiError if the API request fails or times out
   *
   * @example
   * ```typescript
   * const client = new RagClient({ apiUrl: 'http://localhost:8000/chat' });
   * try {
   *   const response = await client.sendQuery({
   *     query: 'What is ROS2?',
   *     retrieval_scope: 'full_collection',
   *     top_k: 5
   *   });
   *   console.log(response.answer);
   * } catch (error) {
   *   console.error(error.message);
   * }
   * ```
   */
  async sendQuery(request: ChatRequest): Promise<ChatResponse> {
    // Validate request before sending
    this.validateRequest(request);

    const controller = new AbortController();
    const timeoutId = setTimeout(() => controller.abort(), this.timeout);

    try {
      const response = await fetch(this.apiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify(request),
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      // Handle non-2xx status codes
      if (!response.ok) {
        await this.handleHttpError(response);
      }

      // Parse and validate response
      const data = await response.json();
      this.validateResponse(data);

      return data as ChatResponse;
    } catch (error) {
      clearTimeout(timeoutId);

      // Handle different error types
      if (error instanceof TypeError && error.message.includes('Failed to fetch')) {
        throw {
          code: 'NETWORK_ERROR',
          message: 'Unable to connect to the RAG service. Check your internet connection.',
          status: 0,
        } as RagApiError;
      }

      if (error instanceof DOMException && error.name === 'AbortError') {
        throw {
          code: 'TIMEOUT_ERROR',
          message: `Request took too long (>${this.timeout / 1000}s). Please try again.`,
          status: 408,
        } as RagApiError;
      }

      // Re-throw if it's already a RagApiError
      if (error && typeof error === 'object' && 'code' in error && 'message' in error) {
        throw error as RagApiError;
      }

      // Unknown error
      throw {
        code: 'UNKNOWN_ERROR',
        message: error instanceof Error ? error.message : 'An unknown error occurred',
        status: 500,
      } as RagApiError;
    }
  }

  /**
   * Validate the request before sending.
   * Ensures query is non-empty and within size limits.
   *
   * @param request ChatRequest to validate
   * @throws RagApiError if validation fails
   */
  private validateRequest(request: ChatRequest): void {
    if (!request.query) {
      throw {
        code: 'VALIDATION_ERROR',
        message: 'Query cannot be empty',
        status: 400,
      } as RagApiError;
    }

    if (request.query.length > 10000) {
      throw {
        code: 'VALIDATION_ERROR',
        message: 'Query exceeds maximum length of 10,000 characters',
        status: 400,
      } as RagApiError;
    }

    if (request.retrieval_scope === 'text_only' && !request.context_text) {
      throw {
        code: 'VALIDATION_ERROR',
        message: 'context_text is required when retrieval_scope is "text_only"',
        status: 400,
      } as RagApiError;
    }

    if (request.context_text && request.context_text.length > 50000) {
      throw {
        code: 'VALIDATION_ERROR',
        message: 'context_text exceeds maximum length of 50,000 characters',
        status: 400,
      } as RagApiError;
    }

    if (request.top_k && (request.top_k < 1 || request.top_k > 100)) {
      throw {
        code: 'VALIDATION_ERROR',
        message: 'top_k must be between 1 and 100',
        status: 400,
      } as RagApiError;
    }
  }

  /**
   * Validate the API response.
   * Ensures the response has the expected structure.
   *
   * @param response The parsed JSON response
   * @throws RagApiError if validation fails
   */
  private validateResponse(response: any): void {
    if (!response || typeof response !== 'object') {
      throw {
        code: 'MALFORMED_RESPONSE',
        message: 'API returned invalid response format',
        status: 500,
      } as RagApiError;
    }

    if (!response.status || !['success', 'error'].includes(response.status)) {
      throw {
        code: 'MALFORMED_RESPONSE',
        message: 'API response missing required "status" field',
        status: 500,
      } as RagApiError;
    }

    // If error status, error field should be present
    if (response.status === 'error') {
      if (!response.error || !response.error.code || !response.error.message) {
        throw {
          code: 'MALFORMED_RESPONSE',
          message: 'API error response missing required fields',
          status: 500,
        } as RagApiError;
      }
    }
  }

  /**
   * Handle HTTP error responses from the API.
   * Maps HTTP status codes and error responses to RagApiError.
   *
   * @param response The fetch Response object
   * @throws RagApiError with appropriate code and message
   */
  private async handleHttpError(response: Response): Promise<never> {
    let errorData: any;

    try {
      errorData = await response.json();
    } catch {
      // If response is not JSON, create a generic error
      errorData = {};
    }

    const status = response.status;

    // Map status codes to error codes
    if (status === 400) {
      throw {
        code: errorData.error?.code || 'VALIDATION_ERROR',
        message: errorData.error?.message || 'Invalid request. Please check your input.',
        status: 400,
      } as RagApiError;
    }

    if (status === 429) {
      throw {
        code: 'RATE_LIMITED',
        message: 'Too many requests. Please wait a moment and try again.',
        status: 429,
      } as RagApiError;
    }

    if (status === 500) {
      throw {
        code: errorData.error?.code || 'AGENT_FAILED',
        message: errorData.error?.message || 'Server error. Please try again later.',
        status: 500,
      } as RagApiError;
    }

    if (status === 503) {
      throw {
        code: errorData.error?.code || 'SERVICE_UNAVAILABLE',
        message: errorData.error?.message || 'RAG service is temporarily unavailable. Please try again later.',
        status: 503,
      } as RagApiError;
    }

    // Generic HTTP error for other status codes
    throw {
      code: `HTTP_${status}`,
      message: `Server returned status ${status}: ${response.statusText}`,
      status: status,
    } as RagApiError;
  }
}

/**
 * Create a singleton instance of the RAG client for use throughout the app.
 *
 * @param apiUrl The RAG Agent API URL (e.g., http://localhost:8000/chat)
 * @returns RagClient instance
 *
 * @example
 * ```typescript
 * const client = createRagClient('http://localhost:8000/chat');
 * ```
 */
export function createRagClient(apiUrl: string): RagClient {
  return new RagClient({ apiUrl });
}
