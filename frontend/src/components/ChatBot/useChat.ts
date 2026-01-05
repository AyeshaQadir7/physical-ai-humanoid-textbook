/**
 * useChat Hook
 *
 * Handles all communication with the RAG Agent API.
 * Manages sending queries, handling responses, and error handling.
 *
 * Uses native Fetch API with AbortController for timeout handling.
 * Integrates with RagClient for request/response serialization.
 */

import { useCallback } from 'react';
import { RagClient, type RagApiError } from '../../services/ragClient';
import { ragConfig } from '../../config/ragConfig';
import type { ChatRequest, ChatResponse, RetrievedChunk } from '../../types/chat';

/**
 * Result of a chat query
 */
export interface ChatQueryResult {
  success: boolean;
  answer?: string;
  chunks?: RetrievedChunk[];
  error?: string;
}

/**
 * useChat Hook
 *
 * Provides methods to communicate with the RAG API and manage request state.
 * Handles timeout, error mapping, and response validation.
 *
 * @param onSuccess Optional callback when API request succeeds
 * @param onError Optional callback when API request fails
 * @returns Methods to send queries and manage state
 *
 * @example
 * ```typescript
 * const { sendQuery, isLoading, error } = useChat();
 *
 * const handleSubmit = async (query: string) => {
 *   const result = await sendQuery({ query, top_k: 5 });
 *   if (result.success) {
 *     console.log('Answer:', result.answer);
 *   } else {
 *     console.error('Error:', result.error);
 *   }
 * };
 * ```
 */
export function useChat(
  onSuccess?: (result: ChatQueryResult) => void,
  onError?: (error: string) => void
) {
  /**
   * Send a query to the RAG Agent API.
   *
   * @param request The chat request (query and optional parameters)
   * @returns Promise<ChatQueryResult> with success/answer/chunks or error
   */
  const sendQuery = useCallback(
    async (request: ChatRequest): Promise<ChatQueryResult> => {
      try {
        // Validate request on client side
        if (!request.query || request.query.trim().length === 0) {
          const error = 'Please enter a question';
          onError?.(error);
          return { success: false, error };
        }

        if (request.query.length > 10000) {
          const error = 'Question exceeds maximum length (10,000 characters)';
          onError?.(error);
          return { success: false, error };
        }

        // Create API client with configured URL
        const client = new RagClient({
          apiUrl: ragConfig.apiUrl,
          timeout: ragConfig.requestTimeout,
        });

        // Send request to API
        const response: ChatResponse = await client.sendQuery(request);

        // Handle API error response
        if (response.status === 'error') {
          const errorMessage = response.error?.message || 'Unknown error occurred';
          onError?.(errorMessage);
          return { success: false, error: errorMessage };
        }

        // Handle success response
        if (!response.answer) {
          const error = 'No answer received from API';
          onError?.(error);
          return { success: false, error };
        }

        const result: ChatQueryResult = {
          success: true,
          answer: response.answer,
          chunks: response.retrieved_chunks,
        };

        onSuccess?.(result);
        return result;
      } catch (err) {
        // Handle client-side errors
        let errorMessage = 'An unknown error occurred';

        if (err && typeof err === 'object') {
          if ('message' in err) {
            errorMessage = (err as any).message;
          } else if ('code' in err) {
            // RagApiError
            const apiError = err as RagApiError;
            errorMessage = apiError.message;
          }
        } else if (err instanceof Error) {
          errorMessage = err.message;
        }

        // Map common errors to user-friendly messages
        if (errorMessage.includes('Failed to fetch')) {
          errorMessage = 'Unable to connect to the chatbot service. Please check your internet connection.';
        } else if (errorMessage.includes('timeout') || errorMessage.includes('timed out')) {
          errorMessage = 'Request took too long. Please try again.';
        } else if (errorMessage.includes('network')) {
          errorMessage = 'Network error. Please check your internet connection and try again.';
        }

        onError?.(errorMessage);
        return { success: false, error: errorMessage };
      }
    },
    [onSuccess, onError]
  );

  return {
    sendQuery,
  };
}
