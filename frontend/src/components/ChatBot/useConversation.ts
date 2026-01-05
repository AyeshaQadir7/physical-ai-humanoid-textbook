/**
 * useConversation Hook
 *
 * Manages conversation state for the chatbot, including:
 * - Message history (queries and responses)
 * - Loading state during API requests
 * - Error messages
 * - Session management (cleared on page reload)
 *
 * All conversation data is stored in React state.
 * Session-based (cleared when component unmounts or page reloads).
 */

import { useState, useCallback } from 'react';
import type { ChatMessage, RetrievedChunk } from '../../types/chat';

/**
 * Interface for useConversation hook return value
 */
export interface UseConversationResult {
  // State
  messages: ChatMessage[];
  isLoading: boolean;
  error: string | null;

  // Actions
  addPendingMessage: (query: string, selectedText?: string) => string; // Returns message ID
  updateMessageWithResponse: (
    messageId: string,
    answer: string,
    retrievedChunks: RetrievedChunk[]
  ) => void;
  updateMessageWithError: (messageId: string, error: string) => void;
  clearError: () => void;
  clearHistory: () => void;
  setLoading: (loading: boolean) => void;
}

/**
 * useConversation Hook
 *
 * Manages all conversation state for a chatbot session.
 * Messages persist across navigation but are cleared on page reload.
 *
 * @returns UseConversationResult with messages array and action methods
 *
 * @example
 * ```typescript
 * const {
 *   messages,
 *   isLoading,
 *   error,
 *   addPendingMessage,
 *   updateMessageWithResponse,
 *   clearError,
 * } = useConversation();
 *
 * // Add a pending message when user submits query
 * const messageId = addPendingMessage('What is ROS2?');
 *
 * // Update when API responds
 * updateMessageWithResponse(
 *   messageId,
 *   'ROS2 is...',
 *   retrievedChunks
 * );
 * ```
 */
export function useConversation(): UseConversationResult {
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [isLoading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  /**
   * Add a pending message to the conversation.
   * Called when user submits a query.
   *
   * @param query The user's question
   * @param selectedText Optional selected text for scoped queries
   * @returns The message ID (needed to update it later)
   */
  const addPendingMessage = useCallback(
    (query: string, selectedText?: string): string => {
      const messageId = crypto.randomUUID();

      const pendingMessage: ChatMessage = {
        id: messageId,
        query,
        answer: null,
        retrieved_chunks: [],
        timestamp: new Date(),
        status: 'pending',
        selected_text: selectedText,
      };

      setMessages((prev) => [...prev, pendingMessage]);
      setError(null); // Clear any previous error when new message is sent

      return messageId;
    },
    []
  );

  /**
   * Update a message with the successful API response.
   *
   * @param messageId ID of the message to update
   * @param answer The AI-generated response
   * @param retrievedChunks The source passages used to generate the response
   */
  const updateMessageWithResponse = useCallback(
    (messageId: string, answer: string, retrievedChunks: RetrievedChunk[]): void => {
      setMessages((prev) =>
        prev.map((msg) =>
          msg.id === messageId
            ? {
                ...msg,
                answer,
                retrieved_chunks: retrievedChunks,
                status: 'success' as const,
              }
            : msg
        )
      );
      setLoading(false);
    },
    []
  );

  /**
   * Update a message with an error.
   *
   * @param messageId ID of the message to update
   * @param errorMessage The error message to display
   */
  const updateMessageWithError = useCallback(
    (messageId: string, errorMessage: string): void => {
      setMessages((prev) =>
        prev.map((msg) =>
          msg.id === messageId
            ? {
                ...msg,
                status: 'error' as const,
                error: errorMessage,
              }
            : msg
        )
      );
      setError(errorMessage);
      setLoading(false);
    },
    []
  );

  /**
   * Clear the global error message.
   * Called when user dismisses error or sends a new message.
   */
  const clearError = useCallback((): void => {
    setError(null);
  }, []);

  /**
   * Clear all conversation history.
   * Called when user clicks "Clear History" button.
   */
  const clearHistory = useCallback((): void => {
    setMessages([]);
    setError(null);
    setLoading(false);
  }, []);

  return {
    messages,
    isLoading,
    error,
    addPendingMessage,
    updateMessageWithResponse,
    updateMessageWithError,
    clearError,
    clearHistory,
    setLoading,
  };
}
