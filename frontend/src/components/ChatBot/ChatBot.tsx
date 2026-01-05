/**
 * ChatBot Component
 *
 * Main chatbot component that allows users to ask questions about the textbook.
 * Integrates conversation state, API communication, and message display.
 *
 * Features:
 * - Ask questions about book content
 * - Display responses with source citations
 * - Conversation history (session-based)
 * - Error handling and retry
 * - Loading indicators
 * - Responsive design (sidebar on desktop, modal on mobile)
 */

import React, { useEffect, useRef, useState } from "react";
import { Send, X } from "lucide-react";
import { useConversation } from "./useConversation";
import { useChat } from "./useChat";
import { ChatMessage } from "./ChatMessage";
import { ErrorDisplay } from "./ErrorDisplay";
import styles from "./ChatBot.module.css";
import type { ChatRequest } from "../../types/chat";

export interface ChatBotProps {
  /** Optional selected text for scoped queries (P2 feature) */
  selectedText?: string;
  /** Optional callback when selectedText is accepted */
  onSelectedTextUsed?: () => void;
  /** Optional CSS class name */
  className?: string;
  /** Optional callback to close the chatbot */
  onClose?: () => void;
}

/**
 * ChatBot Component
 *
 * Main component for the RAG chatbot. Manages user input, API communication,
 * conversation history, and message display.
 *
 * @example
 * ```tsx
 * <ChatBot />
 * ```
 *
 * @example With selected text (P2 feature)
 * ```tsx
 * <ChatBot
 *   selectedText="ROS2 is a middleware"
 *   onSelectedTextUsed={() => clearSelection()}
 * />
 * ```
 */
export const ChatBot: React.FC<ChatBotProps> = ({
  selectedText,
  onSelectedTextUsed,
  className,
  onClose,
}) => {
  const {
    messages,
    isLoading,
    error,
    addPendingMessage,
    updateMessageWithResponse,
    updateMessageWithError,
    clearError,
    clearHistory,
    setLoading,
  } = useConversation();

  const [inputValue, setInputValue] = useState("");
  const lastPendingMsgRef = useRef<string>("");

  const { sendQuery } = useChat(
    // On success
    (result) => {
      if (lastPendingMsgRef.current) {
        updateMessageWithResponse(
          lastPendingMsgRef.current,
          result.answer || "",
          result.chunks || []
        );

        // Clear input after successful response
        setInputValue("");

        lastPendingMsgRef.current = "";
      }
    },
    // On error
    (errorMsg) => {
      if (lastPendingMsgRef.current) {
        updateMessageWithError(lastPendingMsgRef.current, errorMsg);
        lastPendingMsgRef.current = "";
      }
    }
  );

  const inputRef = useRef<HTMLTextAreaElement>(null);
  const messagesEndRef = useRef<HTMLDivElement>(null);

  // Auto-scroll to bottom when new messages arrive
  useEffect(() => {
    messagesEndRef.current?.scrollIntoView({ behavior: "smooth" });
  }, [messages]);

  // Pre-fill with selected text if provided (P2 feature)
  useEffect(() => {
    if (selectedText && inputRef.current) {
      inputRef.current.value = `Explain this: ${selectedText}`;
      inputRef.current.focus();
    }
  }, [selectedText]);

  /**
   * Handle form submission: send query to API
   */
  const handleSubmit = async (e: React.FormEvent<HTMLFormElement>) => {
    e.preventDefault();

    const query = inputValue.trim();

    if (!query) {
      clearError();
      return; // Prevent submission of empty query
    }

    // Add pending message and track it for updates
    setLoading(true);
    const messageId = addPendingMessage(query, selectedText);
    lastPendingMsgRef.current = messageId;

    if (selectedText) {
      onSelectedTextUsed?.();
    }

    // Send query to API
    const chatRequest: ChatRequest = {
      query,
      retrieval_scope: "full_collection",
      top_k: 5,
    };

    await sendQuery(chatRequest);
  };

  /**
   * Handle retry: resend the last failed message
   */
  const handleRetry = async () => {
    const lastErrorMsg = [...messages]
      .reverse()
      .find((msg) => msg.status === "error");

    if (lastErrorMsg) {
      setLoading(true);
      clearError();
      lastPendingMsgRef.current = lastErrorMsg.id;

      const chatRequest: ChatRequest = {
        query: lastErrorMsg.query,
        retrieval_scope: "full_collection",
        top_k: 5,
      };

      await sendQuery(chatRequest);
    }
  };

  return (
    <div className={`${styles.chatbotContainer} ${className || ""}`}>
      {/* Header */}
      <div className={styles.chatbotHeader}>
        <h3 className={styles.headerTitle}>Textbook Assistant</h3>
        {onClose && (
          <button
            className={styles.closeButton}
            onClick={onClose}
            title="Close chatbot"
            aria-label="Close chatbot"
          >
            <X size={20} strokeWidth={3} />
          </button>
        )}
      </div>

      {/* Messages Container */}
      <div className={styles.messagesContainer}>
        {messages.length === 0 && !error && (
          <div className={styles.emptyState}>
            <p>Ask me anything about the textbook...</p>
          </div>
        )}

        {messages.map((msg) => (
          <ChatMessage key={msg.id} message={msg} />
        ))}

        <div ref={messagesEndRef} />
      </div>

      {/* Error Display */}
      {error && (
        <ErrorDisplay
          message={error}
          onRetry={handleRetry}
          onDismiss={clearError}
        />
      )}

      {/* Input Form */}
      <form className={styles.inputForm} onSubmit={handleSubmit}>
        <textarea
          ref={inputRef}
          value={inputValue}
          onChange={(e) => setInputValue(e.target.value)}
          onKeyDown={(e) => {
            // Send message on Enter, Shift+Enter for new line
            if (e.key === "Enter" && !e.shiftKey) {
              e.preventDefault();
              handleSubmit(e as any);
            }
          }}
          className={styles.inputField}
          placeholder="Ask a question about the textbook..."
          disabled={isLoading}
          rows={2}
          aria-label="Question input"
          maxLength={10000}
        />

        <button
          className={styles.submitButton}
          type="submit"
          disabled={isLoading || !inputValue.trim()}
          aria-label="Submit question"
        >
          {isLoading ? "Sending..." : <Send size={20} strokeWidth={1.5} />}
        </button>
      </form>
    </div>
  );
};

export default ChatBot;
