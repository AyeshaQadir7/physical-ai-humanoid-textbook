/**
 * ChatMessage Component
 *
 * Displays a single query-response pair in the conversation.
 * Shows:
 * - User query (right-aligned or highlighted)
 * - AI response (left-aligned with different styling)
 * - Loading spinner (while waiting for response)
 * - Error message (if request failed)
 * - Source citations (retrieved chunks with links)
 */

import React from "react";
import type { ChatMessage as ChatMessageType } from "../../types/chat";
import { LoadingSpinner } from "./LoadingSpinner";
import { SourceCitation } from "./SourceCitation";
import styles from "./ChatBot.module.css";

export interface ChatMessageProps {
  /** The message to display */
  message: ChatMessageType;
  /** Optional CSS class name */
  className?: string;
}

/**
 * ChatMessage Component
 *
 * Renders a complete message pair: user query + AI response + sources.
 * Handles different message states: pending, success, error.
 */
export const ChatMessage: React.FC<ChatMessageProps> = ({
  message,
  className,
}) => {
  return (
    <div className={`${styles.messageContainer} ${className || ""}`}>
      {/* User Query */}
      <div className={styles.userMessage}>
        <strong>You:</strong> {message.query}
      </div>

      {/* AI Response - different states */}
      {message.status === "pending" && (
        <div className={styles.assistantMessage}>
          <LoadingSpinner />
        </div>
      )}

      {message.status === "success" && message.answer && (
        <>
          <div className={styles.assistantMessage}>
            <strong>Assistant:</strong> {message.answer}
          </div>

          {/* Source Citations */}
          {message.retrieved_chunks.length > 0 && (
            <div className={styles.sourcesContainer}>
              <div className={styles.sourcesTitle}>Sources:</div>
              <div className={styles.sourcesList}>
                {message.retrieved_chunks.map((chunk, index) => (
                  <SourceCitation key={chunk.chunk_id || index} chunk={chunk} />
                ))}
              </div>
            </div>
          )}

          {/* No sources message */}
          {message.retrieved_chunks.length === 0 && (
            <div className={styles.noSourcesMessage}>
              No relevant sources found in the textbook.
            </div>
          )}
        </>
      )}

      {message.status === "error" && (
        <div className={styles.errorMessage}>
          <strong>Error:</strong> {message.error || "Failed to get response"}
        </div>
      )}
    </div>
  );
};
