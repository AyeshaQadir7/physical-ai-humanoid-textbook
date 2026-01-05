/**
 * LoadingSpinner Component
 *
 * Displays an animated spinner while the chatbot is waiting for an API response.
 */

import React from 'react';
import styles from './ChatBot.module.css';

export interface LoadingSpinnerProps {
  /** Optional custom message (default: "Waiting for response...") */
  message?: string;
}

/**
 * LoadingSpinner Component
 *
 * Shows a spinning animation with a loading message.
 * Used to indicate that the API request is in progress.
 */
export const LoadingSpinner: React.FC<LoadingSpinnerProps> = ({
  message = 'Waiting for response...',
}) => {
  return (
    <div className={styles.loadingContainer}>
      <div className={styles.spinner} aria-label="Loading">
        <div className={styles.spinnerCircle} />
      </div>
      <p className={styles.loadingText}>{message}</p>
    </div>
  );
};
