/**
 * ErrorDisplay Component
 *
 * Shows error messages when API requests fail.
 * Provides retry and dismiss actions to the user.
 */

import React from 'react';
import styles from './ChatBot.module.css';

export interface ErrorDisplayProps {
  /** The error message to display */
  message: string;
  /** Callback when user clicks retry */
  onRetry?: () => void;
  /** Callback when user dismisses the error */
  onDismiss?: () => void;
}

/**
 * ErrorDisplay Component
 *
 * Shows an error message with retry and dismiss buttons.
 * Used when API requests fail due to network, timeout, or server errors.
 */
export const ErrorDisplay: React.FC<ErrorDisplayProps> = ({
  message,
  onRetry,
  onDismiss,
}) => {
  return (
    <div className={styles.errorContainer} role="alert">
      <div className={styles.errorContent}>
        <strong>Error:</strong> {message}
      </div>
      <div className={styles.errorActions}>
        {onRetry && (
          <button className={styles.retryButton} onClick={onRetry}>
            Retry
          </button>
        )}
        {onDismiss && (
          <button className={styles.dismissButton} onClick={onDismiss}>
            Dismiss
          </button>
        )}
      </div>
    </div>
  );
};
