/**
 * TextSelectionMenu Component
 *
 * Displays a context menu or floating button when text is selected.
 * Allows users to ask the chatbot about the selected text.
 */

import React, { useEffect, useRef } from 'react';
import styles from './ChatBot.module.css';

/**
 * Props for TextSelectionMenu
 */
export interface TextSelectionMenuProps {
  selectedText: string;
  position: { x: number; y: number };
  onAskAboutSelection: (text: string) => void;
  onDismiss: () => void;
}

/**
 * TextSelectionMenu Component
 *
 * Renders a floating menu near selected text with "Ask about this selection" option.
 *
 * @param props Component props
 * @returns Menu element or null if not visible
 *
 * @example
 * <TextSelectionMenu
 *   selectedText="ROS2 is middleware"
 *   position={{ x: 200, y: 300 }}
 *   onAskAboutSelection={(text) => chatBot.prefilWith(text)}
 *   onDismiss={() => clearSelection()}
 * />
 */
export const TextSelectionMenu: React.FC<TextSelectionMenuProps> = ({
  selectedText,
  position,
  onAskAboutSelection,
  onDismiss,
}) => {
  const menuRef = useRef<HTMLDivElement>(null);

  /**
   * Truncate selection preview to 50 chars
   */
  const getPreviewText = (): string => {
    const maxLength = 50;
    if (selectedText.length > maxLength) {
      return selectedText.substring(0, maxLength) + '...';
    }
    return selectedText;
  };

  /**
   * Handle click outside menu to dismiss
   */
  useEffect(() => {
    const handleClickOutside = (event: MouseEvent) => {
      if (menuRef.current && !menuRef.current.contains(event.target as Node)) {
        onDismiss();
      }
    };

    document.addEventListener('mousedown', handleClickOutside);
    return () => {
      document.removeEventListener('mousedown', handleClickOutside);
    };
  }, [onDismiss]);

  /**
   * Handle Escape key to dismiss
   */
  useEffect(() => {
    const handleKeyDown = (event: KeyboardEvent) => {
      if (event.key === 'Escape') {
        onDismiss();
      }
    };

    document.addEventListener('keydown', handleKeyDown);
    return () => {
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [onDismiss]);

  /**
   * Clamp position to viewport
   */
  const getClampedPosition = (): React.CSSProperties => {
    let x = position.x;
    let y = position.y;

    const menuWidth = 300; // Approximate menu width
    const menuHeight = 60; // Approximate menu height
    const padding = 10;

    // Clamp horizontal
    if (x + menuWidth / 2 > window.innerWidth - padding) {
      x = window.innerWidth - menuWidth / 2 - padding;
    }
    if (x - menuWidth / 2 < padding) {
      x = menuWidth / 2 + padding;
    }

    // Clamp vertical
    if (y + menuHeight > window.innerHeight - padding) {
      y = window.innerHeight - menuHeight - padding;
    }
    if (y < padding) {
      y = padding;
    }

    return {
      position: 'fixed',
      left: `${x}px`,
      top: `${y}px`,
      transform: 'translate(-50%, 0)',
    };
  };

  return (
    <div
      ref={menuRef}
      className={styles.textSelectionMenu}
      style={getClampedPosition()}
      role="menu"
      aria-label="Text selection menu"
    >
      <button
        className={styles.textSelectionButton}
        onClick={() => {
          onAskAboutSelection(selectedText);
          onDismiss();
        }}
        role="menuitem"
        aria-label={`Ask about "${getPreviewText()}"`}
        title={`Ask the chatbot about: ${selectedText}`}
      >
        <span className={styles.buttonIcon}>💬</span>
        <span className={styles.buttonText}>
          Ask: <em>{getPreviewText()}</em>
        </span>
      </button>
    </div>
  );
};

export default TextSelectionMenu;
