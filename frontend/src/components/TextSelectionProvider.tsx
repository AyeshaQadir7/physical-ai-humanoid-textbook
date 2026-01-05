/**
 * TextSelectionProvider Component
 *
 * Provides text selection detection and integration with ChatBot.
 * Wraps the book content and manages the text selection menu.
 */

import React, { useState, useCallback } from 'react';
import { useTextSelection, type SelectionInfo } from './ChatBot/useTextSelection';
import TextSelectionMenu from './ChatBot/TextSelectionMenu';

export interface TextSelectionProviderProps {
  children: React.ReactNode;
  onAskAboutText?: (text: string) => void;
}

/**
 * TextSelectionProvider Component
 *
 * Wraps the main content area and provides text selection detection.
 * Shows a floating menu when text is selected for asking the chatbot.
 *
 * @example
 * ```tsx
 * <TextSelectionProvider onAskAboutText={(text) => chatBot.preFill(text)}>
 *   <main>Book content here</main>
 * </TextSelectionProvider>
 * ```
 */
export const TextSelectionProvider: React.FC<TextSelectionProviderProps> = ({
  children,
  onAskAboutText,
}) => {
  const [selectedInfo, setSelectedInfo] = useState<SelectionInfo | null>(null);

  const handleSelection = useCallback(
    (info: SelectionInfo) => {
      setSelectedInfo(info);
    },
    []
  );

  const handleClear = useCallback(() => {
    setSelectedInfo(null);
  }, []);

  const { clearSelection } = useTextSelection({
    minSelectionLength: 5,
    onSelection: handleSelection,
    onClear: handleClear,
  });

  const handleAskAboutSelection = useCallback(
    (text: string) => {
      onAskAboutText?.(text);
      clearSelection();
      setSelectedInfo(null);
    },
    [onAskAboutText, clearSelection]
  );

  return (
    <>
      {children}
      {selectedInfo && (
        <TextSelectionMenu
          selectedText={selectedInfo.text}
          position={selectedInfo}
          onAskAboutSelection={handleAskAboutSelection}
          onDismiss={handleClear}
        />
      )}
    </>
  );
};

export default TextSelectionProvider;
