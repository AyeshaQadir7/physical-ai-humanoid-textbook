/**
 * useTextSelection Hook
 *
 * Detects text selection on the page and provides selection information
 * for contextual chatbot queries.
 *
 * Features:
 * - Listens to mouseup and touchend events
 * - Captures selected text via window.getSelection()
 * - Filters selections < 5 characters
 * - Returns position (x, y) for menu placement
 * - Only triggers on text within book content area
 */

import { useEffect, useRef, useCallback } from 'react';

/**
 * Represents selected text and its position on screen
 */
export interface SelectionInfo {
  text: string;
  x: number;
  y: number;
}

/**
 * Configuration for text selection behavior
 */
export interface UseTextSelectionConfig {
  minSelectionLength?: number;
  contentSelector?: string;
  onSelection?: (info: SelectionInfo) => void;
  onClear?: () => void;
}

/**
 * Result of useTextSelection hook
 */
export interface UseTextSelectionResult {
  selectedText: string | null;
  selectionPosition: { x: number; y: number } | null;
  clearSelection: () => void;
}

/**
 * Hook to detect and track text selection on the page
 *
 * @param config Configuration options for selection detection
 * @returns Selection state and clear function
 *
 * @example
 * const { selectedText, selectionPosition, clearSelection } = useTextSelection({
 *   minSelectionLength: 5,
 *   onSelection: (info) => console.log('Selected:', info.text),
 * });
 */
export function useTextSelection(
  config: UseTextSelectionConfig = {}
): UseTextSelectionResult {
  const {
    minSelectionLength = 5,
    contentSelector = 'main, [role="main"], .docusaurus-mt-lg',
    onSelection,
    onClear,
  } = config;

  const selectedTextRef = useRef<string | null>(null);
  const selectionPositionRef = useRef<{ x: number; y: number } | null>(null);

  /**
   * Check if selection is within content area
   */
  const isInContentArea = useCallback((range: Range): boolean => {
    try {
      const contentArea = document.querySelector(contentSelector);
      if (!contentArea) return true; // Allow if content area not found

      return contentArea.contains(range.commonAncestorContainer);
    } catch {
      return true; // Allow on error
    }
  }, [contentSelector]);

  /**
   * Get position for selection menu (below selection)
   */
  const getSelectionPosition = useCallback((): { x: number; y: number } => {
    const selection = window.getSelection();
    if (!selection || selection.rangeCount === 0) {
      return { x: 0, y: 0 };
    }

    const range = selection.getRangeAt(0);
    const rect = range.getBoundingClientRect();

    return {
      x: rect.left + rect.width / 2, // Center horizontally on selection
      y: rect.bottom + 8, // 8px below selection
    };
  }, []);

  /**
   * Handle selection events (mouseup, touchend)
   */
  const handleSelectionEvent = useCallback(() => {
    const selection = window.getSelection();

    if (!selection || selection.rangeCount === 0) {
      // No selection
      selectedTextRef.current = null;
      selectionPositionRef.current = null;
      onClear?.();
      return;
    }

    const range = selection.getRangeAt(0);
    const text = selection.toString().trim();

    // Check minimum length and content area
    if (text.length < minSelectionLength || !isInContentArea(range)) {
      selectedTextRef.current = null;
      selectionPositionRef.current = null;
      onClear?.();
      return;
    }

    // Valid selection
    selectedTextRef.current = text;
    selectionPositionRef.current = getSelectionPosition();

    onSelection?.({
      text,
      x: selectionPositionRef.current.x,
      y: selectionPositionRef.current.y,
    });
  }, [minSelectionLength, isInContentArea, getSelectionPosition, onSelection, onClear]);

  /**
   * Clear selection
   */
  const clearSelection = useCallback(() => {
    selectedTextRef.current = null;
    selectionPositionRef.current = null;
    window.getSelection()?.removeAllRanges();
    onClear?.();
  }, [onClear]);

  // Set up event listeners
  useEffect(() => {
    document.addEventListener('mouseup', handleSelectionEvent);
    document.addEventListener('touchend', handleSelectionEvent);

    return () => {
      document.removeEventListener('mouseup', handleSelectionEvent);
      document.removeEventListener('touchend', handleSelectionEvent);
    };
  }, [handleSelectionEvent]);

  return {
    selectedText: selectedTextRef.current,
    selectionPosition: selectionPositionRef.current,
    clearSelection,
  };
}

/**
 * Hook to detect and manage text selection with React state
 * (Stateful version for use in components)
 */
export function useTextSelectionState(
  config: UseTextSelectionConfig = {}
): UseTextSelectionResult & {
  forceUpdate: () => void;
} {
  const result = useTextSelection(config);
  const [, forceUpdate] = require('react').useState(0);

  return {
    ...result,
    forceUpdate: () => {
      forceUpdate(n => n + 1);
    },
  };
}
