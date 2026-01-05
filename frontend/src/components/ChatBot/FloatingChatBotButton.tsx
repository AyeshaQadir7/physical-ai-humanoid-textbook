/**
 * Floating ChatBot Button Component
 *
 * Displays a floating action button in the bottom-right corner that opens the chatbot.
 * Only visible when the chatbot is closed.
 *
 * Features:
 * - Cyberpunk aesthetic with electric cyan color
 * - Glow effects on hover
 * - Scale animation for tactile feedback
 * - Responsive positioning
 */

import React from "react";
import { BotMessageSquare } from "lucide-react";
import styles from "./FloatingChatBotButton.module.css";

export interface FloatingChatBotButtonProps {
  /** Callback when button is clicked to open chatbot */
  onOpen: () => void;
}

/**
 * FloatingChatBotButton Component
 *
 * Renders a floating circular button with a chatbot icon that opens the chatbot
 * when clicked. Positioned in the bottom-right corner with cyberpunk styling.
 *
 * @param onOpen Callback function to open the chatbot
 * @returns React element
 */
export const FloatingChatBotButton: React.FC<FloatingChatBotButtonProps> = ({
  onOpen,
}) => {
  return (
    <button
      className={styles.floatingButton}
      onClick={onOpen}
      title="Open chatbot"
      aria-label="Open chatbot"
    >
      <BotMessageSquare size={36} color="#ffffff" strokeWidth={1.25} />
    </button>
  );
};

export default FloatingChatBotButton;
