/**
 * Root Component
 *
 * Docusaurus v3 theme wrapper that adds the ChatBot to every page.
 * This component wraps the entire site and provides the chatbot UI.
 *
 * How it works:
 * - Docusaurus automatically loads ./src/theme/Root.tsx as a theme wrapper
 * - The Root component receives {children} which is the entire site content
 * - We wrap it with the ChatBot to make it available globally
 *
 * Reference: https://docusaurus.io/docs/api/themes/theme-classic#root-component
 */

import React, { useState } from 'react';
import ChatBot from '../components/ChatBot/ChatBot';
import FloatingChatBotButton from '../components/ChatBot/FloatingChatBotButton';
import type { ReactNode } from 'react';

/**
 * Root Component Props
 */
export interface RootProps {
  children?: ReactNode;
}

/**
 * Root Component
 *
 * Wraps the entire Docusaurus site with the RAG ChatBot.
 * The chatbot appears as a persistent sidebar (desktop) or modal (mobile)
 * on every page of the textbook.
 *
 * @param children The site content from Docusaurus
 * @returns The site content with ChatBot overlaid
 *
 * @example
 * This is automatically used by Docusaurus as the root theme wrapper.
 * Users don't need to manually import or use this component.
 */
export default function Root({ children }: RootProps): React.ReactElement {
  const [isChatBotOpen, setIsChatBotOpen] = useState(true);

  return (
    <>
      {/* Site content from Docusaurus */}
      {children}

      {/* ChatBot overlay - appears on top of all pages */}
      {isChatBotOpen && (
        <ChatBot onClose={() => setIsChatBotOpen(false)} />
      )}

      {/* Floating chatbot button - appears when chatbot is closed */}
      {!isChatBotOpen && (
        <FloatingChatBotButton onOpen={() => setIsChatBotOpen(true)} />
      )}
    </>
  );
}
