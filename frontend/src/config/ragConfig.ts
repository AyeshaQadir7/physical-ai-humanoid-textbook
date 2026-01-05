/**
 * RAG Agent Configuration
 *
 * Reads the RAG_AGENT_URL from build-time environment variables
 * and provides a singleton configuration object for the chatbot.
 *
 * Environment Variables:
 * - REACT_APP_RAG_AGENT_URL: Full URL to RAG Agent API endpoint (e.g., http://localhost:8000/chat)
 *   Default: http://localhost:8000/chat
 *
 * Configuration Injection:
 * - At build time, Docusaurus reads process.env.REACT_APP_RAG_AGENT_URL
 * - This module provides the URL to client-side code
 * - In development, reads from .env.local
 * - In production, must be set via CI/CD or deployment platform
 */

/**
 * Get the RAG Agent API URL from the environment.
 *
 * This function attempts to read from:
 * 1. Injected global variable __RAG_AGENT_URL__ (if available)
 * 2. Process environment variable REACT_APP_RAG_AGENT_URL (development)
 * 3. Fallback default: http://localhost:8000/chat
 *
 * @returns The RAG Agent API URL
 */
function getRagAgentUrl(): string {
  // Try to read from injected global (production build via Docusaurus clientModules)
  if (typeof window !== 'undefined' && (window as any).__RAG_AGENT_URL__) {
    return (window as any).__RAG_AGENT_URL__;
  }

  // Fallback to environment variable (development)
  if (typeof process !== 'undefined' && process.env.REACT_APP_RAG_AGENT_URL) {
    return process.env.REACT_APP_RAG_AGENT_URL;
  }

  // Default fallback for local development
  return 'http://localhost:8000/chat';
}

/**
 * RAG Configuration Object
 *
 * Exported configuration that the chatbot components use to communicate with the API.
 * All values are determined at build time and are immutable at runtime.
 */
export const ragConfig = {
  /** The RAG Agent API endpoint URL (e.g., http://localhost:8000/chat) */
  apiUrl: getRagAgentUrl(),

  /** Default timeout for API requests in milliseconds */
  requestTimeout: 30000,

  /** Maximum number of chunks to retrieve per query */
  maxChunks: 10,

  /** Minimum text length to trigger context menu (characters) */
  minSelectionLength: 5,

  /** Maximum query length in characters */
  maxQueryLength: 10000,

  /** Maximum context text length in characters */
  maxContextTextLength: 50000,
} as const;

/**
 * Type definition for configuration (for TypeScript inference)
 */
export type RagConfig = typeof ragConfig;

/**
 * Log configuration in development mode (for debugging)
 */
if (process.env.NODE_ENV !== 'production') {
  console.log('[RAG Config] Initialized with:', {
    apiUrl: ragConfig.apiUrl,
    timeout: ragConfig.requestTimeout,
  });
}
