/**
 * SourceCitation Component
 *
 * Displays a source reference (retrieved chunk) with:
 * - Text excerpt from the source
 * - Similarity score (relevance)
 * - Clickable link to the source page
 * - Breadcrumb path through the document
 */

import React from 'react';
import type { RetrievedChunk } from '../../types/chat';
import styles from './ChatBot.module.css';

export interface SourceCitationProps {
  /** The retrieved chunk to display */
  chunk: RetrievedChunk;
  /** Optional CSS class name */
  className?: string;
}

/**
 * SourceCitation Component
 *
 * Renders a single source reference with text excerpt, relevance score,
 * and clickable link to the source page.
 */
export const SourceCitation: React.FC<SourceCitationProps> = ({
  chunk,
  className,
}) => {
  // Truncate very long text excerpts
  const truncatedText =
    chunk.text.length > 200 ? chunk.text.substring(0, 200) + '...' : chunk.text;

  // Format similarity score as percentage
  const scorePercentage = Math.round(chunk.similarity_score * 100);

  return (
    <div className={`${styles.sourceChunk} ${className || ''}`}>
      <div className={styles.chunkText}>
        <strong>Source:</strong> {truncatedText}
      </div>

      <div className={styles.chunkMetadata}>
        <div className={styles.relevanceScore}>
          Relevance: <span className={styles.scoreValue}>{scorePercentage}%</span>
        </div>

        <a
          href={chunk.source_url}
          target="_blank"
          rel="noopener noreferrer"
          className={styles.sourceLink}
          title={`Link to ${chunk.page_title}`}
        >
          {chunk.page_title}
        </a>
      </div>

      {chunk.section_headers && chunk.section_headers.length > 0 && (
        <div className={styles.breadcrumb}>
          {chunk.section_headers.join(' > ')}
        </div>
      )}
    </div>
  );
};
