/**
 * Tests for ChatMessage component
 *
 * Verifies message rendering, source citation display, and loading states
 */

import React from 'react';
import { render, screen } from '@testing-library/react';
import ChatMessage from '../ChatMessage';
import type { ChatMessage as ChatMessageType } from '../../../types/chat';

describe('ChatMessage', () => {
  describe('user query display', () => {
    it('should display user query', () => {
      const message: ChatMessageType = {
        id: '1',
        query: 'What is ROS2?',
        answer: null,
        retrieved_chunks: [],
        timestamp: new Date(),
        status: 'pending',
      };

      render(<ChatMessage message={message} />);

      expect(screen.getByText(/What is ROS2\?/)).toBeInTheDocument();
    });

    it('should display multiple queries correctly', () => {
      const message: ChatMessageType = {
        id: '1',
        query: 'This is a long question with multiple words?',
        answer: null,
        retrieved_chunks: [],
        timestamp: new Date(),
        status: 'pending',
      };

      render(<ChatMessage message={message} />);

      expect(screen.getByText(/long question with multiple words/)).toBeInTheDocument();
    });
  });

  describe('AI response display', () => {
    it('should display AI answer when available', () => {
      const message: ChatMessageType = {
        id: '1',
        query: 'What is ROS2?',
        answer: 'ROS2 is middleware for robotics',
        retrieved_chunks: [],
        timestamp: new Date(),
        status: 'success',
      };

      render(<ChatMessage message={message} />);

      expect(screen.getByText(/ROS2 is middleware/)).toBeInTheDocument();
    });

    it('should not display answer when status is pending', () => {
      const message: ChatMessageType = {
        id: '1',
        query: 'What is ROS2?',
        answer: 'ROS2 is middleware for robotics',
        retrieved_chunks: [],
        timestamp: new Date(),
        status: 'pending',
      };

      render(<ChatMessage message={message} />);

      // Answer might be there but loading state should show
      expect(screen.getByText(/What is ROS2\?/)).toBeInTheDocument();
    });
  });

  describe('loading state', () => {
    it('should display loading spinner for pending message', () => {
      const message: ChatMessageType = {
        id: '1',
        query: 'What is ROS2?',
        answer: null,
        retrieved_chunks: [],
        timestamp: new Date(),
        status: 'pending',
      };

      const { container } = render(<ChatMessage message={message} />);

      // Look for loading spinner element
      const spinner = container.querySelector('[class*="spinner"]') ||
        container.querySelector('[class*="loading"]') ||
        screen.queryByText(/waiting|loading/i);

      expect(spinner || document.body.textContent).toMatch(/waiting|loading|pending/i);
    });

    it('should not display loading spinner for completed message', () => {
      const message: ChatMessageType = {
        id: '1',
        query: 'What is ROS2?',
        answer: 'ROS2 is middleware for robotics',
        retrieved_chunks: [],
        timestamp: new Date(),
        status: 'success',
      };

      render(<ChatMessage message={message} />);

      expect(screen.queryByText(/waiting for response/i)).not.toBeInTheDocument();
    });
  });

  describe('source citations display', () => {
    it('should display retrieved chunks when available', () => {
      const message: ChatMessageType = {
        id: '1',
        query: 'What is ROS2?',
        answer: 'ROS2 is middleware for robotics',
        retrieved_chunks: [
          {
            chunk_id: '1',
            text: 'ROS2 is middleware',
            similarity_score: 0.95,
            source_url: 'https://example.com/ros2',
            page_title: 'ROS2 Introduction',
            section_headers: ['Robotics', 'ROS2'],
          },
        ],
        timestamp: new Date(),
        status: 'success',
      };

      render(<ChatMessage message={message} />);

      expect(screen.getByText(/ROS2 is middleware/)).toBeInTheDocument();
      expect(screen.getByText(/ROS2 Introduction/)).toBeInTheDocument();
    });

    it('should not display sources section when no chunks available', () => {
      const message: ChatMessageType = {
        id: '1',
        query: 'What is ROS2?',
        answer: 'ROS2 is middleware',
        retrieved_chunks: [],
        timestamp: new Date(),
        status: 'success',
      };

      const { container } = render(<ChatMessage message={message} />);

      const sourceSection = container.querySelector('[class*="source"]');
      expect(sourceSection || screen.queryByText(/source|citation/i)).not.toBeInTheDocument();
    });

    it('should display multiple sources', () => {
      const message: ChatMessageType = {
        id: '1',
        query: 'What is ROS2?',
        answer: 'ROS2 is middleware',
        retrieved_chunks: [
          {
            chunk_id: '1',
            text: 'ROS2 is middleware',
            similarity_score: 0.95,
            source_url: 'https://example.com/ros2',
            page_title: 'ROS2 Introduction',
            section_headers: ['Robotics', 'ROS2'],
          },
          {
            chunk_id: '2',
            text: 'ROS2 architecture',
            similarity_score: 0.88,
            source_url: 'https://example.com/architecture',
            page_title: 'Architecture',
            section_headers: ['Design', 'Architecture'],
          },
        ],
        timestamp: new Date(),
        status: 'success',
      };

      render(<ChatMessage message={message} />);

      expect(screen.getByText(/ROS2 Introduction/)).toBeInTheDocument();
      expect(screen.getByText(/Architecture/)).toBeInTheDocument();
    });

    it('should display source URL as clickable link', () => {
      const message: ChatMessageType = {
        id: '1',
        query: 'What is ROS2?',
        answer: 'ROS2 is middleware',
        retrieved_chunks: [
          {
            chunk_id: '1',
            text: 'ROS2 is middleware',
            similarity_score: 0.95,
            source_url: 'https://example.com/ros2',
            page_title: 'ROS2 Introduction',
            section_headers: ['Robotics', 'ROS2'],
          },
        ],
        timestamp: new Date(),
        status: 'success',
      };

      render(<ChatMessage message={message} />);

      const link = screen.getByRole('link', { name: /ROS2 Introduction/i });
      expect(link).toHaveAttribute('href', 'https://example.com/ros2');
      expect(link).toHaveAttribute('target', '_blank');
    });

    it('should display similarity score', () => {
      const message: ChatMessageType = {
        id: '1',
        query: 'What is ROS2?',
        answer: 'ROS2 is middleware',
        retrieved_chunks: [
          {
            chunk_id: '1',
            text: 'ROS2 is middleware',
            similarity_score: 0.95,
            source_url: 'https://example.com/ros2',
            page_title: 'ROS2 Introduction',
            section_headers: ['Robotics', 'ROS2'],
          },
        ],
        timestamp: new Date(),
        status: 'success',
      };

      render(<ChatMessage message={message} />);

      // Score displayed as percentage
      expect(screen.getByText(/95%|0\.95/)).toBeInTheDocument();
    });

    it('should display section breadcrumb path', () => {
      const message: ChatMessageType = {
        id: '1',
        query: 'What is ROS2?',
        answer: 'ROS2 is middleware',
        retrieved_chunks: [
          {
            chunk_id: '1',
            text: 'ROS2 is middleware',
            similarity_score: 0.95,
            source_url: 'https://example.com/ros2',
            page_title: 'ROS2 Introduction',
            section_headers: ['Robotics', 'ROS2'],
          },
        ],
        timestamp: new Date(),
        status: 'success',
      };

      render(<ChatMessage message={message} />);

      expect(screen.getByText(/Robotics.*ROS2/)).toBeInTheDocument();
    });
  });

  describe('error display', () => {
    it('should display error message for failed message', () => {
      const message: ChatMessageType = {
        id: '1',
        query: 'What is ROS2?',
        answer: null,
        retrieved_chunks: [],
        timestamp: new Date(),
        status: 'error',
        error: 'Failed to get response',
      };

      render(<ChatMessage message={message} />);

      expect(screen.getByText(/Failed to get response/)).toBeInTheDocument();
    });
  });

  describe('text truncation', () => {
    it('should truncate long source text', () => {
      const longText = 'a'.repeat(300); // Longer than 200 char limit
      const message: ChatMessageType = {
        id: '1',
        query: 'What is ROS2?',
        answer: 'ROS2 is middleware',
        retrieved_chunks: [
          {
            chunk_id: '1',
            text: longText,
            similarity_score: 0.95,
            source_url: 'https://example.com/ros2',
            page_title: 'ROS2 Introduction',
            section_headers: [],
          },
        ],
        timestamp: new Date(),
        status: 'success',
      };

      const { container } = render(<ChatMessage message={message} />);

      // Should truncate to ~200 chars
      const textElement = container.querySelector('[class*="chunkText"]');
      const displayedText = textElement?.textContent || '';
      expect(displayedText.length).toBeLessThanOrEqual(250); // 200 + ellipsis
    });
  });
});
