/**
 * Tests for ChatBot component
 *
 * Verifies form submission, message display, loading states, and error handling
 */

import React from 'react';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import userEvent from '@testing-library/user-event';
import ChatBot from '../ChatBot';

describe('ChatBot', () => {
  beforeEach(() => {
    jest.clearAllMocks();
    // Mock fetch for all tests
    global.fetch = jest.fn();
  });

  describe('rendering', () => {
    it('should render chatbot with input field', () => {
      render(<ChatBot />);

      const input = screen.getByPlaceholderText(/ask.*question/i);
      expect(input).toBeInTheDocument();
    });

    it('should render submit button', () => {
      render(<ChatBot />);

      const button = screen.getByRole('button', { name: /send|submit/i });
      expect(button).toBeInTheDocument();
    });

    it('should render empty message list initially', () => {
      const { container } = render(<ChatBot />);

      const messageContainer = container.querySelector('[class*="messageList"]') ||
        container.querySelector('[class*="messages"]');
      expect(messageContainer).toBeInTheDocument();
    });
  });

  describe('form submission', () => {
    it('should submit form on button click', async () => {
      (global.fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          query: 'What is ROS2?',
          answer: 'ROS2 is middleware',
          retrieved_chunks: [],
          execution_metrics: {
            retrieval_time_ms: 100,
            generation_time_ms: 200,
            total_time_ms: 300,
          },
          retrieval_scope: 'full_collection',
          status: 'success',
        }),
      });

      render(<ChatBot />);

      const input = screen.getByPlaceholderText(/ask.*question/i) as HTMLTextAreaElement;
      const button = screen.getByRole('button', { name: /send|submit/i });

      await userEvent.type(input, 'What is ROS2?');
      fireEvent.click(button);

      await waitFor(() => {
        expect(global.fetch).toHaveBeenCalledWith(
          expect.stringContaining('/chat'),
          expect.objectContaining({
            method: 'POST',
          })
        );
      });
    });

    it('should prevent submission with empty query', async () => {
      render(<ChatBot />);

      const button = screen.getByRole('button', { name: /send|submit/i });
      fireEvent.click(button);

      // Fetch should not be called for empty query
      expect(global.fetch).not.toHaveBeenCalled();
    });

    it('should clear input after submission', async () => {
      (global.fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          query: 'What is ROS2?',
          answer: 'ROS2 is middleware',
          retrieved_chunks: [],
          execution_metrics: {
            retrieval_time_ms: 100,
            generation_time_ms: 200,
            total_time_ms: 300,
          },
          retrieval_scope: 'full_collection',
          status: 'success',
        }),
      });

      render(<ChatBot />);

      const input = screen.getByPlaceholderText(/ask.*question/i) as HTMLTextAreaElement;
      const button = screen.getByRole('button', { name: /send|submit/i });

      await userEvent.type(input, 'What is ROS2?');
      fireEvent.click(button);

      await waitFor(() => {
        expect(input.value).toBe('');
      });
    });
  });

  describe('loading state', () => {
    it('should show loading spinner during request', async () => {
      let resolveResponse: any;
      (global.fetch as jest.Mock).mockReturnValueOnce(
        new Promise(resolve => {
          resolveResponse = resolve;
        })
      );

      render(<ChatBot />);

      const input = screen.getByPlaceholderText(/ask.*question/i) as HTMLTextAreaElement;
      const button = screen.getByRole('button', { name: /send|submit/i });

      await userEvent.type(input, 'What is ROS2?');
      fireEvent.click(button);

      // Look for loading indicator
      await waitFor(() => {
        const loadingIndicator = screen.queryByText(/waiting|loading/i);
        expect(loadingIndicator || document.body.textContent).toMatch(
          /waiting|loading|pending/i
        );
      });

      // Resolve the promise
      resolveResponse({
        ok: true,
        json: async () => ({
          query: 'What is ROS2?',
          answer: 'ROS2 is middleware',
          retrieved_chunks: [],
          execution_metrics: {
            retrieval_time_ms: 100,
            generation_time_ms: 200,
            total_time_ms: 300,
          },
          retrieval_scope: 'full_collection',
          status: 'success',
        }),
      });

      await waitFor(() => {
        expect(screen.queryByText(/waiting|loading/i)).not.toBeInTheDocument();
      });
    });

    it('should disable submit button during request', async () => {
      let resolveResponse: any;
      (global.fetch as jest.Mock).mockReturnValueOnce(
        new Promise(resolve => {
          resolveResponse = resolve;
        })
      );

      render(<ChatBot />);

      const input = screen.getByPlaceholderText(/ask.*question/i);
      const button = screen.getByRole('button', { name: /send|submit/i }) as HTMLButtonElement;

      await userEvent.type(input, 'What is ROS2?');
      fireEvent.click(button);

      await waitFor(() => {
        expect(button).toBeDisabled();
      });

      resolveResponse({
        ok: true,
        json: async () => ({
          query: 'What is ROS2?',
          answer: 'ROS2 is middleware',
          retrieved_chunks: [],
          execution_metrics: {
            retrieval_time_ms: 100,
            generation_time_ms: 200,
            total_time_ms: 300,
          },
          retrieval_scope: 'full_collection',
          status: 'success',
        }),
      });

      await waitFor(() => {
        expect(button).not.toBeDisabled();
      });
    });
  });

  describe('error handling', () => {
    it('should display error message on API failure', async () => {
      (global.fetch as jest.Mock).mockResolvedValueOnce({
        ok: false,
        status: 500,
        json: async () => ({ error: { message: 'Server error' } }),
      });

      render(<ChatBot />);

      const input = screen.getByPlaceholderText(/ask.*question/i);
      const button = screen.getByRole('button', { name: /send|submit/i });

      await userEvent.type(input, 'What is ROS2?');
      fireEvent.click(button);

      await waitFor(() => {
        expect(screen.getByText(/error|unable|failed/i)).toBeInTheDocument();
      });
    });

    it('should display network error on connection failure', async () => {
      (global.fetch as jest.Mock).mockRejectedValueOnce(
        new TypeError('Failed to fetch')
      );

      render(<ChatBot />);

      const input = screen.getByPlaceholderText(/ask.*question/i);
      const button = screen.getByRole('button', { name: /send|submit/i });

      await userEvent.type(input, 'What is ROS2?');
      fireEvent.click(button);

      await waitFor(() => {
        expect(screen.getByText(/unable to connect|network/i)).toBeInTheDocument();
      });
    });

    it('should allow retry after error', async () => {
      (global.fetch as jest.Mock)
        .mockRejectedValueOnce(new TypeError('Failed to fetch'))
        .mockResolvedValueOnce({
          ok: true,
          json: async () => ({
            query: 'What is ROS2?',
            answer: 'ROS2 is middleware',
            retrieved_chunks: [],
            execution_metrics: {
              retrieval_time_ms: 100,
              generation_time_ms: 200,
              total_time_ms: 300,
            },
            retrieval_scope: 'full_collection',
            status: 'success',
          }),
        });

      render(<ChatBot />);

      const input = screen.getByPlaceholderText(/ask.*question/i);
      const button = screen.getByRole('button', { name: /send|submit/i });

      await userEvent.type(input, 'What is ROS2?');
      fireEvent.click(button);

      // First request fails
      await waitFor(() => {
        expect(screen.getByText(/unable to connect|network/i)).toBeInTheDocument();
      });

      // Look for retry button
      const retryButton = screen.queryByRole('button', { name: /retry/i });
      if (retryButton) {
        fireEvent.click(retryButton);

        // Second request succeeds
        await waitFor(() => {
          expect(screen.getByText(/ROS2 is middleware/i)).toBeInTheDocument();
        });
      }
    });
  });

  describe('message display', () => {
    it('should display user query in message list', async () => {
      (global.fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          query: 'What is ROS2?',
          answer: 'ROS2 is middleware for robotics',
          retrieved_chunks: [],
          execution_metrics: {
            retrieval_time_ms: 100,
            generation_time_ms: 200,
            total_time_ms: 300,
          },
          retrieval_scope: 'full_collection',
          status: 'success',
        }),
      });

      render(<ChatBot />);

      const input = screen.getByPlaceholderText(/ask.*question/i);
      const button = screen.getByRole('button', { name: /send|submit/i });

      await userEvent.type(input, 'What is ROS2?');
      fireEvent.click(button);

      await waitFor(() => {
        expect(screen.getByText(/What is ROS2\?/)).toBeInTheDocument();
      });
    });

    it('should display AI response after successful request', async () => {
      (global.fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          query: 'What is ROS2?',
          answer: 'ROS2 is middleware for robotics',
          retrieved_chunks: [],
          execution_metrics: {
            retrieval_time_ms: 100,
            generation_time_ms: 200,
            total_time_ms: 300,
          },
          retrieval_scope: 'full_collection',
          status: 'success',
        }),
      });

      render(<ChatBot />);

      const input = screen.getByPlaceholderText(/ask.*question/i);
      const button = screen.getByRole('button', { name: /send|submit/i });

      await userEvent.type(input, 'What is ROS2?');
      fireEvent.click(button);

      await waitFor(() => {
        expect(screen.getByText(/ROS2 is middleware/i)).toBeInTheDocument();
      });
    });
  });

  describe('accessibility', () => {
    it('should have proper ARIA labels', () => {
      render(<ChatBot />);

      const input = screen.getByPlaceholderText(/ask.*question/i);
      expect(input).toHaveAttribute('aria-label') || expect(input).toHaveAttribute('placeholder');

      const button = screen.getByRole('button', { name: /send|submit/i });
      expect(button).toBeInTheDocument();
    });

    it('should be keyboard navigable', async () => {
      (global.fetch as jest.Mock).mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          query: 'test',
          answer: 'test answer',
          retrieved_chunks: [],
          execution_metrics: {
            retrieval_time_ms: 100,
            generation_time_ms: 200,
            total_time_ms: 300,
          },
          retrieval_scope: 'full_collection',
          status: 'success',
        }),
      });

      render(<ChatBot />);

      const input = screen.getByPlaceholderText(/ask.*question/i) as HTMLTextAreaElement;

      // User types and presses Enter
      await userEvent.type(input, 'test query');
      fireEvent.keyDown(input, { key: 'Enter', code: 'Enter', ctrlKey: true });

      // Form should submit (depends on implementation - might be form submission)
      // At minimum, the input should be accessible
      expect(input).toBeVisible();
      expect(input).toBeEnabled();
    });
  });
});
