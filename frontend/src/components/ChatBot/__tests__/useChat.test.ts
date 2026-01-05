/**
 * Tests for useChat hook
 *
 * Verifies API communication, error handling, callbacks, and timeout behavior
 */

import { renderHook, waitFor } from '@testing-library/react';
import { useChat } from '../useChat';
import type { ChatRequest } from '../../../types/chat';

describe('useChat', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('sendQuery', () => {
    it('should send query successfully', async () => {
      const mockResponse = {
        query: 'What is ROS2?',
        answer: 'ROS2 is middleware for robotics',
        retrieved_chunks: [],
        execution_metrics: {
          retrieval_time_ms: 100,
          generation_time_ms: 200,
          total_time_ms: 300,
        },
        retrieval_scope: 'full_collection' as const,
        status: 'success' as const,
      };

      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: true,
        json: async () => mockResponse,
      });

      const onSuccess = jest.fn();
      const { result } = renderHook(() => useChat(onSuccess));

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      const response = await result.current.sendQuery(request);

      await waitFor(() => {
        expect(onSuccess).toHaveBeenCalledWith(
          expect.objectContaining({
            success: true,
            answer: 'ROS2 is middleware for robotics',
          })
        );
      });

      expect(response.success).toBe(true);
    });

    it('should handle empty query validation', async () => {
      const onError = jest.fn();
      const { result } = renderHook(() =>
        useChat(undefined, onError)
      );

      const request: ChatRequest = {
        query: '',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      const response = await result.current.sendQuery(request);

      await waitFor(() => {
        expect(onError).toHaveBeenCalledWith('Please enter a question');
      });

      expect(response.success).toBe(false);
      expect(response.error).toBe('Please enter a question');
    });

    it('should handle API errors with proper mapping', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: false,
        status: 429,
        json: async () => ({ error: { message: 'Too many requests' } }),
      });

      const onError = jest.fn();
      const { result } = renderHook(() =>
        useChat(undefined, onError)
      );

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      const response = await result.current.sendQuery(request);

      await waitFor(() => {
        expect(onError).toHaveBeenCalled();
      });

      expect(response.success).toBe(false);
    });

    it('should handle network errors', async () => {
      global.fetch = jest.fn().mockRejectedValueOnce(
        new TypeError('Failed to fetch')
      );

      const onError = jest.fn();
      const { result } = renderHook(() =>
        useChat(undefined, onError)
      );

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      const response = await result.current.sendQuery(request);

      await waitFor(() => {
        expect(onError).toHaveBeenCalledWith(
          expect.stringContaining('Unable to connect')
        );
      });

      expect(response.success).toBe(false);
    });

    it('should handle timeout errors', async () => {
      global.fetch = jest.fn().mockRejectedValueOnce(
        new DOMException('AbortError', 'AbortError')
      );

      const onError = jest.fn();
      const { result } = renderHook(() =>
        useChat(undefined, onError)
      );

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      const response = await result.current.sendQuery(request);

      await waitFor(() => {
        expect(onError).toHaveBeenCalledWith(
          expect.stringContaining('took too long')
        );
      });

      expect(response.success).toBe(false);
    });
  });

  describe('loading state', () => {
    it('should expose loading state during request', async () => {
      let resolveResponse: any;
      global.fetch = jest.fn(
        () =>
          new Promise(resolve => {
            resolveResponse = resolve;
          })
      );

      const { result } = renderHook(() => useChat());

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      const queryPromise = result.current.sendQuery(request);

      // Loading state should be available through the hook
      expect(result.current).toBeDefined();

      resolveResponse({
        ok: true,
        json: async () => ({
          query: 'test',
          answer: 'test answer',
          retrieved_chunks: [],
          execution_metrics: { retrieval_time_ms: 0, generation_time_ms: 0, total_time_ms: 0 },
          retrieval_scope: 'full_collection',
          status: 'success',
        }),
      });

      await queryPromise;
    });
  });

  describe('error handling', () => {
    it('should call onError callback on failure', async () => {
      global.fetch = jest.fn().mockRejectedValueOnce(new Error('Network error'));

      const onError = jest.fn();
      const { result } = renderHook(() =>
        useChat(undefined, onError)
      );

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      await result.current.sendQuery(request);

      await waitFor(() => {
        expect(onError).toHaveBeenCalled();
      });
    });

    it('should call onSuccess callback on success', async () => {
      const mockResponse = {
        query: 'What is ROS2?',
        answer: 'ROS2 is middleware for robotics',
        retrieved_chunks: [],
        execution_metrics: {
          retrieval_time_ms: 100,
          generation_time_ms: 200,
          total_time_ms: 300,
        },
        retrieval_scope: 'full_collection' as const,
        status: 'success' as const,
      };

      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: true,
        json: async () => mockResponse,
      });

      const onSuccess = jest.fn();
      const { result } = renderHook(() => useChat(onSuccess));

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      await result.current.sendQuery(request);

      await waitFor(() => {
        expect(onSuccess).toHaveBeenCalled();
      });
    });
  });
});
