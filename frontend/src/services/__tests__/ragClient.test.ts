/**
 * Tests for RagClient HTTP service
 *
 * Verifies request validation, response parsing, error handling, and timeout behavior
 */

import { RagClient, RagApiError } from '../ragClient';
import type { ChatRequest, ChatResponse } from '../../types/chat';

describe('RagClient', () => {
  let client: RagClient;
  const mockApiUrl = 'http://localhost:8000/chat';

  beforeEach(() => {
    client = new RagClient({
      apiUrl: mockApiUrl,
      timeout: 5000,
    });

    // Clear all mocks before each test
    jest.clearAllMocks();
  });

  describe('initialization', () => {
    it('should create client with valid config', () => {
      expect(client).toBeDefined();
      expect(client['apiUrl']).toBe(mockApiUrl);
      expect(client['timeout']).toBe(5000);
    });

    it('should accept custom timeout', () => {
      const customClient = new RagClient({
        apiUrl: mockApiUrl,
        timeout: 10000,
      });
      expect(customClient['timeout']).toBe(10000);
    });
  });

  describe('request validation', () => {
    it('should reject empty query', async () => {
      const request: ChatRequest = {
        query: '',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      await expect(client.sendQuery(request)).rejects.toThrow();
    });

    it('should reject query exceeding max length', async () => {
      const request: ChatRequest = {
        query: 'a'.repeat(10001), // Exceeds 10000 char limit
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      await expect(client.sendQuery(request)).rejects.toThrow();
    });

    it('should reject invalid top_k', async () => {
      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 0, // Invalid
      };

      await expect(client.sendQuery(request)).rejects.toThrow();
    });

    it('should accept valid request', async () => {
      // Mock fetch to avoid actual network call
      global.fetch = jest.fn().mockResolvedValueOnce({
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
        } as ChatResponse),
      });

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      const response = await client.sendQuery(request);
      expect(response.status).toBe('success');
      expect(response.answer).toBeDefined();
    });
  });

  describe('response validation', () => {
    it('should validate required fields in response', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          query: 'What is ROS2?',
          // Missing 'answer' field
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

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      await expect(client.sendQuery(request)).rejects.toThrow();
    });

    it('should parse valid response correctly', async () => {
      const mockResponse: ChatResponse = {
        query: 'What is ROS2?',
        answer: 'ROS2 is a middleware for robotics',
        retrieved_chunks: [
          {
            chunk_id: '1',
            text: 'ROS2 is middleware...',
            similarity_score: 0.95,
            source_url: 'https://example.com/ros2',
            page_title: 'ROS2 Introduction',
            section_headers: ['Robotics', 'ROS2'],
          },
        ],
        execution_metrics: {
          retrieval_time_ms: 150,
          generation_time_ms: 250,
          total_time_ms: 400,
        },
        retrieval_scope: 'full_collection',
        status: 'success',
      };

      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: true,
        json: async () => mockResponse,
      });

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      const response = await client.sendQuery(request);
      expect(response.status).toBe('success');
      expect(response.retrieved_chunks).toHaveLength(1);
      expect(response.retrieved_chunks[0].similarity_score).toBe(0.95);
    });
  });

  describe('error handling', () => {
    it('should handle 400 Bad Request', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: false,
        status: 400,
        json: async () => ({ error: { code: 'INVALID_REQUEST', message: 'Bad request' } }),
      });

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      await expect(client.sendQuery(request)).rejects.toMatchObject({
        code: 'VALIDATION_ERROR',
      });
    });

    it('should handle 429 Rate Limit', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: false,
        status: 429,
        json: async () => ({ error: { message: 'Too many requests' } }),
      });

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      await expect(client.sendQuery(request)).rejects.toMatchObject({
        code: 'RATE_LIMIT_ERROR',
      });
    });

    it('should handle 500 Server Error', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: false,
        status: 500,
        json: async () => ({ error: { message: 'Internal server error' } }),
      });

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      await expect(client.sendQuery(request)).rejects.toMatchObject({
        code: 'SERVER_ERROR',
      });
    });

    it('should handle network errors', async () => {
      global.fetch = jest.fn().mockRejectedValueOnce(
        new TypeError('Failed to fetch')
      );

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      await expect(client.sendQuery(request)).rejects.toMatchObject({
        code: 'NETWORK_ERROR',
      });
    });

    it('should handle timeout errors', async () => {
      global.fetch = jest.fn().mockRejectedValueOnce(
        new DOMException('AbortError', 'AbortError')
      );

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      await expect(client.sendQuery(request)).rejects.toMatchObject({
        code: 'TIMEOUT_ERROR',
      });
    });
  });

  describe('request headers and body', () => {
    it('should send correct Content-Type header', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
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

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      await client.sendQuery(request);

      expect(global.fetch).toHaveBeenCalledWith(
        mockApiUrl,
        expect.objectContaining({
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
        })
      );
    });

    it('should serialize request body correctly', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
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

      const request: ChatRequest = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      await client.sendQuery(request);

      const callArgs = (global.fetch as jest.Mock).mock.calls[0];
      const body = JSON.parse(callArgs[1].body);

      expect(body.query).toBe('What is ROS2?');
      expect(body.retrieval_scope).toBe('full_collection');
      expect(body.top_k).toBe(5);
    });
  });
});
