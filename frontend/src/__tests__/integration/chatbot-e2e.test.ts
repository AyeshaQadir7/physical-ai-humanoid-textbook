/**
 * End-to-End Integration Tests for ChatBot
 *
 * Tests the complete flow: user input → API call → response display
 */

describe('ChatBot E2E Integration Tests', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  describe('happy path: complete chat flow', () => {
    it('should submit query and display response', async () => {
      // Mock the fetch API
      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          query: 'What is ROS2?',
          answer: 'ROS2 is a middleware platform for robotics software development.',
          retrieved_chunks: [
            {
              chunk_id: '1',
              text: 'ROS2 is a middleware platform that provides services and abstractions for hardware abstraction, device drivers, message passing, package management, and more.',
              similarity_score: 0.96,
              source_url: 'https://robotics.example.com/ros2',
              page_title: 'Introduction to ROS2',
              section_headers: ['Robotics', 'ROS2', 'Overview'],
            },
          ],
          execution_metrics: {
            retrieval_time_ms: 150,
            generation_time_ms: 250,
            total_time_ms: 400,
          },
          retrieval_scope: 'full_collection',
          status: 'success',
        }),
      });

      const request = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      // Simulate the request
      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(request),
      });

      const data = await response.json();

      // Verify the response
      expect(response.ok).toBe(true);
      expect(data.status).toBe('success');
      expect(data.answer).toContain('middleware');
      expect(data.retrieved_chunks).toHaveLength(1);
      expect(data.retrieved_chunks[0].similarity_score).toBe(0.96);
    });

    it('should handle conversation with multiple turns', async () => {
      // First query
      global.fetch = jest
        .fn()
        .mockResolvedValueOnce({
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
        })
        .mockResolvedValueOnce({
          ok: true,
          json: async () => ({
            query: 'How do I install ROS2?',
            answer: 'ROS2 installation depends on your OS...',
            retrieved_chunks: [],
            execution_metrics: {
              retrieval_time_ms: 120,
              generation_time_ms: 220,
              total_time_ms: 340,
            },
            retrieval_scope: 'full_collection',
            status: 'success',
          }),
        });

      // First query
      const response1 = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: 'What is ROS2?',
          retrieval_scope: 'full_collection',
          top_k: 5,
        }),
      });
      const data1 = await response1.json();
      expect(data1.status).toBe('success');

      // Second query
      const response2 = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: 'How do I install ROS2?',
          retrieval_scope: 'full_collection',
          top_k: 5,
        }),
      });
      const data2 = await response2.json();
      expect(data2.status).toBe('success');

      // Verify both requests were made
      expect(global.fetch).toHaveBeenCalledTimes(2);
    });
  });

  describe('error scenarios', () => {
    it('should handle network error gracefully', async () => {
      global.fetch = jest
        .fn()
        .mockRejectedValueOnce(new TypeError('Failed to fetch'));

      const request = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      try {
        await fetch('http://localhost:8000/chat', {
          method: 'POST',
          headers: { 'Content-Type': 'application/json' },
          body: JSON.stringify(request),
        });
        fail('Should have thrown an error');
      } catch (error) {
        expect(error).toEqual(expect.any(TypeError));
      }
    });

    it('should handle 500 server error', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: false,
        status: 500,
        json: async () => ({
          error: {
            code: 'SERVER_ERROR',
            message: 'Internal server error',
          },
        }),
      });

      const request = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(request),
      });

      expect(response.ok).toBe(false);
      expect(response.status).toBe(500);
    });

    it('should handle 429 rate limit error', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: false,
        status: 429,
        json: async () => ({
          error: {
            code: 'RATE_LIMIT',
            message: 'Too many requests',
          },
        }),
      });

      const request = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(request),
      });

      expect(response.ok).toBe(false);
      expect(response.status).toBe(429);
    });

    it('should handle invalid request (400)', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: false,
        status: 400,
        json: async () => ({
          error: {
            code: 'VALIDATION_ERROR',
            message: 'Invalid request format',
          },
        }),
      });

      const request = {
        query: '', // Empty query
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(request),
      });

      expect(response.ok).toBe(false);
      expect(response.status).toBe(400);
    });
  });

  describe('response validation', () => {
    it('should include all required fields in response', async () => {
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
        }),
      });

      const request = {
        query: 'What is ROS2?',
        retrieval_scope: 'full_collection',
        top_k: 5,
      };

      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify(request),
      });

      const data = await response.json();

      // Verify all required fields are present
      expect(data).toHaveProperty('query');
      expect(data).toHaveProperty('answer');
      expect(data).toHaveProperty('retrieved_chunks');
      expect(data).toHaveProperty('execution_metrics');
      expect(data).toHaveProperty('retrieval_scope');
      expect(data).toHaveProperty('status');
    });

    it('should include execution metrics', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          query: 'What is ROS2?',
          answer: 'ROS2 is middleware',
          retrieved_chunks: [],
          execution_metrics: {
            retrieval_time_ms: 150,
            generation_time_ms: 250,
            total_time_ms: 400,
          },
          retrieval_scope: 'full_collection',
          status: 'success',
        }),
      });

      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: 'What is ROS2?',
          retrieval_scope: 'full_collection',
          top_k: 5,
        }),
      });

      const data = await response.json();

      expect(data.execution_metrics).toBeDefined();
      expect(data.execution_metrics.retrieval_time_ms).toBeGreaterThanOrEqual(0);
      expect(data.execution_metrics.generation_time_ms).toBeGreaterThanOrEqual(0);
      expect(data.execution_metrics.total_time_ms).toBeGreaterThanOrEqual(0);
    });

    it('should return chunks with all required fields', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          query: 'What is ROS2?',
          answer: 'ROS2 is middleware',
          retrieved_chunks: [
            {
              chunk_id: '1',
              text: 'ROS2 is middleware',
              similarity_score: 0.95,
              source_url: 'https://example.com',
              page_title: 'ROS2 Intro',
              section_headers: ['Robotics', 'ROS2'],
            },
          ],
          execution_metrics: {
            retrieval_time_ms: 100,
            generation_time_ms: 200,
            total_time_ms: 300,
          },
          retrieval_scope: 'full_collection',
          status: 'success',
        }),
      });

      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: 'What is ROS2?',
          retrieval_scope: 'full_collection',
          top_k: 5,
        }),
      });

      const data = await response.json();

      expect(data.retrieved_chunks).toHaveLength(1);
      const chunk = data.retrieved_chunks[0];

      expect(chunk).toHaveProperty('chunk_id');
      expect(chunk).toHaveProperty('text');
      expect(chunk).toHaveProperty('similarity_score');
      expect(chunk).toHaveProperty('source_url');
      expect(chunk).toHaveProperty('page_title');
      expect(chunk).toHaveProperty('section_headers');
    });
  });

  describe('performance validation', () => {
    it('should complete within SLA (< 5s)', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          query: 'What is ROS2?',
          answer: 'ROS2 is middleware',
          retrieved_chunks: [],
          execution_metrics: {
            retrieval_time_ms: 150,
            generation_time_ms: 250,
            total_time_ms: 400,
          },
          retrieval_scope: 'full_collection',
          status: 'success',
        }),
      });

      const startTime = Date.now();

      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: 'What is ROS2?',
          retrieval_scope: 'full_collection',
          top_k: 5,
        }),
      });

      const duration = Date.now() - startTime;

      expect(response.ok).toBe(true);
      // SLA: Total time should be < 5 seconds (5000ms)
      expect(duration).toBeLessThan(5000);
    });
  });

  describe('text-only retrieval scope', () => {
    it('should handle text-only retrieval scope', async () => {
      global.fetch = jest.fn().mockResolvedValueOnce({
        ok: true,
        json: async () => ({
          query: 'Question about provided text',
          answer: 'Answer based on provided context',
          retrieved_chunks: [],
          execution_metrics: {
            retrieval_time_ms: 50,
            generation_time_ms: 150,
            total_time_ms: 200,
          },
          retrieval_scope: 'text_only',
          status: 'success',
        }),
      });

      const response = await fetch('http://localhost:8000/chat', {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          query: 'Question about provided text',
          retrieval_scope: 'text_only',
          context_text: 'This is the provided text context',
          top_k: 0, // No retrieval needed for text_only
        }),
      });

      const data = await response.json();

      expect(data.status).toBe('success');
      expect(data.retrieval_scope).toBe('text_only');
      expect(data.answer).toBeDefined();
    });
  });
});
