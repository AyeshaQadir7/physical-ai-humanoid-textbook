# ============================================================================
# Multi-stage build for optimized Docker image
# ============================================================================

# Stage 1: Builder - Install dependencies
FROM python:3.11-slim as builder

WORKDIR /build

# Copy requirements from backend (source of truth)
COPY backend/requirements.txt .

# Install Python packages to /root/.local (for efficient layer copying)
RUN pip install --no-cache-dir --user -r requirements.txt

# ============================================================================
# Stage 2: Runtime - Minimal production image
# ============================================================================

FROM python:3.11-slim

WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Copy only installed packages from builder stage
COPY --from=builder /root/.local /root/.local

# Copy entire backend folder (single source of truth)
COPY backend/ ./

# Set environment variables
ENV PATH=/root/.local/bin:$PATH
ENV PYTHONUNBUFFERED=1
ENV PYTHONDONTWRITEBYTECODE=1
ENV PORT=7860

# Expose Hugging Face Spaces default port
EXPOSE 7860

# Health check - verify API is responding
HEALTHCHECK --interval=30s --timeout=10s --start-period=10s --retries=3 \
    CMD curl -f http://localhost:7860/health || exit 1

# Run FastAPI application
# agent.py is now at root level (copied from backend/)
CMD ["python", "-m", "uvicorn", "agent:app", "--host", "0.0.0.0", "--port", "7860", "--log-level", "info"]
