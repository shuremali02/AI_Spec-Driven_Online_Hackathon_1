# API Contract: System Operations for RAG Chatbot

## Overview

This document defines the API contracts for system-level operations, monitoring, and administrative functions for the RAG Chatbot system.

## Base URL

```
https://api.yourdomain.com/v1
```

## Authentication

System endpoints require administrative credentials for access.

## Endpoints

### 1. Health Check

**Endpoint**: `GET /health`

**Description**: Checks the health status of the system and its dependencies.

**Response** (200 OK):
```json
{
  "status": "healthy",
  "timestamp": "timestamp",
  "services": {
    "api_server": "healthy",
    "postgres_db": "healthy",
    "qdrant_db": "healthy",
    "llm_provider": "healthy",
    "rate_limiter": "healthy"
  },
  "version": "string",
  "uptime_seconds": "integer"
}
```

**Error Responses**:
- 503: Service unavailable

### 2. System Metrics

**Endpoint**: `GET /metrics`

**Description**: Retrieves system performance metrics.

**Response** (200 OK):
```json
{
  "timestamp": "timestamp",
  "metrics": {
    "active_conversations": "integer",
    "requests_per_minute": "float",
    "avg_response_time_ms": "float",
    "error_rate": "float",
    "token_usage_current_minute": "integer",
    "embedding_usage_current_minute": "integer",
    "vector_db_queries_per_minute": "float",
    "cache_hit_rate": "float"
  },
  "performance": {
    "p95_response_time": "float (ms)",
    "p99_response_time": "float (ms)",
    "concurrent_users": "integer"
  }
}
```

**Error Responses**:
- 401: Unauthorized

### 3. Rate Limit Configuration

**Endpoint**: `GET /rate-limit/config`

**Description**: Retrieves current rate limiting configuration (FR-071).

**Response** (200 OK):
```json
{
  "default_limit": "integer (requests per minute)",
  "default_window": "integer (seconds)",
  "burst_limit": "integer (requests)",
  "user_specific_limits": {
    "student": "integer",
    "educator": "integer",
    "admin": "integer"
  },
  "last_updated": "timestamp"
}
```

**Error Responses**:
- 401: Unauthorized

### 4. Update Rate Limit Configuration

**Endpoint**: `PUT /rate-limit/config`

**Description**: Updates rate limiting configuration (FR-071).

**Request**:
```json
{
  "default_limit": "integer (requests per minute)",
  "user_specific_limits": {
    "student": "integer",
    "educator": "integer",
    "admin": "integer"
  }
}
```

**Response** (200 OK):
```json
{
  "config_id": "uuid",
  "updated_at": "timestamp",
  "applied": true
}
```

**Error Responses**:
- 400: Invalid request format
- 401: Unauthorized

### 5. Get Security Events

**Endpoint**: `GET /security/events`

**Description**: Retrieves security-relevant events for audit purposes (FR-073).

**Query Parameters**:
- `start_time`: "timestamp (default: last 24 hours)"
- `end_time`: "timestamp (default: now)"
- `event_type`: "enum: rate_limit_exceeded|invalid_input|auth_failure|suspicious_activity"
- `limit`: "integer (default: 100, max: 1000)"

**Response** (200 OK):
```json
{
  "events": [
    {
      "event_id": "uuid",
      "event_type": "string",
      "timestamp": "timestamp",
      "user_id": "string (if applicable)",
      "ip_address": "string",
      "user_agent": "string",
      "details": "object",
      "severity": "enum: low|medium|high|critical"
    }
  ],
  "total_count": "integer",
  "next_cursor": "string (if more results available)"
}
```

**Error Responses**:
- 401: Unauthorized

### 6. Trigger Content Reindex

**Endpoint**: `POST /system/reindex`

**Description**: Triggers a full reindex of textbook content.

**Request**:
```json
{
  "force_reindex": "boolean (default: false)",
  "content_filter": {
    "chapter_ids": ["string"],
    "date_modified_since": "timestamp"
  }
}
```

**Response** (202 Accepted):
```json
{
  "job_id": "uuid",
  "status": "processing",
  "estimated_duration_minutes": "integer",
  "total_items_to_reindex": "integer"
}
```

**Error Responses**:
- 401: Unauthorized

### 7. Get System Configuration

**Endpoint**: `GET /system/config`

**Description**: Retrieves current system configuration.

**Response** (200 OK):
```json
{
  "config": {
    "llm_provider": "string (e.g., gemini)",
    "embedding_model": "string",
    "vector_db": "string (e.g., qdrant)",
    "database": "string (e.g., neon postgres)",
    "max_concurrent_users": "integer",
    "response_timeout_seconds": "integer",
    "max_tokens_per_response": "integer",
    "min_confidence_score": "float",
    "content_chunk_size": "integer (should be 512 per FR-076)"
  },
  "last_updated": "timestamp"
}
```

**Error Responses**:
- 401: Unauthorized

### 8. Update System Configuration

**Endpoint**: `PUT /system/config`

**Description**: Updates system configuration parameters.

**Request**:
```json
{
  "config": {
    "max_concurrent_users": "integer",
    "response_timeout_seconds": "integer",
    "max_tokens_per_response": "integer",
    "min_confidence_score": "float"
  }
}
```

**Response** (200 OK):
```json
{
  "config_id": "uuid",
  "updated_fields": ["string"],
  "updated_at": "timestamp",
  "restart_required": "boolean"
}
```

**Error Responses**:
- 400: Invalid request format
- 401: Unauthorized

### 9. Get Performance Analytics

**Endpoint**: `GET /analytics/performance`

**Description**: Retrieves performance analytics for the system (FR-075).

**Query Parameters**:
- `start_time`: "timestamp"
- `end_time`: "timestamp"
- `granularity`: "enum: minute|hour|day|week"

**Response** (200 OK):
```json
{
  "period": {
    "start_time": "timestamp",
    "end_time": "timestamp"
  },
  "performance_metrics": {
    "avg_response_time_ms": "float",
    "p95_response_time_ms": "float",
    "p99_response_time_ms": "float",
    "query_success_rate": "float",
    "within_5s_threshold_rate": "float (should be >= 0.95 per FR-075)",
    "requests_count": "integer"
  },
  "availability": {
    "uptime_percentage": "float (should be >= 99.9 per FR-077)",
    "downtime_minutes": "integer",
    "incident_count": "integer"
  }
}
```

**Error Responses**:
- 401: Unauthorized

### 10. Get User Analytics

**Endpoint**: `GET /analytics/users`

**Description**: Retrieves user engagement analytics.

**Query Parameters**:
- `start_time`: "timestamp"
- `end_time`: "timestamp"
- `user_type_filter`: "enum: student|educator|all"
- `granularity`: "enum: hour|day|week"

**Response** (200 OK):
```json
{
  "period": {
    "start_time": "timestamp",
    "end_time": "timestamp"
  },
  "user_metrics": {
    "active_users": "integer",
    "new_users": "integer",
    "total_conversations": "integer",
    "avg_conversations_per_user": "float",
    "avg_messages_per_conversation": "float",
    "user_satisfaction_score": "float (if feedback available)",
    "engagement_metrics": {
      "avg_session_duration_minutes": "float",
      "return_user_rate": "float",
      "feature_usage": {
        "chat_queries": "integer",
        "content_searches": "integer",
        "citations_viewed": "integer",
        "feedback_submitted": "integer"
      }
    }
  }
}
```

**Error Responses**:
- 401: Unauthorized

## Error Format

All error responses follow this format:

```json
{
  "error": {
    "code": "string (error code)",
    "message": "string (human-readable message)",
    "details": "object (optional, specific error details)",
    "timestamp": "timestamp"
  }
}
```

## Performance Requirements

- System must respond to 95% of queries within 5 seconds under normal load conditions (FR-075)
- System must maintain 99.9% availability measured monthly (FR-077)

## Security & Rate Limiting

- System must implement rate limiting of 100 requests per user per minute to prevent abuse (FR-071)
- System must validate all user inputs using sanitization filters to prevent injection attacks (FR-072)
- System must log all security-relevant events for audit purposes (FR-073)

## Failure Handling

- System must return a graceful fallback message when vector database is unavailable (FR-074)