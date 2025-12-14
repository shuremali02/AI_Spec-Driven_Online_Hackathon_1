# API Contract: RAG Chatbot for Physical AI Textbook

## Overview

This document defines the API contracts for the RAG Chatbot feature, specifying the endpoints, request/response formats, authentication, and error handling for the chat functionality.

## Base URL

```
https://api.yourdomain.com/v1
```

## Authentication

All API endpoints require authentication using API keys. Include the API key in the Authorization header:

```
Authorization: Bearer {api_key}
```

For rate limiting, the system implements 100 requests per user per minute (FR-071).

## Endpoints

### 1. Create Conversation

**Endpoint**: `POST /conversations`

**Description**: Creates a new conversation session for a user (FR-025).

**Request**:
```json
{
  "user_id": "string (optional)",
  "session_id": "string (required for anonymous users)",
  "initial_query": "string (optional)"
}
```

**Response** (201 Created):
```json
{
  "conversation_id": "uuid",
  "created_at": "timestamp",
  "title": "string (auto-generated if not provided)",
  "is_active": true
}
```

**Error Responses**:
- 400: Invalid request format
- 401: Unauthorized
- 429: Rate limit exceeded (FR-071)

### 2. Send Message

**Endpoint**: `POST /conversations/{conversation_id}/messages`

**Description**: Sends a message in a conversation and receives a response from the RAG system (FR-001, FR-002, FR-008, FR-009).

**Request**:
```json
{
  "content": "string (user query)",
  "message_type": "enum: query|follow-up|text-selection (default: query)"
}
```

**Response** (200 OK):
```json
{
  "message_id": "uuid",
  "conversation_id": "uuid",
  "sender_type": "system",
  "content": "string (generated response)",
  "citations": [
    {
      "chapter_title": "string",
      "section_title": "string",
      "url_path": "string",
      "confidence_score": "float (0-1)",
      "content_snippet": "string"
    }
  ],
  "created_at": "timestamp",
  "message_type": "response"
}
```

**Streaming Response** (for SSE - FR-008):
```
data: {"content_chunk": "string", "is_complete": false, "citations": []}

data: {"content_chunk": "string", "is_complete": true, "citations": [{"chapter_title": "...", "section_title": "...", "url_path": "...", "confidence_score": 0.95}]}
```

**Error Responses**:
- 400: Invalid request format
- 401: Unauthorized
- 404: Conversation not found
- 422: Query validation failed (FR-031)
- 429: Rate limit exceeded (FR-071)
- 503: Vector database unavailable (FR-074)

### 3. Get Conversation History

**Endpoint**: `GET /conversations/{conversation_id}`

**Description**: Retrieves the history of messages in a conversation (FR-025).

**Response** (200 OK):
```json
{
  "conversation_id": "uuid",
  "title": "string",
  "created_at": "timestamp",
  "updated_at": "timestamp",
  "is_active": "boolean",
  "messages": [
    {
      "message_id": "uuid",
      "sender_type": "enum: user|system",
      "content": "string",
      "created_at": "timestamp",
      "citations": "array (for system messages)",
      "message_type": "enum: query|response|system_message"
    }
  ]
}
```

**Error Responses**:
- 401: Unauthorized
- 404: Conversation not found

### 4. Clear Conversation History

**Endpoint**: `DELETE /conversations/{conversation_id}/messages`

**Description**: Clears all messages in a conversation while keeping the conversation (FR-026).

**Response** (204 No Content)

**Error Responses**:
- 401: Unauthorized
- 404: Conversation not found

### 5. Get Citations for Message

**Endpoint**: `GET /messages/{message_id}/citations`

**Description**: Retrieves citations for a specific response message (FR-009, FR-019).

**Response** (200 OK):
```json
{
  "message_id": "uuid",
  "citations": [
    {
      "citation_id": "uuid",
      "chapter_title": "string",
      "section_title": "string",
      "url_path": "string",
      "confidence_score": "float (0-1)",
      "content_snippet": "string"
    }
  ]
}
```

**Error Responses**:
- 401: Unauthorized
- 404: Message not found

### 6. Submit Feedback

**Endpoint**: `POST /feedback`

**Description**: Allows users to report inaccurate or problematic responses (FR-038).

**Request**:
```json
{
  "message_id": "uuid",
  "conversation_id": "uuid",
  "feedback_type": "enum: positive|negative|report_inaccurate|report_inappropriate",
  "comment": "string (optional)"
}
```

**Response** (200 OK):
```json
{
  "feedback_id": "uuid",
  "status": "submitted"
}
```

**Error Responses**:
- 400: Invalid request format
- 401: Unauthorized

### 7. Search Textbook Content

**Endpoint**: `POST /search`

**Description**: Allows users to search textbook content directly (FR-042).

**Request**:
```json
{
  "query": "string (search query)",
  "max_results": "integer (default: 5, max: 20)",
  "include_citations": "boolean (default: true)"
}
```

**Response** (200 OK):
```json
{
  "query": "string",
  "results": [
    {
      "content_id": "uuid",
      "chapter_title": "string",
      "section_title": "string",
      "url_path": "string",
      "content_snippet": "string",
      "relevance_score": "float (0-1)"
    }
  ],
  "total_results": "integer"
}
```

**Error Responses**:
- 400: Invalid request format
- 401: Unauthorized
- 429: Rate limit exceeded

### 8. Get Analytics (Educator Access)

**Endpoint**: `GET /analytics/common-questions`

**Description**: Provides educators with insights on common student questions (FR-034).

**Query Parameters**:
- `time_range`: enum: daily|weekly|monthly (default: weekly)
- `limit`: integer (default: 20, max: 100)

**Response** (200 OK):
```json
{
  "time_range": "string",
  "common_questions": [
    {
      "question": "string",
      "frequency": "integer",
      "most_common_citations": [
        {
          "chapter_title": "string",
          "section_title": "string",
          "frequency": "integer"
        }
      ],
      "difficulty_score": "float (0-1)"
    }
  ]
}
```

**Error Responses**:
- 401: Unauthorized
- 403: Insufficient permissions

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

## Rate Limiting

All endpoints are subject to rate limiting as per FR-071:
- 100 requests per user per minute
- Header: `X-RateLimit-Remaining: {remaining_requests}`
- Header: `X-RateLimit-Reset: {timestamp_when_reset_occurs}`

## Content Validation

All user inputs are validated to prevent malicious queries (FR-031):
- Input sanitization filters are applied
- Maximum content length: 10,000 characters
- Prohibited content patterns are blocked

## Response Time Requirements

The system must respond to 95% of queries within 5 seconds under normal load conditions (FR-075).

## Fallback Handling

When vector database is unavailable, the system returns graceful fallback messages (FR-074):
- Status code: 503 Service Unavailable
- Error code: "VECTOR_DATABASE_UNAVAILABLE"
- User-friendly message explaining the temporary issue

## Cross-Cutting Constraints

All responses must be limited to information available within the Physical AI & Humanoid Robotics textbook (FR-010), and the RAG system must not generate responses that contradict textbook content (Constitution: No Hallucinations principle).