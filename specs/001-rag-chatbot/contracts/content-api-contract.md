# API Contract: Content Management for RAG Chatbot

## Overview

This document defines the API contracts for managing textbook content used by the RAG Chatbot system, including content ingestion, embedding, and management functionality.

## Base URL

```
https://api.yourdomain.com/v1
```

## Authentication

All API endpoints require authentication using service credentials for administrative functions.

## Endpoints

### 1. Ingest Textbook Content

**Endpoint**: `POST /content/ingest`

**Description**: Ingests textbook content and generates embeddings for RAG retrieval (FR-005, FR-007, FR-028).

**Request**:
```json
{
  "source_type": "enum: docusaurus_docs|markdown|text",
  "content_path": "string (path to content)",
  "chunk_size": "integer (default: 512, per FR-076)",
  "process_images": "boolean (default: false)",
  "process_code_blocks": "boolean (default: true)"
}
```

**Response** (202 Accepted):
```json
{
  "job_id": "uuid",
  "status": "processing",
  "estimated_completion": "timestamp",
  "total_chunks": "integer",
  "processed_chunks": "integer"
}
```

**Error Responses**:
- 400: Invalid request format
- 401: Unauthorized
- 422: Content validation failed

### 2. Get Content Processing Status

**Endpoint**: `GET /content/ingest/{job_id}`

**Description**: Checks the status of a content ingestion job.

**Response** (200 OK):
```json
{
  "job_id": "uuid",
  "status": "enum: processing|completed|failed",
  "progress": "float (0-1)",
  "total_chunks": "integer",
  "processed_chunks": "integer",
  "completed_at": "timestamp (if completed)",
  "error_message": "string (if failed)"
}
```

**Error Responses**:
- 401: Unauthorized
- 404: Job not found

### 3. Get Textbook Content

**Endpoint**: `GET /content/{content_id}`

**Description**: Retrieves specific textbook content by ID.

**Response** (200 OK):
```json
{
  "content_id": "uuid",
  "chapter_id": "string",
  "section_path": "string",
  "content_type": "enum: text|code|diagram_description|mathematical_formula",
  "content_text": "string",
  "metadata": {
    "token_count": "integer",
    "difficulty": "enum: beginner|intermediate|advanced",
    "tags": ["string"],
    "related_concepts": ["string"]
  },
  "created_at": "timestamp",
  "updated_at": "timestamp"
}
```

**Error Responses**:
- 401: Unauthorized
- 404: Content not found

### 4. Search Content by Query

**Endpoint**: `POST /content/search`

**Description**: Performs semantic search on textbook content using vector database (FR-002).

**Request**:
```json
{
  "query": "string (search query)",
  "max_results": "integer (default: 10, max: 50)",
  "min_relevance_score": "float (default: 0.3, range: 0-1)",
  "chapter_filter": "string (optional, filter by specific chapter)",
  "content_type_filter": "enum: text|code|diagram_description|mathematical_formula (optional)"
}
```

**Response** (200 OK):
```json
{
  "query": "string",
  "results": [
    {
      "content_id": "uuid",
      "chapter_id": "string",
      "section_path": "string",
      "content_type": "string",
      "content_snippet": "string",
      "relevance_score": "float (0-1)",
      "token_count": "integer",
      "metadata": "object"
    }
  ],
  "total_results": "integer",
  "search_time_ms": "float"
}
```

**Error Responses**:
- 400: Invalid request format
- 401: Unauthorized
- 503: Vector database unavailable (FR-074)

### 5. Get Content Embeddings

**Endpoint**: `GET /content/{content_id}/embeddings`

**Description**: Retrieves the vector embeddings for specific content.

**Response** (200 OK):
```json
{
  "content_id": "uuid",
  "embedding_id": "string",
  "vector_size": "integer",
  "vector_data": "array of floats",
  "model_version": "string",
  "created_at": "timestamp"
}
```

**Error Responses**:
- 401: Unauthorized
- 404: Content or embeddings not found

### 6. Update Content Metadata

**Endpoint**: `PATCH /content/{content_id}/metadata`

**Description**: Updates metadata for existing content without regenerating embeddings.

**Request**:
```json
{
  "metadata": {
    "difficulty": "enum: beginner|intermediate|advanced",
    "tags": ["string"],
    "related_concepts": ["string"],
    "deprecated": "boolean"
  }
}
```

**Response** (200 OK):
```json
{
  "content_id": "uuid",
  "updated_fields": ["string"],
  "updated_at": "timestamp"
}
```

**Error Responses**:
- 400: Invalid request format
- 401: Unauthorized
- 404: Content not found

### 7. Delete Content

**Endpoint**: `DELETE /content/{content_id}`

**Description**: Removes content and its embeddings from the system.

**Response** (204 No Content)

**Error Responses**:
- 401: Unauthorized
- 404: Content not found

### 8. Get Content Statistics

**Endpoint**: `GET /content/stats`

**Description**: Retrieves statistics about the content corpus.

**Response** (200 OK):
```json
{
  "total_content_items": "integer",
  "total_chunks": "integer",
  "total_tokens": "integer",
  "chapters_count": "integer",
  "last_updated": "timestamp",
  "storage_usage_mb": "float",
  "embedding_model": "string",
  "content_types": {
    "text": "integer",
    "code": "integer",
    "diagram_description": "integer",
    "mathematical_formula": "integer"
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

## Content Processing Requirements

- Content must be chunked into 512-token segments for embedding generation (FR-076)
- System must handle large volumes of textbook content efficiently (FR-028)
- Semantic meaning of textbook content must be preserved during retrieval (FR-029)
- System must handle different types of content including text, diagrams, and code examples (FR-039)

## Fallback Handling

When vector database is unavailable, the system returns graceful fallback messages (FR-074).