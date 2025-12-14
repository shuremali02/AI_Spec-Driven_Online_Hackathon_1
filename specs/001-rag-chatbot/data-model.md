# Data Model: RAG Chatbot for Physical AI Textbook

## Overview

This document defines the data models for the RAG Chatbot feature, including database schemas, vector database structures, and data relationships that support the Retrieval-Augmented Generation functionality.

## Key Entities

### 1. Conversation

Represents a single user session with the chatbot, containing multiple exchanges between user and system.

**Fields:**
- `id` (UUID): Unique identifier for the conversation
- `user_id` (UUID): Reference to the user who initiated the conversation
- `created_at` (timestamp): When the conversation started
- `updated_at` (timestamp): When the conversation was last updated
- `title` (string, optional): Auto-generated title based on first query
- `is_active` (boolean): Whether the conversation is currently active

**Relationships:**
- One-to-many with Message entities
- Belongs to a User entity

**Validation Rules:**
- `user_id` must reference a valid user
- `created_at` must be before `updated_at`
- `title` must be 1-100 characters if provided

### 2. Message

An individual communication within a conversation, either from the user (query) or system (response).

**Fields:**
- `id` (UUID): Unique identifier for the message
- `conversation_id` (UUID): Reference to the parent conversation
- `sender_type` (enum): 'user' or 'system'
- `content` (text): The actual message content
- `created_at` (timestamp): When the message was created
- `citations` (JSON): Array of textbook section references
- `message_type` (enum): 'query', 'response', 'system_message'

**Relationships:**
- Belongs to a Conversation entity
- Belongs to a User entity (for user messages)

**Validation Rules:**
- `conversation_id` must reference a valid conversation
- `sender_type` must be either 'user' or 'system'
- `content` must not exceed 10,000 characters
- `citations` must be a valid JSON array of textbook references

### 3. Textbook Content

The source material from the Physical AI & Humanoid Robotics textbook, including chapters, sections, and subsections. This is the content used for RAG retrieval.

**Fields:**
- `id` (UUID): Unique identifier for the content segment
- `chapter_id` (string): Identifier for the textbook chapter
- `section_path` (string): Path to the specific section (e.g., "chapter-1/section-2")
- `content_text` (text): The actual text content
- `content_type` (enum): 'text', 'code', 'diagram_description', 'mathematical_formula'
- `metadata` (JSON): Additional information about the content
- `embedding_id` (string): Reference to the vector embedding in Qdrant
- `token_count` (integer): Number of tokens in the content

**Relationships:**
- One-to-one with Embedding entities (via `embedding_id`)
- Used for retrieval in the RAG process

**Validation Rules:**
- `content_text` must not be empty
- `token_count` must be a positive integer
- `content_type` must be one of the defined enum values
- `embedding_id` must match the corresponding vector in Qdrant

### 4. Embedding

Vector representation of textbook content used for semantic search and retrieval in the Qdrant vector database.

**Fields:**
- `id` (string): Unique identifier for the embedding
- `textbook_content_id` (UUID): Reference to the original content
- `vector` (array of floats): The embedding vector (dimension depends on model)
- `created_at` (timestamp): When the embedding was generated
- `model_version` (string): Version of the embedding model used
- `chunk_index` (integer): Position of this chunk in the original content

**Relationships:**
- Belongs to a Textbook Content entity
- Stored in Qdrant Cloud vector database

**Validation Rules:**
- `vector` must have the correct dimension for the embedding model
- `textbook_content_id` must reference a valid textbook content record
- `chunk_index` must be non-negative

### 5. User

A student or educator interacting with the chatbot system.

**Fields:**
- `id` (UUID): Unique identifier for the user
- `user_type` (enum): 'student', 'educator', 'admin'
- `created_at` (timestamp): When the user account was created
- `last_active_at` (timestamp): When the user was last active
- `rate_limit_reset_at` (timestamp): When the rate limit counter resets

**Relationships:**
- One-to-many with Conversation entities
- One-to-many with Message entities (for user messages)

**Validation Rules:**
- `user_type` must be one of the defined enum values
- `rate_limit_reset_at` must be in the future

### 6. Citation

Reference to specific locations within the textbook that support the chatbot's responses.

**Fields:**
- `id` (UUID): Unique identifier for the citation
- `message_id` (UUID): Reference to the message containing this citation
- `textbook_content_id` (UUID): Reference to the cited content
- `chapter_title` (string): Title of the referenced chapter
- `section_title` (string): Title of the referenced section
- `url_path` (string): Relative path to the content in the textbook
- `confidence_score` (float): Confidence in the citation relevance (0-1)

**Relationships:**
- Belongs to a Message entity
- Belongs to a Textbook Content entity

**Validation Rules:**
- `confidence_score` must be between 0 and 1
- `url_path` must be a valid relative path
- `message_id` and `textbook_content_id` must reference valid entities

### 7. Analytics Record

Data about user interactions, common questions, and system usage patterns for educators and administrators.

**Fields:**
- `id` (UUID): Unique identifier for the analytics record
- `record_type` (enum): 'user_interaction', 'common_question', 'system_usage', 'performance_metric'
- `user_id` (UUID, optional): Reference to the user involved
- `conversation_id` (UUID, optional): Reference to the conversation
- `data_payload` (JSON): The actual analytics data
- `created_at` (timestamp): When the record was created
- `aggregation_period` (string): 'hourly', 'daily', 'weekly', 'monthly'

**Relationships:**
- Optionally belongs to a User entity
- Optionally belongs to a Conversation entity

**Validation Rules:**
- `record_type` must be one of the defined enum values
- At least one of `user_id` or `conversation_id` must be provided for interaction records
- `data_payload` must be valid JSON

## Database Schema (PostgreSQL)

```sql
-- Users table
CREATE TABLE users (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_type VARCHAR(20) NOT NULL CHECK (user_type IN ('student', 'educator', 'admin')),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    last_active_at TIMESTAMP WITH TIME ZONE,
    rate_limit_reset_at TIMESTAMP WITH TIME ZONE
);

-- Conversations table
CREATE TABLE conversations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    user_id UUID NOT NULL REFERENCES users(id) ON DELETE CASCADE,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    updated_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    title VARCHAR(100),
    is_active BOOLEAN DEFAULT true
);

-- Messages table
CREATE TABLE messages (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    conversation_id UUID NOT NULL REFERENCES conversations(id) ON DELETE CASCADE,
    sender_type VARCHAR(10) NOT NULL CHECK (sender_type IN ('user', 'system')),
    content TEXT NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    citations JSONB,
    message_type VARCHAR(20) NOT NULL DEFAULT 'query' CHECK (message_type IN ('query', 'response', 'system_message'))
);

-- Citations table
CREATE TABLE citations (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    message_id UUID NOT NULL REFERENCES messages(id) ON DELETE CASCADE,
    textbook_content_id UUID NOT NULL,
    chapter_title VARCHAR(200) NOT NULL,
    section_title VARCHAR(200) NOT NULL,
    url_path VARCHAR(500) NOT NULL,
    confidence_score DECIMAL(3, 2) CHECK (confidence_score >= 0 AND confidence_score <= 1)
);

-- Textbook content table (for metadata, actual content in vector DB)
CREATE TABLE textbook_content (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    chapter_id VARCHAR(100) NOT NULL,
    section_path VARCHAR(200) NOT NULL,
    content_type VARCHAR(50) NOT NULL CHECK (content_type IN ('text', 'code', 'diagram_description', 'mathematical_formula')),
    metadata JSONB,
    embedding_id VARCHAR(100) NOT NULL,
    token_count INTEGER CHECK (token_count > 0),
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP
);

-- Analytics records table
CREATE TABLE analytics_records (
    id UUID PRIMARY KEY DEFAULT gen_random_uuid(),
    record_type VARCHAR(30) NOT NULL CHECK (record_type IN ('user_interaction', 'common_question', 'system_usage', 'performance_metric')),
    user_id UUID REFERENCES users(id),
    conversation_id UUID REFERENCES conversations(id),
    data_payload JSONB NOT NULL,
    created_at TIMESTAMP WITH TIME ZONE DEFAULT CURRENT_TIMESTAMP,
    aggregation_period VARCHAR(10) CHECK (aggregation_period IN ('hourly', 'daily', 'weekly', 'monthly'))
);

-- Indexes for performance
CREATE INDEX idx_messages_conversation_id ON messages(conversation_id);
CREATE INDEX idx_messages_created_at ON messages(created_at);
CREATE INDEX idx_citations_message_id ON citations(message_id);
CREATE INDEX idx_conversations_user_id ON conversations(user_id);
CREATE INDEX idx_analytics_records_type_created ON analytics_records(record_type, created_at);
```

## Vector Database Schema (Qdrant)

The vector database will store embeddings for semantic search and retrieval:

```json
{
  "collection_name": "textbook_content_embeddings",
  "vector_size": 768,
  "distance": "Cosine",
  "payload_schema": {
    "textbook_content_id": {
      "type": "keyword"
    },
    "chapter_id": {
      "type": "keyword"
    },
    "section_path": {
      "type": "keyword"
    },
    "token_count": {
      "type": "integer"
    },
    "content_type": {
      "type": "keyword"
    },
    "chunk_index": {
      "type": "integer"
    }
  }
}
```

## Data Flow

1. **Content Ingestion**: Textbook content is processed, chunked into 512-token segments, and stored in both PostgreSQL (metadata) and Qdrant (embeddings)
2. **Query Processing**: User queries are embedded and compared against textbook content embeddings in Qdrant
3. **Retrieval**: Relevant content segments are retrieved based on similarity scores
4. **Response Generation**: The RAG agent uses retrieved content to generate responses with proper citations
5. **Storage**: Conversations and messages are stored in PostgreSQL with citation references
6. **Analytics**: Usage patterns are tracked and stored as analytics records for educators

## Constraints and Validation

1. **Referential Integrity**: Foreign key constraints ensure data consistency between related entities
2. **Data Validation**: Check constraints and data type validation prevent invalid data
3. **Content Limitations**: Text content is limited to prevent oversized entries
4. **Rate Limiting**: User rate limit tracking prevents abuse
5. **Citation Accuracy**: Citations must reference valid textbook content
6. **Embedding Consistency**: Vector embeddings must match the expected dimension and model version