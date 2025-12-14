# Quickstart Guide: RAG Chatbot for Physical AI Textbook

## Overview

This guide provides a step-by-step introduction to setting up and using the RAG Chatbot for the Physical AI & Humanoid Robotics textbook. The chatbot uses Retrieval-Augmented Generation (RAG) methodology to answer student questions based strictly on textbook content.

## Prerequisites

Before getting started, ensure you have:

### System Requirements
- **Operating System**: Linux (Ubuntu 22.04+), macOS 10.15+, or Windows 10/11 with WSL2
- **Python**: Version 3.10 or higher
- **Node.js**: Version 18 or higher (for Docusaurus frontend)
- **Memory**: 8GB RAM minimum (16GB recommended)
- **Storage**: 2GB available space

### External Services
- **Qdrant Cloud Account**: Free tier account for vector database
- **Neon Serverless Postgres Account**: Free tier account for conversation storage
- **Gemini API Access**: API key for LLM and embeddings (via OpenAI-compatible endpoint)

### Required Tools
- `uv` Python package manager (will be installed automatically)
- Git for version control
- Docker (optional, for containerized deployment)

## Installation

### 1. Clone the Repository

```bash
git clone https://github.com/your-organization/physical-ai-textbook.git
cd physical-ai-textbook
```

### 2. Set Up Backend Environment

```bash
# Navigate to backend directory
cd backend/

# Install uv if not already installed
pip install uv

# Create virtual environment and install dependencies
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate
uv pip install -r requirements.txt
```

### 3. Configure Environment Variables

Create a `.env` file in the `backend/` directory:

```env
# Qdrant Configuration
QDRANT_URL=https://your-cluster-url.qdrant.tech
QDRANT_API_KEY=your-qdrant-api-key
QDRANT_COLLECTION_NAME=textbook_content_embeddings

# Neon Postgres Configuration
DATABASE_URL=postgresql://username:password@ep-xxx.us-east-1.aws.neon.tech/your-db-name?sslmode=require

# Gemini API Configuration (via OpenAI-compatible endpoint)
OPENAI_API_KEY=your-gemini-api-key
OPENAI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai/

# Application Settings
APP_ENV=development
LOG_LEVEL=info
RATE_LIMIT_REQUESTS=100
RATE_LIMIT_WINDOW=60
```

### 4. Set Up Frontend (Docusaurus)

```bash
# From the project root
cd book-write/

# Install dependencies
npm install

# Start the Docusaurus development server
npm start
```

## Basic Usage

### 1. Initialize the System

Before using the chatbot, you need to initialize the vector database with textbook content:

```bash
# From the backend directory
cd backend/
source .venv/bin/activate

# Run the content ingestion script
python -m scripts.ingest_content --source-path ../book-write/docs --chunk-size 512
```

This process will:
- Parse all textbook content in the `book-write/docs` directory
- Chunk content into 512-token segments (per FR-076)
- Generate embeddings using the Gemini embeddings model
- Store embeddings in Qdrant Cloud
- Create metadata records in Neon Postgres

### 2. Start the Backend API Server

```bash
# From the backend directory
cd backend/
source .venv/bin/activate

# Start the FastAPI server
uvicorn main:app --reload --host 0.0.0.0 --port 8000
```

The API will be available at `http://localhost:8000`.

### 3. Using the Chatbot

#### Via Docusaurus Frontend (Recommended)
1. Ensure both the backend (port 8000) and Docusaurus (port 3000) are running
2. Navigate to `http://localhost:3000` in your browser
3. The chatbot will appear as a floating button on all textbook pages
4. Click the button to open the chat interface
5. Ask questions about the textbook content

#### Via API Directly
Create a conversation:
```bash
curl -X POST http://localhost:8000/v1/conversations \
  -H "Authorization: Bearer your-api-key" \
  -H "Content-Type: application/json" \
  -d '{
    "user_id": "optional-user-id",
    "session_id": "unique-session-id"
  }'
```

Send a message:
```bash
curl -X POST http://localhost:8000/v1/conversations/{conversation_id}/messages \
  -H "Authorization: Bearer your-api-key" \
  -H "Content-Type: application/json" \
  -d '{
    "content": "What is ROS 2 and how does it differ from ROS 1?",
    "message_type": "query"
  }'
```

## Configuration Options

### Backend Configuration

The system behavior can be customized through environment variables:

| Variable | Default | Description |
|----------|---------|-------------|
| `EMBEDDING_DIMENSION` | 768 | Dimension of the embedding vectors |
| `MAX_TOKENS_PER_CHUNK` | 512 | Maximum tokens per content chunk (per FR-076) |
| `RAG_TOP_K` | 5 | Number of content chunks to retrieve for each query |
| `RAG_MIN_SCORE` | 0.3 | Minimum relevance score for retrieved content |
| `SSE_STREAMING_ENABLED` | true | Enable Server-Sent Events for streaming responses (per FR-008) |

### Frontend Configuration

Update the chatbot component in `book-write/src/components/Chatbot/config.ts`:

```typescript
export const CHATBOT_CONFIG = {
  apiBaseUrl: 'http://localhost:8000/v1',
  maxMessageLength: 10000,
  enableTypingIndicator: true,
  enableCitations: true,
  enableRateLimiting: true,
  floatingButtonPosition: 'bottom-right'
};
```

## First Steps for Students

1. **Ask Simple Questions**: Start with basic questions about textbook concepts
   - "What is embodied intelligence?"
   - "Explain ROS 2 nodes and topics"

2. **Explore Citations**: Each response includes citations to textbook sections
   - Click on citations to navigate directly to relevant content
   - Use citations to dive deeper into topics

3. **Use Follow-up Questions**: Build on previous responses
   - "Can you elaborate on that?"
   - "How does this apply to humanoid robots?"

4. **Search Content**: Use the search functionality for specific topics
   - "Show me all content about URDF"
   - "Find examples of Python code"

## First Steps for Educators

1. **Monitor Analytics**: Access educator dashboard to review common questions
   - Identify challenging concepts for students
   - Track engagement with different textbook sections

2. **Review Responses**: Evaluate the accuracy of chatbot responses
   - Use the feedback mechanism to report inaccuracies
   - Ensure responses align with textbook content only (no hallucinations)

3. **Guide Student Usage**: Integrate chatbot usage into learning activities
   - Suggest the chatbot for pre-lecture preparation
   - Use for post-lecture clarification

## Troubleshooting

### Common Issues

#### Issue: "Vector database unavailable"
**Solution**: Check your Qdrant Cloud connection:
```bash
# Verify Qdrant connection
python -c "from vector.qdrant_client import QdrantClient; client = QdrantClient(); print(client.health())"
```

#### Issue: "Rate limit exceeded"
**Solution**: The system implements 100 requests per user per minute (per FR-071). Wait for the rate limit to reset or contact your administrator.

#### Issue: "No relevant content found"
**Solution**: This may occur if:
- The question is outside textbook scope (responses limited to textbook content per FR-010)
- Content ingestion is incomplete
- The relevance threshold is too high

### Performance Considerations

- **Response Time**: The system should respond to 95% of queries within 5 seconds (per FR-075)
- **Availability**: Target 99.9% monthly availability (per FR-077)
- **Concurrent Users**: Supports 100+ concurrent users (per success criteria)

## Next Steps

1. **Customize the Chatbot**: Modify the system prompt in `backend/agents/system_prompt.py` to match your specific textbook
2. **Add More Content**: Use the ingestion script to add additional textbook materials
3. **Configure Analytics**: Set up educator dashboards for monitoring student engagement
4. **Production Deployment**: Follow the deployment guide for production environments