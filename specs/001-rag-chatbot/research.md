# Research Summary: RAG Chatbot for Physical AI Textbook

## Decision: Technology Stack Selection
**Rationale**: Selected the technology stack based on the feature requirements and constraints specified in the feature specification and Part-2 of the project constitution. The stack includes OpenAI ChatKit Agents SDK with Gemini LLM provider via OpenAI-compatible endpoint, FastAPI, Qdrant Cloud, Neon Serverless Postgres, and Docusaurus integration. This aligns with the constitution's Gemini LLM Integration principle and performance reliability requirements.

## Decision: Architecture Pattern
**Rationale**: Chose a web application architecture with separate backend and frontend components to properly separate concerns. The backend handles RAG processing, LLM integration, and data storage, while the frontend provides the chat interface within the Docusaurus textbook. This supports the constitution's Student-Centric Design principle by providing immediate, contextual responses tailored to the educational context.

## Decision: RAG Implementation Approach
**Rationale**: Selected Retrieval-Augmented Generation methodology as the core approach to ensure responses are grounded in textbook content. This aligns with the requirement to answer questions "strictly from the book's five chapters" and prevents hallucinations, supporting the constitution's Content Fidelity principle and No Hallucinations requirement.

## Decision: Gemini LLM Integration
**Rationale**: Implemented Gemini LLM provider via OpenAI-compatible endpoint using the AsyncOpenAI base provider with OpenAIChatCompletionsModel wrapper on gemini-2.5-flash pattern as specified in Part-2 of the constitution. This follows the exact implementation pattern required by the constitution's Part-2 standards.

## Decision: Vector Database Selection
**Rationale**: Chose Qdrant Cloud (Free Tier) for vector storage based on the specification requirements. Qdrant provides efficient similarity search capabilities needed for RAG systems and offers a free tier that meets the 1GB storage requirement. This supports the performance reliability requirements from the constitution's Part-2.

## Decision: Database for Conversation History
**Rationale**: Selected Neon Serverless Postgres for storing conversation history based on the specification requirements. This provides a reliable, scalable solution for storing user interactions and maintaining conversation context, supporting the 99.9% availability requirement from the constitution.

## Decision: Content Processing Strategy
**Rationale**: Implemented 512-token segments for embedding generation as specified in FR-076 and constitution Part-2. This provides a good balance between context preservation and retrieval efficiency for the RAG system while meeting the content processing requirements.

## Decision: SSE Streaming Implementation
**Rationale**: Implemented Server-Sent Events (SSE) streaming for real-time chat responses as required by FR-008 and the constitution's Part-2 SSE streaming requirements. This includes proper citation tracking during generation and connection stability during long-running responses.

## Decision: Rate Limiting Strategy
**Rationale**: Implemented 100 requests per user per minute rate limiting as specified in FR-071 and constitution Part-2 to prevent abuse while allowing reasonable usage for educational purposes. This meets the security and rate limiting requirements.

## Decision: Content Validation Approach
**Rationale**: Implemented content scope validation to ensure all responses are grounded in textbook content only, with fallback responses when content is not available. This aligns with the constitution's Content Scope Validation principle and prevents hallucinations.

## Alternatives Considered

### Alternative: Different LLM Providers
- **OpenAI GPT**: Direct integration possible but constitution Part-2 specifically requires Gemini LLM provider via OpenAI-compatible endpoint
- **Anthropic Claude**: Alternative provider but doesn't meet constitution's Gemini LLM Integration requirement
- **Selected**: Gemini via OpenAI-compatible endpoint as mandated by constitution Part-2

### Alternative: Different Vector Database Options
- **Pinecone**: More expensive, more managed features but exceeds budget constraints and doesn't specifically support the required performance metrics
- **Weaviate**: Self-hosted option but adds operational complexity that could impact the 99.9% availability requirement
- **Milvus**: Open-source but requires more infrastructure management
- **Selected**: Qdrant Cloud for its balance of managed service, performance capabilities, and free tier availability that meets constitution requirements

### Alternative: Different Architecture Patterns
- **Monolithic**: Single codebase but doesn't separate concerns effectively and could impact performance reliability requirements
- **Serverless functions**: Could work but FastAPI provides better state management for conversations and more predictable performance for the 5-second response time requirement
- **Selected**: Backend/Frontend separation for better maintainability, scalability, and ability to meet constitution's performance reliability requirements

### Alternative: Different Streaming Approaches
- **WebSocket**: More complex implementation but provides bidirectional communication
- **Long polling**: Simpler but less efficient than SSE for the real-time chat responses required
- **Selected**: Server-Sent Events (SSE) as specifically required by FR-008 and constitution Part-2 for real-time responses with proper citation tracking