from openai import AsyncOpenAI
import os
from typing import List, Dict, Any, Optional
import logging
from pydantic import BaseModel
import asyncio

from vector.qdrant_client import get_qdrant_manager
from scripts.embedder import get_embedder
from agents.system_prompt import get_system_prompt

logger = logging.getLogger(__name__)

class Citation(BaseModel):
    """
    Citation model for tracking textbook references
    """
    chapter_title: str
    section_title: str
    url_path: str
    confidence_score: float
    content_snippet: str


class RAGResponse(BaseModel):
    """
    Response model for RAG queries
    """
    content: str
    citations: List[Citation]
    query: str
    retrieved_chunks_count: int


class RAGAgent:
    """
    RAG Agent that retrieves relevant content and generates responses
    """
    def __init__(self):
        # Initialize OpenAI client for Gemini
        # Note: base_url should NOT have trailing slash for Gemini API
        base_url = os.getenv("OPENAI_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai")
        # Remove trailing slash if present
        base_url = base_url.rstrip('/')
        api_key = os.getenv("OPENAI_API_KEY")

        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")

        self.client = AsyncOpenAI(
            base_url=base_url,
            api_key=api_key
        )

        self.model = os.getenv("GEMINI_MODEL", "gemini-2.0-flash")
        self.top_k = int(os.getenv("RAG_TOP_K", "5"))
        self.min_score = float(os.getenv("RAG_MIN_SCORE", "0.1"))  # Lowered threshold
        self.max_tokens = int(os.getenv("MAX_RESPONSE_TOKENS", "2000"))

    async def retrieve_context(self, query: str) -> List[Dict[str, Any]]:
        """
        Retrieve relevant context from vector database
        """
        try:
            # Generate embedding for the query
            logger.info(f"Generating embedding for query: {query}")
            embedder = get_embedder()
            query_embedding = await embedder.generate_embedding(query)
            logger.info(f"Embedding generated, length: {len(query_embedding)}")

            # Search in Qdrant
            qdrant_manager = get_qdrant_manager()
            logger.info(f"Searching Qdrant with top_k={self.top_k}, min_score={self.min_score}")
            search_results = await qdrant_manager.search_similar(
                query_vector=query_embedding,
                top_k=self.top_k,
                min_score=self.min_score
            )
            logger.info(f"Qdrant returned {len(search_results)} results")

            # Format results
            retrieved_chunks = []
            for result in search_results:
                payload = result['payload']
                logger.info(f"Result score: {result['score']}, section: {payload.get('section_path', 'unknown')}")
                retrieved_chunks.append({
                    'id': result['id'],
                    'score': result['score'],
                    'content': payload.get('content', ''),
                    'chapter_id': payload.get('chapter_id', ''),
                    'section_path': payload.get('section_path', ''),
                    'token_count': payload.get('token_count', 0),
                    'content_type': payload.get('content_type', 'text'),
                    'chunk_index': payload.get('chunk_index', 0)
                })

            logger.info(f"Retrieved {len(retrieved_chunks)} chunks for query: {query[:50]}...")
            return retrieved_chunks

        except Exception as e:
            logger.error(f"Error retrieving context: {e}")
            import traceback
            traceback.print_exc()
            return []

    async def generate_response(self, query: str, context_chunks: List[Dict[str, Any]]) -> RAGResponse:
        """
        Generate a response based on query and retrieved context
        """
        try:
            # Build context string from retrieved chunks
            context_str = ""
            citations = []

            for chunk in context_chunks:
                if chunk.get('content'):
                    context_str += f"\n\n[Relevant Content]\n{chunk['content']}\n"

                    # Create citation
                    citation = Citation(
                        chapter_title=chunk.get('chapter_id', 'Unknown Chapter'),
                        section_title=chunk.get('section_path', 'Unknown Section'),
                        url_path=chunk.get('section_path', '#'),
                        confidence_score=chunk.get('score', 0.0),
                        content_snippet=chunk['content'][:200] + "..." if len(chunk['content']) > 200 else chunk['content']
                    )
                    citations.append(citation)

            # Build the full prompt
            system_prompt = get_system_prompt()
            user_prompt = f"""
            Query: {query}

            Retrieved Context:
            {context_str}

            Please provide a comprehensive answer based on the textbook content.
            """

            # Call the LLM
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=self.max_tokens,
                temperature=0.3  # Lower temperature for more consistent, factual responses
            )

            response_content = response.choices[0].message.content

            return RAGResponse(
                content=response_content,
                citations=citations,
                query=query,
                retrieved_chunks_count=len(context_chunks)
            )

        except Exception as e:
            logger.error(f"Error generating response: {e}")
            import traceback
            traceback.print_exc()
            # Return a fallback response when generation fails
            return RAGResponse(
                content="I'm sorry, but I couldn't generate a response at this time. Please try again later.",
                citations=[],
                query=query,
                retrieved_chunks_count=0
            )

    def _is_casual_message(self, query: str) -> bool:
        """
        Check if the query is a casual/greeting message that doesn't need RAG
        """
        casual_patterns = [
            'hi', 'hello', 'hey', 'hii', 'hiii', 'yo',
            'good morning', 'good afternoon', 'good evening', 'good night',
            'thanks', 'thank you', 'thankyou', 'thx',
            'bye', 'goodbye', 'see you', 'ok', 'okay',
            'how are you', 'whats up', "what's up", 'wassup',
            'nice', 'great', 'awesome', 'cool',
            'yes', 'no', 'yeah', 'yep', 'nope',
            'sorry', 'help me', 'can you help'
        ]
        query_lower = query.lower().strip().rstrip('!?.،')
        return len(query_lower) < 20 and any(
            query_lower == p or query_lower.startswith(p) for p in casual_patterns
        )

    async def generate_casual_response(self, query: str) -> RAGResponse:
        """
        Generate a conversational response using LLM without RAG context
        """
        try:
            system_prompt = get_system_prompt()
            user_prompt = f"""User message: {query}

This is a casual/greeting message. Respond naturally and friendly as a helpful robotics tutor. Keep it brief and conversational."""

            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=300,
                temperature=0.7
            )

            return RAGResponse(
                content=response.choices[0].message.content,
                citations=[],
                query=query,
                retrieved_chunks_count=0
            )
        except Exception as e:
            logger.error(f"Error generating casual response: {e}")
            return RAGResponse(
                content="Hello! I'm your robotics textbook assistant. How can I help you learn about ROS 2 and Physical AI today?",
                citations=[],
                query=query,
                retrieved_chunks_count=0
            )

    async def query(self, query: str) -> RAGResponse:
        """
        Main method to process a query through RAG
        """
        logger.info(f"Processing RAG query: {query[:50]}...")

        # Check for casual messages - use LLM directly without RAG
        if self._is_casual_message(query):
            logger.info("Detected casual message, generating conversational response")
            return await self.generate_casual_response(query)

        # Retrieve relevant context for knowledge questions
        context_chunks = await self.retrieve_context(query)

        if not context_chunks:
            # No relevant content - still use LLM to respond helpfully
            return await self.generate_casual_response(query + "\n(Note: No relevant textbook content found for this query)")

        # Generate response based on context
        response = await self.generate_response(query, context_chunks)

        return response

    async def query_with_streaming(self, query: str):
        """
        Generator method for streaming responses via SSE
        """
        logger.info(f"Processing streaming RAG query: {query[:50]}...")

        # Retrieve relevant context
        context_chunks = await self.retrieve_context(query)

        if not context_chunks:
            # No relevant content found
            yield {
                "content_chunk": "I couldn't find relevant content in the textbook to answer your question. Please try rephrasing your question or check if the topic is covered in the textbook.",
                "is_complete": True,
                "citations": []
            }
            return

        # Build context string from retrieved chunks
        context_str = ""
        citations = []

        for chunk in context_chunks:
            if chunk.get('content'):
                context_str += f"\n\n[Relevant Content]\n{chunk['content']}\n"

                # Create citation
                citation = Citation(
                    chapter_title=chunk.get('chapter_id', 'Unknown Chapter'),
                    section_title=chunk.get('section_path', 'Unknown Section'),
                    url_path=chunk.get('section_path', '#'),
                    confidence_score=chunk.get('score', 0.0),
                    content_snippet=chunk['content'][:200] + "..." if len(chunk['content']) > 200 else chunk['content']
                )
                citations.append(citation)

        # Build the full prompt
        system_prompt = get_system_prompt()
        user_prompt = f"""
        Query: {query}

        Retrieved Context:
        {context_str}

        Please provide a comprehensive answer based on the textbook content.
        """

        try:
            # Stream the response from the LLM
            stream = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=self.max_tokens,
                temperature=0.3,
                stream=True
            )

            full_response = ""
            async for chunk in stream:
                if chunk.choices and chunk.choices[0].delta.content:
                    content_chunk = chunk.choices[0].delta.content
                    full_response += content_chunk

                    yield {
                        "content_chunk": content_chunk,
                        "is_complete": False,
                        "citations": []
                    }

            # Send final chunk with citations
            yield {
                "content_chunk": "",
                "is_complete": True,
                "citations": [c.dict() for c in citations]
            }

        except Exception as e:
            logger.error(f"Error in streaming response: {e}")
            yield {
                "content_chunk": "I'm sorry, but I encountered an error while processing your request. Please try again later.",
                "is_complete": True,
                "citations": []
            }


# Lazy-initialized RAG agent instance
_rag_agent: Optional[RAGAgent] = None

def get_rag_agent() -> RAGAgent:
    """Get or create the RAG agent instance (lazy initialization)"""
    global _rag_agent
    if _rag_agent is None:
        _rag_agent = RAGAgent()
    return _rag_agent