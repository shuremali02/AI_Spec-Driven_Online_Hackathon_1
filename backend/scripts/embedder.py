import os
from typing import List, Dict, Any, Optional
import logging

from openai import AsyncOpenAI

logger = logging.getLogger(__name__)

class Embedder:
    def __init__(self):
        """
        Initialize the embedder with OpenAI-compatible API
        """
        # Use OpenAI-compatible endpoint for Gemini
        base_url = os.getenv("OPENAI_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai/")
        api_key = os.getenv("OPENAI_API_KEY")

        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")

        self.client = AsyncOpenAI(
            base_url=base_url,
            api_key=api_key
        )

        # Use Gemini embedding model
        self.model = os.getenv("EMBEDDING_MODEL", "text-embedding-004")  # Default to a common embedding model
        self.dimension = int(os.getenv("EMBEDDING_DIMENSION", "768"))  # Default to 768

    async def generate_embedding(self, text: str) -> List[float]:
        """
        Generate embedding for a single text
        """
        try:
            response = await self.client.embeddings.create(
                input=text,
                model=self.model
            )

            embedding = response.data[0].embedding
            return embedding
        except Exception as e:
            logger.error(f"Failed to generate embedding: {e}")
            raise

    async def generate_embeddings_batch(self, texts: List[str]) -> List[List[float]]:
        """
        Generate embeddings for a batch of texts
        """
        try:
            # Process in smaller batches to avoid rate limits
            batch_size = 20  # Typical safe batch size
            all_embeddings = []

            for i in range(0, len(texts), batch_size):
                batch = texts[i:i + batch_size]
                response = await self.client.embeddings.create(
                    input=batch,
                    model=self.model
                )

                batch_embeddings = [item.embedding for item in response.data]
                all_embeddings.extend(batch_embeddings)

            return all_embeddings
        except Exception as e:
            logger.error(f"Failed to generate embeddings batch: {e}")
            raise

    async def get_embedding_dimension(self) -> int:
        """
        Get the dimension of the embeddings
        """
        # Test with a short text to determine actual dimension
        test_embedding = await self.generate_embedding("test")
        return len(test_embedding)


# Lazy-initialized embedder instance
_embedder: Optional[Embedder] = None

def get_embedder() -> Embedder:
    """Get or create the Embedder instance (lazy initialization)"""
    global _embedder
    if _embedder is None:
        _embedder = Embedder()
    return _embedder
