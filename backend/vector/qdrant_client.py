from qdrant_client import AsyncQdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import Distance, VectorParams
from typing import List, Dict, Any, Optional
import os
import logging
from uuid import UUID

logger = logging.getLogger(__name__)

class QdrantManager:
    def __init__(self):
        # Get Qdrant configuration from environment
        self.url = os.getenv("QDRANT_URL", "http://localhost:6333")
        self.api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("QDRANT_COLLECTION_NAME", "textbook_content_embeddings")

        # Initialize async Qdrant client
        if self.api_key:
            self.client = AsyncQdrantClient(
                url=self.url,
                api_key=self.api_key,
                prefer_grpc=True
            )
        else:
            self.client = AsyncQdrantClient(
                url=self.url,
                prefer_grpc=True
            )

    async def health(self) -> bool:
        """
        Check if Qdrant is available
        """
        try:
            # Use get_collections() to check connectivity
            await self.client.get_collections()
            return True
        except Exception as e:
            logger.error(f"Qdrant health check failed: {e}")
            return False

    async def create_collection(self):
        """
        Create the textbook content embeddings collection if it doesn't exist
        """
        try:
            # Check if collection exists
            collections = await self.client.get_collections()
            collection_names = [collection.name for collection in collections.collections]

            if self.collection_name not in collection_names:
                # Create collection with specified vector size and payload schema
                await self.client.create_collection(
                    collection_name=self.collection_name,
                    vectors_config=VectorParams(size=768, distance=Distance.COSINE),  # Using 768 for Gemini embeddings
                )

                # Create payload index for faster filtering
                await self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="textbook_content_id",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                await self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="chapter_id",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                await self.client.create_payload_index(
                    collection_name=self.collection_name,
                    field_name="section_path",
                    field_schema=models.PayloadSchemaType.KEYWORD
                )

                logger.info(f"Created Qdrant collection: {self.collection_name}")
            else:
                logger.info(f"Qdrant collection {self.collection_name} already exists")

        except Exception as e:
            logger.error(f"Failed to create Qdrant collection: {e}")
            raise

    async def store_embedding(self,
                             embedding_id: str,
                             vector: List[float],
                             textbook_content_id: str,
                             chapter_id: str,
                             section_path: str,
                             token_count: int,
                             content_type: str,
                             chunk_index: int,
                             content: str = "") -> bool:
        """
        Store an embedding in Qdrant with content text in payload
        """
        try:
            points = [
                models.PointStruct(
                    id=embedding_id,
                    vector=vector,
                    payload={
                        "textbook_content_id": textbook_content_id,
                        "chapter_id": chapter_id,
                        "section_path": section_path,
                        "token_count": token_count,
                        "content_type": content_type,
                        "chunk_index": chunk_index,
                        "content": content  # Store actual text for RAG retrieval
                    }
                )
            ]

            await self.client.upsert(
                collection_name=self.collection_name,
                points=points
            )

            return True
        except Exception as e:
            logger.error(f"Failed to store embedding: {e}")
            return False

    async def search_similar(self,
                           query_vector: List[float],
                           top_k: int = 5,
                           min_score: float = 0.3,
                           filters: Optional[Dict[str, Any]] = None) -> List[Dict[str, Any]]:
        """
        Search for similar embeddings in Qdrant using query_points
        """
        try:
            # Build filters if provided
            qdrant_filters = None
            if filters:
                filter_conditions = []
                for key, value in filters.items():
                    filter_conditions.append(
                        models.FieldCondition(
                            key=key,
                            match=models.MatchValue(value=value)
                        )
                    )

                if filter_conditions:
                    qdrant_filters = models.Filter(
                        must=filter_conditions
                    )

            # Use query_points for AsyncQdrantClient
            search_results = await self.client.query_points(
                collection_name=self.collection_name,
                query=query_vector,
                limit=top_k,
                score_threshold=min_score,
                with_payload=True,
                query_filter=qdrant_filters
            )

            # Format results - query_points returns QueryResponse with .points attribute
            formatted_results = []
            for result in search_results.points:
                formatted_results.append({
                    "id": result.id,
                    "score": result.score,
                    "payload": result.payload
                })

            return formatted_results
        except Exception as e:
            logger.error(f"Search failed: {e}")
            import traceback
            traceback.print_exc()
            return []

    async def get_embedding(self, embedding_id: str) -> Optional[Dict[str, Any]]:
        """
        Get a specific embedding by ID
        """
        try:
            records = await self.client.retrieve(
                collection_name=self.collection_name,
                ids=[embedding_id],
                with_payload=True,
                with_vectors=True
            )

            if records:
                record = records[0]
                return {
                    "id": record.id,
                    "vector": record.vector,
                    "payload": record.payload
                }

            return None
        except Exception as e:
            logger.error(f"Failed to get embedding: {e}")
            return None

    async def delete_embedding(self, embedding_id: str) -> bool:
        """
        Delete an embedding by ID
        """
        try:
            await self.client.delete(
                collection_name=self.collection_name,
                points_selector=models.PointIdsList(
                    points=[embedding_id]
                )
            )
            return True
        except Exception as e:
            logger.error(f"Failed to delete embedding: {e}")
            return False

    async def close(self):
        """
        Close the Qdrant client connection
        """
        await self.client.close()

# Lazy-initialized global instance
_qdrant_manager: Optional[QdrantManager] = None

def get_qdrant_manager() -> QdrantManager:
    """Get or create the Qdrant manager instance (lazy initialization)"""
    global _qdrant_manager
    if _qdrant_manager is None:
        _qdrant_manager = QdrantManager()
    return _qdrant_manager

# For backward compatibility, use property pattern
class _QdrantManagerProxy:
    """Proxy class for lazy initialization of QdrantManager"""
    def __getattr__(self, name):
        return getattr(get_qdrant_manager(), name)

qdrant_manager = _QdrantManagerProxy()