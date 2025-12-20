import os
from typing import Optional

class Config:
    """
    Configuration class for the RAG Chatbot backend
    """
    # Database configuration
    DATABASE_URL: str = os.getenv("DATABASE_URL", "postgresql+asyncpg://user:password@localhost/rag_chatbot")

    # Qdrant configuration
    QDRANT_URL: str = os.getenv("QDRANT_URL", "http://localhost:6333")
    QDRANT_API_KEY: Optional[str] = os.getenv("QDRANT_API_KEY")
    QDRANT_COLLECTION_NAME: str = os.getenv("QDRANT_COLLECTION_NAME", "textbook_content_embeddings")

    # OpenAI/Gemini API configuration
    OPENAI_API_KEY: str = os.getenv("OPENAI_API_KEY", "")
    OPENAI_BASE_URL: str = os.getenv("OPENAI_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai/")
    GEMINI_MODEL: str = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")

    # Embedding configuration
    EMBEDDING_MODEL: str = os.getenv("EMBEDDING_MODEL", "text-embedding-004")
    EMBEDDING_DIMENSION: int = int(os.getenv("EMBEDDING_DIMENSION", "768"))

    # Application settings
    APP_ENV: str = os.getenv("APP_ENV", "development")
    LOG_LEVEL: str = os.getenv("LOG_LEVEL", "INFO")

    # Rate limiting
    RATE_LIMIT_REQUESTS: int = int(os.getenv("RATE_LIMIT_REQUESTS", "100"))
    RATE_LIMIT_WINDOW: int = int(os.getenv("RATE_LIMIT_WINDOW", "60"))  # in seconds

    # RAG settings
    RAG_TOP_K: int = int(os.getenv("RAG_TOP_K", "5"))
    RAG_MIN_SCORE: float = float(os.getenv("RAG_MIN_SCORE", "0.3"))
    MAX_TOKENS_PER_CHUNK: int = int(os.getenv("MAX_TOKENS_PER_CHUNK", "512"))
    MAX_RESPONSE_TOKENS: int = int(os.getenv("MAX_RESPONSE_TOKENS", "2000"))

    # CORS settings
    ALLOWED_ORIGINS: list = os.getenv("ALLOWED_ORIGINS", "*").split(",")

    # SSE Streaming
    SSE_STREAMING_ENABLED: bool = os.getenv("SSE_STREAMING_ENABLED", "true").lower() == "true"

    # Personalization settings
    PERSONALIZE_MAX_TOKENS: int = int(os.getenv("PERSONALIZE_MAX_TOKENS", "8192"))
    PERSONALIZE_RATE_LIMIT: int = int(os.getenv("PERSONALIZE_RATE_LIMIT", "5"))
    PERSONALIZE_RATE_LIMIT_WINDOW: int = int(os.getenv("PERSONALIZE_RATE_LIMIT_WINDOW", "60"))  # in seconds
    PERSONALIZE_TIMEOUT: int = int(os.getenv("PERSONALIZE_TIMEOUT", "60"))  # in seconds
    AUTH_BACKEND_URL: str = os.getenv("AUTH_BACKEND_URL", "http://localhost:3001")

    @classmethod
    def validate(cls):
        """
        Validate that required configuration values are set
        """
        errors = []

        if not cls.OPENAI_API_KEY:
            errors.append("OPENAI_API_KEY is required")

        if cls.APP_ENV not in ["development", "staging", "production"]:
            errors.append(f"APP_ENV must be one of: development, staging, production. Got: {cls.APP_ENV}")

        if errors:
            raise ValueError(f"Configuration validation failed: {'; '.join(errors)}")


# Validate configuration on import
Config.validate()