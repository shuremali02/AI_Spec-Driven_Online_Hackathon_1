from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from contextlib import asynccontextmanager
import logging
import os
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

from api.routes_chat import router as chat_router
from api.personalize import router as personalize_router
from db.postgres_client import init_db, close_db
from vector.qdrant_client import get_qdrant_manager
from api.middleware import rate_limit_middleware

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

@asynccontextmanager
async def lifespan(app: FastAPI):
    """
    Lifespan event handler for FastAPI
    """
    logger.info("Starting up RAG Chatbot backend...")

    # Initialize database
    try:
        await init_db()
        logger.info("Database initialized successfully")
    except Exception as e:
        logger.error(f"Failed to initialize database: {e}")
        raise

    # Initialize vector database
    try:
        qdrant_manager = get_qdrant_manager()
        await qdrant_manager.create_collection()
        logger.info("Qdrant collection created/verified successfully")
    except Exception as e:
        logger.error(f"Failed to initialize Qdrant: {e}")
        raise

    yield  # Application runs here

    # Cleanup
    logger.info("Shutting down RAG Chatbot backend...")
    try:
        await close_db()
        logger.info("Database connection closed")
    except Exception as e:
        logger.error(f"Error closing database: {e}")

    try:
        qdrant_manager = get_qdrant_manager()
        await qdrant_manager.close()
        logger.info("Qdrant connection closed")
    except Exception as e:
        logger.error(f"Error closing Qdrant: {e}")


# Create FastAPI app
app = FastAPI(
    title="RAG Chatbot for Physical AI Textbook",
    description="API for RAG-based chatbot that answers questions about Physical AI & Humanoid Robotics textbook",
    version="0.1.0",
    lifespan=lifespan
)

# Add CORS middleware
app.add_middleware(
    CORSMiddleware,
    allow_origins=os.getenv("ALLOWED_ORIGINS", "*").split(","),
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Add rate limiting middleware
app.middleware("http")(rate_limit_middleware)

# Include API routes
app.include_router(chat_router)
app.include_router(personalize_router)

@app.get("/")
async def root():
    """
    Root endpoint for health check
    """
    return {
        "message": "RAG Chatbot API for Physical AI Textbook",
        "status": "healthy",
        "version": "0.1.0"
    }

@app.get("/health")
async def health_check():
    """
    Health check endpoint
    """
    # Check database connection
    db_healthy = True  # In a real implementation, we'd test the DB connection

    # Check vector database connection
    qdrant_manager = get_qdrant_manager()
    vector_healthy = await qdrant_manager.health()

    status = "healthy" if (db_healthy and vector_healthy) else "unhealthy"

    return {
        "status": status,
        "checks": {
            "database": "healthy" if db_healthy else "unhealthy",
            "vector_database": "healthy" if vector_healthy else "unhealthy"
        }
    }

# Error handlers
@app.exception_handler(500)
async def internal_exception_handler(request, exc):
    logger.error(f"Internal server error: {exc}", exc_info=True)
    return {"error": "Internal server error"}

@app.exception_handler(422)
async def validation_exception_handler(request, exc):
    logger.warning(f"Validation error: {exc}")
    return {"error": "Validation error", "details": str(exc)}