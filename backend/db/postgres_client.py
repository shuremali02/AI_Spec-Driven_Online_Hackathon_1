from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession, AsyncEngine
from sqlalchemy.orm import sessionmaker
from typing import AsyncGenerator, Optional
import os
from contextlib import asynccontextmanager

# Lazy-initialized engine and session maker
_engine: Optional[AsyncEngine] = None
_AsyncSessionLocal: Optional[sessionmaker] = None

def _get_database_url() -> str:
    """Get and format the database URL for async connection"""
    url = os.getenv("DATABASE_URL", "postgresql://user:password@localhost/rag_chatbot")

    # Convert to asyncpg format if needed
    if url.startswith("postgresql://"):
        url = url.replace("postgresql://", "postgresql+asyncpg://", 1)
    elif url.startswith("postgres://"):
        url = url.replace("postgres://", "postgresql+asyncpg://", 1)

    # Remove sslmode parameter (asyncpg uses 'ssl' instead)
    # Also remove channel_binding as it's not supported by asyncpg
    if "?" in url:
        base_url, params = url.split("?", 1)
        param_list = params.split("&")
        filtered_params = []
        for param in param_list:
            # Skip sslmode and channel_binding - asyncpg handles SSL differently
            if not param.startswith("sslmode=") and not param.startswith("channel_binding="):
                filtered_params.append(param)
        if filtered_params:
            url = base_url + "?" + "&".join(filtered_params)
        else:
            url = base_url

    return url

def get_engine() -> AsyncEngine:
    """Get or create the async engine (lazy initialization)"""
    global _engine
    if _engine is None:
        database_url = _get_database_url()
        # For Neon Postgres and other cloud providers, enable SSL
        _engine = create_async_engine(
            database_url,
            pool_size=5,
            max_overflow=10,
            pool_pre_ping=True,
            pool_recycle=300,
            connect_args={"ssl": True}  # Enable SSL for asyncpg
        )
    return _engine

def get_session_maker() -> sessionmaker:
    """Get or create the session maker (lazy initialization)"""
    global _AsyncSessionLocal
    if _AsyncSessionLocal is None:
        _AsyncSessionLocal = sessionmaker(
            get_engine(),
            class_=AsyncSession,
            expire_on_commit=False
        )
    return _AsyncSessionLocal

async def get_db_session() -> AsyncGenerator[AsyncSession, None]:
    """
    Get database session for dependency injection
    """
    session_maker = get_session_maker()
    async with session_maker() as session:
        yield session
        await session.close()

async def init_db():
    """
    Initialize the database by creating all tables
    """
    from .models import Base

    engine = get_engine()
    async with engine.begin() as conn:
        # Create all tables
        await conn.run_sync(Base.metadata.create_all)

async def close_db():
    """
    Close the database engine
    """
    global _engine
    if _engine is not None:
        await _engine.dispose()
        _engine = None