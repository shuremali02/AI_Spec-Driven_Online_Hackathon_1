#!/usr/bin/env python3
"""
Content ingestion script for the RAG Chatbot
Parses textbook content, chunks it, generates embeddings, and stores in vector DB

Usage:
    cd backend
    uv run python -m scripts.ingest_content --source-path ../book-write/docs

Or directly:
    uv run python scripts/ingest_content.py --source-path ../book-write/docs
"""
import asyncio
import os
import sys
from pathlib import Path
from typing import List, Dict, Any
import logging
import argparse
import hashlib
import uuid

# Add the backend directory to the path so we can import modules
sys.path.insert(0, str(Path(__file__).parent.parent))

# Load environment variables
from dotenv import load_dotenv
load_dotenv()

from db.postgres_client import get_session_maker, init_db
from db.models import TextbookContent
from vector.qdrant_client import get_qdrant_manager
from scripts.chunker import ContentChunker
from scripts.embedder import get_embedder

# Set up logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


async def process_markdown_file(file_path: Path, base_path: Path, chunker: ContentChunker) -> List[Dict[str, Any]]:
    """
    Process a single markdown file: read, chunk, and prepare for embedding
    """
    logger.info(f"Processing file: {file_path}")

    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()

    # Generate unique chapter ID based on file path
    try:
        relative_path = file_path.relative_to(base_path).as_posix()
    except ValueError:
        relative_path = file_path.name

    chapter_id = hashlib.md5(relative_path.encode()).hexdigest()[:12]

    # Chunk the content
    chunks = chunker.chunk_markdown(content, str(file_path))

    processed_chunks = []
    for chunk in chunks:
        # Generate deterministic UUID from content hash (UUID v5 using namespace)
        content_hash_str = f"{relative_path}_{chunk['content']}_{chunk['chunk_index']}"
        # Use UUID5 with a namespace to generate deterministic UUIDs
        embedding_uuid = uuid.uuid5(uuid.NAMESPACE_DNS, content_hash_str)
        content_uuid = uuid.uuid5(uuid.NAMESPACE_DNS, f"content_{content_hash_str}")

        processed_chunk = {
            'textbook_content_id': str(content_uuid),
            'chapter_id': chapter_id,
            'section_path': relative_path,
            'content_text': chunk['content'],
            'content_type': 'text',
            'metadata_content': {
                'source_file': str(file_path),
                'section_header': chunk.get('section_header', ''),
                'section_level': chunk.get('section_level', 0),
                'chunk_index': chunk['chunk_index']
            },
            'embedding_id': str(embedding_uuid),
            'token_count': chunk['token_count'],
        }

        processed_chunks.append(processed_chunk)

    logger.info(f"Processed {len(processed_chunks)} chunks from {file_path}")
    return processed_chunks


async def generate_and_store_embeddings(chunks: List[Dict[str, Any]], embedder, qdrant_manager) -> int:
    """
    Generate embeddings for chunks and store them in Qdrant
    """
    success_count = 0

    for chunk in chunks:
        try:
            # Generate embedding
            embedding_vector = await embedder.generate_embedding(chunk['content_text'])

            # Store in Qdrant with content in payload
            success = await qdrant_manager.store_embedding(
                embedding_id=chunk['embedding_id'],
                vector=embedding_vector,
                textbook_content_id=chunk['textbook_content_id'],
                chapter_id=chunk['chapter_id'],
                section_path=chunk['section_path'],
                token_count=chunk['token_count'],
                content_type=chunk['content_type'],
                chunk_index=chunk['metadata_content']['chunk_index'],
                content=chunk['content_text']  # Include actual text for RAG
            )

            if success:
                success_count += 1
                logger.debug(f"Stored embedding: {chunk['embedding_id']}")
            else:
                logger.error(f"Failed to store embedding: {chunk['embedding_id']}")

        except Exception as e:
            logger.error(f"Error processing chunk {chunk['embedding_id']}: {e}")
            continue

    return success_count


async def store_content_metadata(session, chunks: List[Dict[str, Any]]) -> int:
    """
    Store content metadata in PostgreSQL
    """
    success_count = 0

    for chunk in chunks:
        try:
            db_content = TextbookContent(
                id=chunk['textbook_content_id'],
                chapter_id=chunk['chapter_id'],
                section_path=chunk['section_path'],
                content_type=chunk['content_type'],
                metadata_content=chunk['metadata_content'],
                embedding_id=chunk['embedding_id'],
                token_count=chunk['token_count']
            )
            session.add(db_content)
            success_count += 1
        except Exception as e:
            logger.error(f"Error adding content to session: {e}")
            continue

    try:
        await session.commit()
        logger.info(f"Committed {success_count} content records to database")
    except Exception as e:
        logger.error(f"Failed to commit to database: {e}")
        await session.rollback()
        return 0

    return success_count


async def ingest_content(source_path: str, chunk_size: int = 512, skip_db: bool = False):
    """
    Main ingestion function
    """
    logger.info(f"Starting content ingestion from: {source_path}")
    logger.info(f"Using chunk size: {chunk_size}")

    source_dir = Path(source_path).resolve()
    if not source_dir.exists():
        logger.error(f"Source path does not exist: {source_path}")
        return

    # Initialize components
    chunker = ContentChunker(max_tokens=chunk_size)
    embedder = get_embedder()
    qdrant_manager = get_qdrant_manager()

    # Verify Qdrant connection
    logger.info("Checking Qdrant connection...")
    if not await qdrant_manager.health():
        logger.error("Qdrant connection failed")
        return

    logger.info("Creating Qdrant collection if it doesn't exist...")
    await qdrant_manager.create_collection()

    # Initialize database if not skipping
    if not skip_db:
        logger.info("Initializing database...")
        await init_db()

    # Find all markdown files
    md_files = list(source_dir.rglob("*.md")) + list(source_dir.rglob("*.mdx"))

    # Filter out node_modules, .git, etc.
    md_files = [f for f in md_files if not any(
        part.startswith('.') or part == 'node_modules'
        for part in f.parts
    )]

    logger.info(f"Found {len(md_files)} markdown files to process")

    if not md_files:
        logger.warning("No markdown files found!")
        return

    total_chunks = 0
    total_embeddings = 0
    total_db_records = 0

    for md_file in md_files:
        try:
            # Process the file (chunk it)
            chunks = await process_markdown_file(md_file, source_dir, chunker)
            total_chunks += len(chunks)

            if not chunks:
                logger.warning(f"No chunks generated from {md_file}")
                continue

            # Generate embeddings and store in Qdrant
            embeddings_stored = await generate_and_store_embeddings(chunks, embedder, qdrant_manager)
            total_embeddings += embeddings_stored

            # Store metadata in PostgreSQL (optional)
            if not skip_db:
                session_maker = get_session_maker()
                async with session_maker() as db_session:
                    db_stored = await store_content_metadata(db_session, chunks)
                    total_db_records += db_stored

            logger.info(f"Completed: {md_file.name} - {len(chunks)} chunks, {embeddings_stored} embeddings")

        except Exception as e:
            logger.error(f"Error processing file {md_file}: {e}")
            import traceback
            traceback.print_exc()
            continue

    logger.info("=" * 60)
    logger.info("Content ingestion completed!")
    logger.info(f"Total chunks processed: {total_chunks}")
    logger.info(f"Total embeddings stored in Qdrant: {total_embeddings}")
    if not skip_db:
        logger.info(f"Total records stored in PostgreSQL: {total_db_records}")
    logger.info("=" * 60)


def main():
    parser = argparse.ArgumentParser(
        description='Ingest textbook content for RAG Chatbot',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    # Ingest from book-write/docs directory
    uv run python scripts/ingest_content.py --source-path ../book-write/docs

    # Ingest with custom chunk size
    uv run python scripts/ingest_content.py --source-path ../book-write/docs --chunk-size 256

    # Skip PostgreSQL storage (only store in Qdrant)
    uv run python scripts/ingest_content.py --source-path ../book-write/docs --skip-db
        """
    )
    parser.add_argument('--source-path', type=str, required=True,
                        help='Path to the source textbook content (e.g., ../book-write/docs)')
    parser.add_argument('--chunk-size', type=int, default=512,
                        help='Maximum tokens per content chunk (default: 512)')
    parser.add_argument('--skip-db', action='store_true',
                        help='Skip PostgreSQL storage, only store embeddings in Qdrant')

    args = parser.parse_args()

    # Run the async ingestion function
    asyncio.run(ingest_content(args.source_path, args.chunk_size, args.skip_db))


if __name__ == "__main__":
    main()
