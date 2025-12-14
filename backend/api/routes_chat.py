from fastapi import APIRouter, Depends, HTTPException, status, Request
from sqlalchemy.ext.asyncio import AsyncSession
from typing import List
import uuid
import logging

from db.postgres_client import get_db_session
from db.models import Conversation, Message, Citation as DBCitation
from agents.rag_agent import get_rag_agent
from api.validation import (
    ConversationCreateRequest, MessageCreateRequest, validate_and_sanitize_query,
    SearchRequest, FeedbackRequest
)
from api.security_logger import security_logger
from api.middleware import get_client_ip

logger = logging.getLogger(__name__)
router = APIRouter(prefix="/v1", tags=["chat"])

@router.post("/conversations", status_code=status.HTTP_201_CREATED)
async def create_conversation(
    request: Request,
    conversation_data: ConversationCreateRequest,
    db_session: AsyncSession = Depends(get_db_session)
):
    """
    Create a new conversation session for a user (FR-025)
    """
    try:
        # Log the API access
        client_ip = get_client_ip(request)
        user_id = conversation_data.user_id or conversation_data.session_id or "anonymous"

        security_logger.log_api_access(
            user_id=user_id,
            ip_address=client_ip,
            endpoint="/v1/conversations",
            method="POST",
            status_code=201
        )

        # Create new conversation
        # user_id is optional - for anonymous users, we use session_id for tracking
        conversation = Conversation(
            user_id=None,  # Anonymous conversation
            session_id=conversation_data.session_id,  # Track by session
            title=None  # Will be set when first message is added
        )

        db_session.add(conversation)
        await db_session.flush()  # Get the ID without committing

        # If there's an initial query, process it
        if conversation_data.initial_query:
            sanitized_query = validate_and_sanitize_query(conversation_data.initial_query)

            # Create user message
            user_message = Message(
                conversation_id=conversation.id,
                sender_type="user",
                content=sanitized_query,
                message_type="query"
            )
            db_session.add(user_message)
            await db_session.flush()

            # Generate RAG response
            rag_response = await get_rag_agent().query(sanitized_query)

            # Create system response message
            system_message = Message(
                conversation_id=conversation.id,
                sender_type="system",
                content=rag_response.content,
                message_type="response",
                citations=[citation.dict() for citation in rag_response.citations]
            )
            db_session.add(system_message)

            # Update conversation title based on first query
            conversation.title = sanitized_query[:100] + "..." if len(sanitized_query) > 100 else sanitized_query

        await db_session.commit()

        return {
            "conversation_id": str(conversation.id),
            "created_at": conversation.created_at.isoformat(),
            "title": conversation.title,
            "is_active": conversation.is_active
        }

    except Exception as e:
        logger.error(f"Error creating conversation: {e}")
        await db_session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to create conversation"
        )


@router.post("/conversations/{conversation_id}/messages")
async def send_message(
    request: Request,
    conversation_id: str,
    message_data: MessageCreateRequest,
    db_session: AsyncSession = Depends(get_db_session)
):
    """
    Send a message in a conversation and receive a response from the RAG system (FR-001, FR-002, FR-008, FR-009)
    """
    try:
        # Validate and sanitize the content
        sanitized_content = validate_and_sanitize_query(message_data.content)

        # Log the API access
        client_ip = get_client_ip(request)
        user_id = request.headers.get('x-user-id', 'anonymous')

        security_logger.log_api_access(
            user_id=user_id,
            ip_address=client_ip,
            endpoint=f"/v1/conversations/{conversation_id}/messages",
            method="POST",
            status_code=200
        )

        # Verify conversation exists
        conversation_uuid = uuid.UUID(conversation_id)
        conversation = await db_session.get(Conversation, conversation_uuid)
        if not conversation:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Conversation not found"
            )

        # Create user message
        user_message = Message(
            conversation_id=conversation.id,
            sender_type="user",
            content=sanitized_content,
            message_type=message_data.message_type
        )
        db_session.add(user_message)
        await db_session.flush()

        # Generate RAG response
        rag_response = await get_rag_agent().query(sanitized_content)

        # Create system response message
        system_message = Message(
            conversation_id=conversation.id,
            sender_type="system",
            content=rag_response.content,
            message_type="response",
            citations=[citation.dict() for citation in rag_response.citations]
        )
        db_session.add(system_message)

        await db_session.commit()

        # Format response with citations
        formatted_citations = [
            {
                "chapter_title": citation.chapter_title,
                "section_title": citation.section_title,
                "url_path": citation.url_path,
                "confidence_score": citation.confidence_score,
                "content_snippet": citation.content_snippet
            }
            for citation in rag_response.citations
        ]

        return {
            "message_id": str(system_message.id),
            "conversation_id": str(conversation.id),
            "sender_type": system_message.sender_type,
            "content": system_message.content,
            "citations": formatted_citations,
            "created_at": system_message.created_at.isoformat(),
            "message_type": system_message.message_type
        }

    except HTTPException:
        # Re-raise HTTP exceptions as-is
        raise
    except Exception as e:
        logger.error(f"Error sending message: {e}")
        await db_session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to process message"
        )


@router.get("/conversations/{conversation_id}")
async def get_conversation(
    request: Request,
    conversation_id: str,
    db_session: AsyncSession = Depends(get_db_session)
):
    """
    Retrieve the history of messages in a conversation (FR-025)
    """
    try:
        # Log the API access
        client_ip = get_client_ip(request)
        user_id = request.headers.get('x-user-id', 'anonymous')

        security_logger.log_api_access(
            user_id=user_id,
            ip_address=client_ip,
            endpoint=f"/v1/conversations/{conversation_id}",
            method="GET",
            status_code=200
        )

        conversation_uuid = uuid.UUID(conversation_id)
        conversation = await db_session.get(Conversation, conversation_uuid)
        if not conversation:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Conversation not found"
            )

        # Get all messages in the conversation
        from sqlalchemy import select
        result = await db_session.execute(
            select(Message)
            .where(Message.conversation_id == conversation.id)
            .order_by(Message.created_at)
        )
        messages = result.scalars().all()

        formatted_messages = [
            {
                "message_id": str(msg.id),
                "sender_type": msg.sender_type,
                "content": msg.content,
                "created_at": msg.created_at.isoformat(),
                "citations": msg.citations if msg.citations else [],
                "message_type": msg.message_type
            }
            for msg in messages
        ]

        return {
            "conversation_id": str(conversation.id),
            "title": conversation.title,
            "created_at": conversation.created_at.isoformat(),
            "updated_at": conversation.updated_at.isoformat(),
            "is_active": conversation.is_active,
            "messages": formatted_messages
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting conversation: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve conversation"
        )


@router.delete("/conversations/{conversation_id}/messages")
async def clear_conversation_messages(
    request: Request,
    conversation_id: str,
    db_session: AsyncSession = Depends(get_db_session)
):
    """
    Clear all messages in a conversation while keeping the conversation (FR-026)
    """
    try:
        # Log the API access
        client_ip = get_client_ip(request)
        user_id = request.headers.get('x-user-id', 'anonymous')

        security_logger.log_api_access(
            user_id=user_id,
            ip_address=client_ip,
            endpoint=f"/v1/conversations/{conversation_id}/messages",
            method="DELETE",
            status_code=204
        )

        conversation_uuid = uuid.UUID(conversation_id)
        conversation = await db_session.get(Conversation, conversation_uuid)
        if not conversation:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Conversation not found"
            )

        # Delete all messages in the conversation
        from sqlalchemy import delete
        await db_session.execute(
            delete(Message).where(Message.conversation_id == conversation.id)
        )

        await db_session.commit()

        return None  # 204 No Content

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error clearing conversation messages: {e}")
        await db_session.rollback()
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to clear conversation messages"
        )


@router.get("/messages/{message_id}/citations")
async def get_message_citations(
    request: Request,
    message_id: str,
    db_session: AsyncSession = Depends(get_db_session)
):
    """
    Retrieve citations for a specific response message (FR-009, FR-019)
    """
    try:
        # Log the API access
        client_ip = get_client_ip(request)
        user_id = request.headers.get('x-user-id', 'anonymous')

        security_logger.log_api_access(
            user_id=user_id,
            ip_address=client_ip,
            endpoint=f"/v1/messages/{message_id}/citations",
            method="GET",
            status_code=200
        )

        message_uuid = uuid.UUID(message_id)
        message = await db_session.get(Message, message_uuid)
        if not message:
            raise HTTPException(
                status_code=status.HTTP_404_NOT_FOUND,
                detail="Message not found"
            )

        # Extract citations from the message
        citations = message.citations if message.citations else []

        return {
            "message_id": str(message.id),
            "citations": citations
        }

    except HTTPException:
        raise
    except Exception as e:
        logger.error(f"Error getting message citations: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to retrieve citations"
        )


@router.post("/feedback")
async def submit_feedback(
    request: Request,
    feedback_data: FeedbackRequest,
    db_session: AsyncSession = Depends(get_db_session)
):
    """
    Allow users to report inaccurate or problematic responses (FR-038)
    """
    try:
        # Log the API access
        client_ip = get_client_ip(request)
        user_id = request.headers.get('x-user-id', 'anonymous')

        security_logger.log_api_access(
            user_id=user_id,
            ip_address=client_ip,
            endpoint="/v1/feedback",
            method="POST",
            status_code=200
        )

        # In a full implementation, we would store feedback in a feedback table
        # For now, we'll just log the feedback for monitoring
        logger.info(f"Feedback received: type={feedback_data.feedback_type}, "
                   f"message_id={feedback_data.message_id}, comment={feedback_data.comment}")

        return {
            "feedback_id": str(uuid.uuid4()),  # In a real implementation, store in DB
            "status": "submitted"
        }

    except Exception as e:
        logger.error(f"Error submitting feedback: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to submit feedback"
        )


@router.post("/search")
async def search_content(
    request: Request,
    search_data: SearchRequest,
    db_session: AsyncSession = Depends(get_db_session)
):
    """
    Allow users to search textbook content directly (FR-042)
    """
    try:
        # Log the API access
        client_ip = get_client_ip(request)
        user_id = request.headers.get('x-user-id', 'anonymous')

        security_logger.log_api_access(
            user_id=user_id,
            ip_address=client_ip,
            endpoint="/v1/search",
            method="POST",
            status_code=200
        )

        # Validate and sanitize the query
        sanitized_query = validate_and_sanitize_query(search_data.query)

        # In a full implementation, this would search the vector database
        # For now, we'll use the RAG agent's retrieval functionality
        context_chunks = await get_rag_agent().retrieve_context(sanitized_query)

        results = []
        for chunk in context_chunks[:search_data.max_results]:
            results.append({
                "content_id": chunk.get('id', ''),
                "chapter_title": chunk.get('chapter_id', 'Unknown Chapter'),
                "section_title": chunk.get('section_path', 'Unknown Section'),
                "url_path": chunk.get('section_path', '#'),
                "content_snippet": chunk.get('content', '')[:300] + "..." if len(chunk.get('content', '')) > 300 else chunk.get('content', ''),
                "relevance_score": chunk.get('score', 0.0)
            })

        return {
            "query": sanitized_query,
            "results": results,
            "total_results": len(results)
        }

    except Exception as e:
        logger.error(f"Error searching content: {e}")
        raise HTTPException(
            status_code=status.HTTP_500_INTERNAL_SERVER_ERROR,
            detail="Failed to search content"
        )