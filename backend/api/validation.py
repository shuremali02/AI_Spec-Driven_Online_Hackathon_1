import re
from typing import Optional, Union
import html
import logging
from pydantic import BaseModel, validator, ValidationError
from fastapi import HTTPException, status

logger = logging.getLogger(__name__)

class QueryValidation:
    """
    Input validation and sanitization for user queries
    """

    # Dangerous patterns to block
    DANGEROUS_PATTERNS = [
        r'system\(',  # System command execution
        r'exec\(',     # Code execution
        r'eval\(',     # Code evaluation
        r'import\s+\w+',  # Import statements
        r'__\w+__',   # Special methods
        r'os\.\w+',   # OS module access
        r'subprocess\.\w+',  # Subprocess access
    ]

    @staticmethod
    def sanitize_input(text: str) -> str:
        """
        Sanitize user input by escaping HTML and removing dangerous patterns
        """
        if not text:
            return text

        # HTML escape to prevent XSS
        sanitized = html.escape(text)

        # Remove dangerous patterns
        for pattern in QueryValidation.DANGEROUS_PATTERNS:
            if re.search(pattern, sanitized, re.IGNORECASE):
                logger.warning(f"Dangerous pattern detected and removed: {pattern}")
                # Remove the dangerous pattern but keep the rest of the text
                sanitized = re.sub(pattern, '', sanitized, flags=re.IGNORECASE)

        return sanitized

    @staticmethod
    def validate_query_length(text: str, max_length: int = 10000) -> str:
        """
        Validate query length
        """
        if len(text) > max_length:
            raise ValueError(f"Query exceeds maximum length of {max_length} characters")
        return text

    @staticmethod
    def validate_content_type(content_type: str) -> bool:
        """
        Validate content type against allowed values
        """
        allowed_types = {'text', 'code', 'diagram_description', 'mathematical_formula'}
        return content_type in allowed_types

    @staticmethod
    def validate_user_type(user_type: str) -> bool:
        """
        Validate user type against allowed values
        """
        allowed_types = {'student', 'educator', 'admin'}
        return user_type in allowed_types

    @staticmethod
    def validate_message_type(message_type: str) -> bool:
        """
        Validate message type against allowed values
        """
        allowed_types = {'query', 'response', 'system_message', 'follow-up', 'text-selection'}
        return message_type in allowed_types

    @staticmethod
    def validate_record_type(record_type: str) -> bool:
        """
        Validate analytics record type against allowed values
        """
        allowed_types = {'user_interaction', 'common_question', 'system_usage', 'performance_metric'}
        return record_type in allowed_types

    @staticmethod
    def validate_aggregation_period(period: str) -> bool:
        """
        Validate aggregation period against allowed values
        """
        allowed_periods = {'hourly', 'daily', 'weekly', 'monthly'}
        return period in allowed_periods

    @staticmethod
    def validate_confidence_score(score: float) -> bool:
        """
        Validate confidence score is between 0 and 1
        """
        return 0 <= score <= 1

    @staticmethod
    def validate_url_path(path: str) -> bool:
        """
        Validate URL path is safe (no directory traversal)
        """
        if '../' in path or '..\\' in path:
            return False
        return True

    @staticmethod
    def is_safe_content(text: str) -> tuple[bool, Optional[str]]:
        """
        Check if content is safe for processing
        Returns (is_safe, reason_if_unsafe)
        """
        # Check for dangerous patterns
        for pattern in QueryValidation.DANGEROUS_PATTERNS:
            if re.search(pattern, text, re.IGNORECASE):
                return False, f"Dangerous pattern detected: {pattern}"

        # Check for excessive length
        if len(text) > 10000:  # 10k characters max
            return False, "Content exceeds maximum length"

        # Check for potential prompt injection attempts
        injection_patterns = [
            r'ignore\s+previous',
            r'forget\s+previous',
            r'system\s+prompt',
            r'you\s+are\s+now',
            r'disregard\s+instructions'
        ]

        for pattern in injection_patterns:
            if re.search(pattern, text, re.IGNORECASE):
                return False, f"Potential prompt injection detected: {pattern}"

        return True, None


def validate_and_sanitize_query(query: str) -> str:
    """
    Main function to validate and sanitize a user query
    """
    if not query or not isinstance(query, str):
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Query must be a non-empty string"
        )

    # Validate length first
    try:
        query = QueryValidation.validate_query_length(query)
    except ValueError as e:
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail=str(e)
        )

    # Check if content is safe
    is_safe, reason = QueryValidation.is_safe_content(query)
    if not is_safe:
        logger.warning(f"Unsafe content detected: {reason}")
        raise HTTPException(
            status_code=status.HTTP_422_UNPROCESSABLE_ENTITY,
            detail="Content validation failed"
        )

    # Sanitize the input
    sanitized_query = QueryValidation.sanitize_input(query)

    return sanitized_query


class ConversationCreateRequest(BaseModel):
    """
    Pydantic model for conversation creation request validation
    """
    user_id: Optional[str] = None
    session_id: Optional[str] = None
    initial_query: Optional[str] = None

    @validator('user_id')
    def validate_user_id(cls, v):
        if v is not None and len(v) > 100:
            raise ValueError('user_id must be 100 characters or less')
        return v

    @validator('session_id')
    def validate_session_id(cls, v):
        if v is not None and len(v) > 100:
            raise ValueError('session_id must be 100 characters or less')
        return v

    @validator('initial_query')
    def validate_initial_query(cls, v):
        if v is not None:
            if len(v) > 10000:
                raise ValueError('initial_query exceeds maximum length of 10000 characters')
            # Additional validation for safety
            is_safe, reason = QueryValidation.is_safe_content(v)
            if not is_safe:
                raise ValueError(f'initial_query contains unsafe content: {reason}')
        return v


class MessageCreateRequest(BaseModel):
    """
    Pydantic model for message creation request validation
    """
    content: str
    message_type: str = 'query'

    @validator('content')
    def validate_content(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('content cannot be empty')
        if len(v) > 10000:
            raise ValueError('content exceeds maximum length of 10000 characters')
        # Additional validation for safety
        is_safe, reason = QueryValidation.is_safe_content(v)
        if not is_safe:
            raise ValueError(f'content contains unsafe content: {reason}')
        return v

    @validator('message_type')
    def validate_message_type(cls, v):
        if not QueryValidation.validate_message_type(v):
            raise ValueError(f'invalid message_type: {v}')
        return v


class SearchRequest(BaseModel):
    """
    Pydantic model for search request validation
    """
    query: str
    max_results: int = 5
    include_citations: bool = True

    @validator('query')
    def validate_search_query(cls, v):
        if not v or len(v.strip()) == 0:
            raise ValueError('query cannot be empty')
        if len(v) > 1000:
            raise ValueError('query exceeds maximum length of 1000 characters')
        # Additional validation for safety
        is_safe, reason = QueryValidation.is_safe_content(v)
        if not is_safe:
            raise ValueError(f'query contains unsafe content: {reason}')
        return v

    @validator('max_results')
    def validate_max_results(cls, v):
        if v < 1 or v > 20:
            raise ValueError('max_results must be between 1 and 20')
        return v


class FeedbackRequest(BaseModel):
    """
    Pydantic model for feedback request validation
    """
    message_id: str
    conversation_id: str
    feedback_type: str
    comment: Optional[str] = None

    @validator('feedback_type')
    def validate_feedback_type(cls, v):
        allowed_types = {'positive', 'negative', 'report_inaccurate', 'report_inappropriate'}
        if v not in allowed_types:
            raise ValueError(f'feedback_type must be one of {allowed_types}')
        return v

    @validator('comment')
    def validate_comment(cls, v):
        if v and len(v) > 1000:
            raise ValueError('comment exceeds maximum length of 1000 characters')
        return v