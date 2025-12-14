from sqlalchemy import Column, Integer, String, DateTime, Boolean, Text, DECIMAL, ForeignKey, JSON
from sqlalchemy.dialects.postgresql import UUID
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.sql import func
import uuid

Base = declarative_base()

class User(Base):
    __tablename__ = "users"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_type = Column(String(20), nullable=False)  # 'student', 'educator', 'admin'
    created_at = Column(DateTime(timezone=True), default=func.current_timestamp())
    last_active_at = Column(DateTime(timezone=True))
    rate_limit_reset_at = Column(DateTime(timezone=True))

    __mapper_args__ = {
        'confirm_deleted_rows': False
    }


class Conversation(Base):
    __tablename__ = "conversations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id", ondelete="SET NULL"), nullable=True)  # Allow anonymous users
    session_id = Column(String(100), nullable=True)  # For anonymous session tracking
    created_at = Column(DateTime(timezone=True), default=func.current_timestamp())
    updated_at = Column(DateTime(timezone=True), default=func.current_timestamp(), onupdate=func.current_timestamp())
    title = Column(String(100))
    is_active = Column(Boolean, default=True)


class Message(Base):
    __tablename__ = "messages"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    conversation_id = Column(UUID(as_uuid=True), ForeignKey("conversations.id", ondelete="CASCADE"), nullable=False)
    sender_type = Column(String(10), nullable=False)  # 'user', 'system'
    content = Column(Text, nullable=False)
    created_at = Column(DateTime(timezone=True), default=func.current_timestamp())
    citations = Column(JSON)  # JSONB equivalent
    message_type = Column(String(20), default='query')  # 'query', 'response', 'system_message'


class Citation(Base):
    __tablename__ = "citations"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    message_id = Column(UUID(as_uuid=True), ForeignKey("messages.id", ondelete="CASCADE"), nullable=False)
    textbook_content_id = Column(UUID(as_uuid=True), nullable=False)  # Not a foreign key since content is in vector DB
    chapter_title = Column(String(200), nullable=False)
    section_title = Column(String(200), nullable=False)
    url_path = Column(String(500), nullable=False)
    confidence_score = Column(DECIMAL(3, 2))  # DECIMAL(3,2) allows values like 0.95


class TextbookContent(Base):
    __tablename__ = "textbook_content"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    chapter_id = Column(String(100), nullable=False)
    section_path = Column(String(200), nullable=False)
    content_type = Column(String(50), nullable=False)  # 'text', 'code', 'diagram_description', 'mathematical_formula'
    metadata_content = Column(JSON)  # JSONB equivalent
    embedding_id = Column(String(100), nullable=False)
    token_count = Column(Integer)


class AnalyticsRecord(Base):
    __tablename__ = "analytics_records"

    id = Column(UUID(as_uuid=True), primary_key=True, default=uuid.uuid4)
    record_type = Column(String(30), nullable=False)  # 'user_interaction', 'common_question', 'system_usage', 'performance_metric'
    user_id = Column(UUID(as_uuid=True), ForeignKey("users.id"))
    conversation_id = Column(UUID(as_uuid=True), ForeignKey("conversations.id"))
    data_payload = Column(JSON, nullable=False)  # JSONB equivalent
    created_at = Column(DateTime(timezone=True), default=func.current_timestamp())
    aggregation_period = Column(String(10))  # 'hourly', 'daily', 'weekly', 'monthly'