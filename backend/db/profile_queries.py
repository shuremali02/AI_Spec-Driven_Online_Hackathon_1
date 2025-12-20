"""
Database queries for user profiles.

Provides functions for querying user profile data from the Neon PostgreSQL database.
"""

import logging
from typing import Optional
from sqlalchemy import text
from sqlalchemy.ext.asyncio import AsyncSession

from api.personalize_models import UserProfile
from db.postgres_client import get_db_session

logger = logging.getLogger(__name__)


async def get_user_profile(auth_user_id: str, session: AsyncSession) -> Optional[UserProfile]:
    """
    Query user profile from the database by auth_user_id.

    Args:
        auth_user_id: The Better-Auth user ID
        session: SQLAlchemy async session

    Returns:
        UserProfile if found, None otherwise
    """
    try:
        query = text("""
            SELECT
                auth_user_id,
                experience_level,
                programming_languages,
                frameworks_platforms,
                device_type,
                operating_system,
                system_capability
            FROM user_profiles
            WHERE auth_user_id = :auth_user_id
            LIMIT 1
        """)

        result = await session.execute(query, {"auth_user_id": auth_user_id})
        row = result.fetchone()

        if not row:
            logger.debug(f"No profile found for user: {auth_user_id}")
            return None

        return UserProfile(
            auth_user_id=row.auth_user_id,
            experience_level=row.experience_level or "",
            programming_languages=row.programming_languages or [],
            frameworks_platforms=row.frameworks_platforms or [],
            device_type=row.device_type or "",
            operating_system=row.operating_system or "",
            system_capability=row.system_capability or ""
        )

    except Exception as e:
        logger.error(f"Error querying user profile: {e}")
        return None


async def profile_exists(auth_user_id: str, session: AsyncSession) -> bool:
    """
    Check if a profile exists for the given user.

    Args:
        auth_user_id: The Better-Auth user ID
        session: SQLAlchemy async session

    Returns:
        True if profile exists, False otherwise
    """
    try:
        query = text("""
            SELECT 1 FROM user_profiles
            WHERE auth_user_id = :auth_user_id
            LIMIT 1
        """)

        result = await session.execute(query, {"auth_user_id": auth_user_id})
        return result.fetchone() is not None

    except Exception as e:
        logger.error(f"Error checking profile existence: {e}")
        return False
