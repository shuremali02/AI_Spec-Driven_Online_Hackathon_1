"""
Authentication helper for validating Better-Auth sessions.

Provides utilities for session validation and user extraction.
"""

import httpx
import logging
from typing import Optional, Dict, Any
from fastapi import Request
from config import Config

logger = logging.getLogger(__name__)


async def validate_session(request: Request) -> Optional[str]:
    """
    Extract and validate session from request cookies.

    Calls the Better-Auth backend to validate the session and returns
    the user ID if valid.

    Args:
        request: FastAPI Request object

    Returns:
        User ID string if session is valid, None otherwise
    """
    # Extract session cookie
    session_token = request.cookies.get("better-auth.session_token")

    if not session_token:
        logger.debug("No session cookie found")
        return None

    try:
        # Call auth backend to validate session
        async with httpx.AsyncClient(timeout=5.0) as client:
            response = await client.get(
                f"{Config.AUTH_BACKEND_URL}/api/auth/get-session",
                cookies={"better-auth.session_token": session_token}
            )

            if response.status_code != 200:
                logger.debug(f"Session validation failed: {response.status_code}")
                return None

            data = response.json()
            user_id = data.get("user", {}).get("id")

            if not user_id:
                logger.debug("No user ID in session response")
                return None

            return user_id

    except httpx.TimeoutException:
        logger.warning("Session validation timed out")
        return None
    except httpx.RequestError as e:
        logger.error(f"Session validation request failed: {e}")
        return None
    except Exception as e:
        logger.error(f"Unexpected error validating session: {e}")
        return None


async def get_session_data(request: Request) -> Optional[Dict[str, Any]]:
    """
    Get full session data from the auth backend.

    Args:
        request: FastAPI Request object

    Returns:
        Session data dictionary if valid, None otherwise
    """
    session_token = request.cookies.get("better-auth.session_token")

    if not session_token:
        return None

    try:
        async with httpx.AsyncClient(timeout=5.0) as client:
            response = await client.get(
                f"{Config.AUTH_BACKEND_URL}/api/auth/get-session",
                cookies={"better-auth.session_token": session_token}
            )

            if response.status_code != 200:
                return None

            return response.json()

    except Exception as e:
        logger.error(f"Error getting session data: {e}")
        return None


def extract_user_id_from_cookies(request: Request) -> Optional[str]:
    """
    Quick check for session cookie presence without validation.

    Useful for early bailout when no cookie exists.

    Args:
        request: FastAPI Request object

    Returns:
        Session token if present, None otherwise
    """
    return request.cookies.get("better-auth.session_token")
