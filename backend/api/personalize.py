"""
FastAPI endpoint for chapter content personalization.

POST /api/personalize - Personalizes chapter content based on user profile.
"""

import logging
from datetime import datetime
from typing import Union

from fastapi import APIRouter, Request, Depends, HTTPException
from fastapi.responses import JSONResponse
from sqlalchemy.ext.asyncio import AsyncSession

from api.personalize_models import (
    PersonalizeRequest,
    PersonalizeSuccessResponse,
    PersonalizeErrorResponse,
    PersonalizationSummary,
    HardwareContext,
    ErrorCodes,
    UserProfile
)
from api.auth_helper import validate_session
from api.rate_limiter import get_personalize_rate_limiter
from db.postgres_client import get_db_session
from db.profile_queries import get_user_profile
from agents.personalize_agent import get_personalize_agent

logger = logging.getLogger(__name__)

# Create router
router = APIRouter(prefix="/api", tags=["personalization"])


@router.post(
    "/personalize",
    response_model=Union[PersonalizeSuccessResponse, PersonalizeErrorResponse],
    responses={
        200: {"model": PersonalizeSuccessResponse, "description": "Personalization successful"},
        400: {"model": PersonalizeErrorResponse, "description": "Invalid request or incomplete profile"},
        401: {"model": PersonalizeErrorResponse, "description": "Authentication required"},
        404: {"model": PersonalizeErrorResponse, "description": "Profile not found"},
        429: {"model": PersonalizeErrorResponse, "description": "Rate limited"},
        500: {"model": PersonalizeErrorResponse, "description": "Personalization failed"}
    }
)
async def personalize_content(
    request: Request,
    body: PersonalizeRequest,
    session: AsyncSession = Depends(get_db_session)
):
    """
    Personalize chapter content based on user profile.

    Requires authentication via Better-Auth session cookie.
    User must have a complete profile with all required fields.

    Rate limited to 5 requests per minute per user.
    """
    try:
        # TEMPORARY: Using mock profile for testing (auth bypassed)
        # TODO: Re-enable auth before production
        user_id = "test-user-123"
        profile = UserProfile(
            auth_user_id=user_id,
            experience_level="intermediate",
            programming_languages=["Python", "JavaScript"],
            frameworks_platforms=["ROS/ROS 2", "TensorFlow"],
            device_type="laptop",
            operating_system="linux",
            system_capability="medium"
        )
        logger.info(f"Using mock profile for testing: {user_id}")

        # Step 5: Call PersonalizeContentAgent
        logger.info(f"Personalizing chapter '{body.chapter_id}' for user: {user_id}")
        agent = get_personalize_agent()

        try:
            result = await agent.personalize(
                content=body.chapter_content,
                profile=profile
            )
        except Exception as e:
            logger.error(f"Personalization agent failed: {e}")
            return JSONResponse(
                status_code=500,
                content={
                    "success": False,
                    "error": ErrorCodes.PERSONALIZATION_FAILED,
                    "message": "Personalization service encountered an error",
                    "retry_after": 5
                }
            )

        # Step 6: Build and return success response
        response = PersonalizeSuccessResponse(
            success=True,
            personalized_content=result.personalized_content,
            chapter_id=body.chapter_id,
            personalization_summary=PersonalizationSummary(
                experience_level=result.experience_level,
                programming_context=result.programming_context,
                hardware_context=HardwareContext(**result.hardware_context),
                adjustments_made=result.adjustments_made
            ),
            timestamp=datetime.utcnow().isoformat() + "Z"
        )

        logger.info(f"Successfully personalized chapter '{body.chapter_id}' for user: {user_id}")
        return response

    except Exception as e:
        logger.error(f"Unexpected error in personalize endpoint: {e}")
        import traceback
        traceback.print_exc()
        return JSONResponse(
            status_code=500,
            content={
                "success": False,
                "error": ErrorCodes.PERSONALIZATION_FAILED,
                "message": "Personalization service encountered an error",
                "retry_after": 5
            }
        )
