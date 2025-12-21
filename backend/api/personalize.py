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
        # ==========================================================
        # LOCAL TESTING MODE - Set to False before production deploy
        # ==========================================================
        # Step 1: Get user ID
        # Priority: 1) Request body user_id, 2) Session cookie validation
        user_id = body.user_id

        if user_id:
            logger.info(f"Using user_id from request body: {user_id}")
        else:
            # Fallback to session cookie validation
            user_id = await validate_session(request)
            if not user_id:
                logger.warning("Personalization request without valid session or user_id")
                return JSONResponse(
                    status_code=401,
                    content={
                        "success": False,
                        "error": ErrorCodes.AUTH_REQUIRED,
                        "message": "Authentication required. Please log in to personalize content."
                    }
                )

        # Step 2: Get user profile from database
        profile = await get_user_profile(user_id, session)
        if not profile:
            logger.warning(f"No profile found for user: {user_id}")
            return JSONResponse(
                status_code=404,
                content={
                    "success": False,
                    "error": ErrorCodes.PROFILE_NOT_FOUND,
                    "message": "User profile not found. Please complete your profile setup."
                }
            )

        # Step 3: Check if profile is complete
        is_complete, missing_fields = profile.is_complete()
        if not is_complete:
            logger.warning(f"Incomplete profile for user {user_id}: missing {missing_fields}")
            return JSONResponse(
                status_code=400,
                content={
                    "success": False,
                    "error": ErrorCodes.PROFILE_INCOMPLETE,
                    "message": "Profile incomplete. Please fill in all required fields.",
                    "missing_fields": missing_fields
                }
            )

        # Step 4: Apply rate limiting
        rate_limiter = get_personalize_rate_limiter()
        is_allowed, retry_after = rate_limiter.check(user_id)
        if not is_allowed:
            logger.warning(f"Rate limited user: {user_id}")
            return JSONResponse(
                status_code=429,
                content={
                    "success": False,
                    "error": ErrorCodes.RATE_LIMITED,
                    "message": "Too many requests. Please wait before trying again.",
                    "retry_after": retry_after or 60
                }
            )

        logger.info(f"Personalizing for user {user_id}: level={profile.experience_level}, "
                    f"languages={profile.programming_languages}, capability={profile.system_capability}")

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


# =============================================================================
# DEBUG ENDPOINTS - Commented out for production
# =============================================================================
# @router.get("/debug/profile")
# async def debug_profile(session: AsyncSession = Depends(get_db_session)):
#     """
#     DEBUG: View all profiles in database.
#     TODO: Remove this endpoint before production deploy!
#     """
#     from sqlalchemy import text
#     result = await session.execute(text("""
#         SELECT auth_user_id, experience_level, programming_languages,
#                frameworks_platforms, device_type, operating_system, system_capability
#         FROM user_profiles
#     """))
#     rows = result.fetchall()
#
#     profiles = []
#     for row in rows:
#         profiles.append({
#             "auth_user_id": row[0],
#             "experience_level": row[1],
#             "programming_languages": row[2],
#             "frameworks_platforms": row[3],
#             "device_type": row[4],
#             "operating_system": row[5],
#             "system_capability": row[6]
#         })
#
#     return {"profiles": profiles, "count": len(profiles)}


# @router.get("/debug/update/{user_id}/{experience_level}")
# async def debug_update_profile(
#     user_id: str,
#     experience_level: str,
#     session: AsyncSession = Depends(get_db_session)
# ):
#     """
#     DEBUG: Update a user's experience level.
#     TODO: Remove this endpoint before production deploy!
#
#     Usage: PUT /api/debug/profile/PK6EryRXKtWazlmRifLU9OD7ZMSSTXT9?experience_level=beginner
#     """
#     from sqlalchemy import text
#
#     valid_levels = ["beginner", "intermediate", "advanced", "expert"]
#     if experience_level not in valid_levels:
#         return {"error": f"Invalid level. Must be one of: {valid_levels}"}
#
#     await session.execute(
#         text("UPDATE user_profiles SET experience_level = :level WHERE auth_user_id = :user_id"),
#         {"level": experience_level, "user_id": user_id}
#     )
#     await session.commit()
#
#     return {"success": True, "user_id": user_id, "new_experience_level": experience_level}
