"""
Pydantic models for the Personalization API endpoint.

These models define the request/response structure for the POST /api/personalize endpoint.
"""

from pydantic import BaseModel, Field
from typing import Optional, List
from datetime import datetime


class PersonalizeRequest(BaseModel):
    """
    Request body for POST /api/personalize endpoint.
    """
    chapter_id: str = Field(
        ...,
        description="Unique identifier for the chapter",
        min_length=1
    )
    chapter_content: str = Field(
        ...,
        description="Full markdown content to personalize",
        min_length=1,
        max_length=50000
    )


class HardwareContext(BaseModel):
    """
    Hardware context included in personalization summary.
    """
    system_capability: str
    operating_system: str


class PersonalizationSummary(BaseModel):
    """
    Summary of personalization adjustments made.
    """
    experience_level: str
    programming_context: List[str]
    hardware_context: HardwareContext
    adjustments_made: List[str]


class PersonalizeSuccessResponse(BaseModel):
    """
    Success response from POST /api/personalize endpoint.
    """
    success: bool = True
    personalized_content: str
    chapter_id: str
    personalization_summary: PersonalizationSummary
    timestamp: str


class PersonalizeErrorResponse(BaseModel):
    """
    Error response from POST /api/personalize endpoint.
    """
    success: bool = False
    error: str
    message: str
    missing_fields: Optional[List[str]] = None
    retry_after: Optional[int] = None
    field: Optional[str] = None


class UserProfile(BaseModel):
    """
    User profile data used for personalization.
    Maps to the user_profiles table in the database.
    """
    auth_user_id: str
    experience_level: str  # beginner, intermediate, advanced, expert
    programming_languages: List[str]
    frameworks_platforms: List[str]
    device_type: str  # desktop, laptop, tablet, mobile, embedded
    operating_system: str  # windows, macos, linux, other
    system_capability: str  # low, medium, high

    def is_complete(self) -> tuple[bool, List[str]]:
        """
        Check if all required fields are present for personalization.

        Returns:
            Tuple of (is_valid, list_of_missing_fields)
        """
        missing = []

        if not self.experience_level:
            missing.append("experience_level")
        if not self.programming_languages:
            missing.append("programming_languages")
        if not self.frameworks_platforms:
            missing.append("frameworks_platforms")
        if not self.device_type:
            missing.append("device_type")
        if not self.operating_system:
            missing.append("operating_system")
        if not self.system_capability:
            missing.append("system_capability")

        return (len(missing) == 0, missing)


# Error code constants
class ErrorCodes:
    AUTH_REQUIRED = "AUTH_REQUIRED"
    PROFILE_NOT_FOUND = "PROFILE_NOT_FOUND"
    PROFILE_INCOMPLETE = "PROFILE_INCOMPLETE"
    INVALID_REQUEST = "INVALID_REQUEST"
    RATE_LIMITED = "RATE_LIMITED"
    PERSONALIZATION_FAILED = "PERSONALIZATION_FAILED"
