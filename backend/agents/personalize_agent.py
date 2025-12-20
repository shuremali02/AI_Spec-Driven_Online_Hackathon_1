"""
PersonalizeContentAgent - Adapts chapter content based on user profile.

This agent follows the same pattern as the RAG agent, using AsyncOpenAI
client with Gemini's OpenAI-compatible endpoint.
"""

from openai import AsyncOpenAI
import os
from typing import List, Optional
import logging
from pydantic import BaseModel
from datetime import datetime

from api.personalize_models import UserProfile
from config import Config

logger = logging.getLogger(__name__)


# =============================================================================
# System Prompt for Personalization
# =============================================================================

PERSONALIZATION_SYSTEM_PROMPT = """You are a content personalization assistant for a Physical AI & Humanoid Robotics textbook.

Your task is to adapt chapter content based on the user's background while following these rules:

MUST DO:
- Adjust explanation depth based on experience level
- Use wording appropriate for the experience level
- Add relevant context based on known programming languages
- Include hardware-aware notes based on system capability
- Preserve the exact document structure (headings, sections, lists)

MUST NOT:
- Change any code blocks (preserve exactly as-is, including all ``` fenced blocks)
- Modify command-line examples
- Alter technical term definitions
- Introduce new topics not in the original
- Change factual meaning of any content
- Modify URLs or links
- Alter Mermaid diagram code (anything inside ```mermaid blocks)
- Add or remove headings

EXPERIENCE LEVEL ADAPTATIONS:
- beginner: Simpler language, more context, step-by-step guidance, explain acronyms
- intermediate: Balanced explanations, assume basic knowledge, practical focus
- advanced: Concise technical language, skip basics, focus on advanced concepts
- expert: Direct technical prose, assume deep domain knowledge, reference edge cases

PROGRAMMING LANGUAGE ADAPTATIONS:
- If user knows Python: Reference Python idioms and patterns where applicable
- If user knows C/C++: Add memory/performance considerations where relevant
- If user knows JavaScript: Draw parallels to async/event-driven patterns
- If user knows Java: Reference OOP concepts familiar to Java developers

HARDWARE ADAPTATIONS:
- low capability: Add warnings about resource-heavy operations, suggest lightweight alternatives
- medium capability: Balance between features and performance
- high capability: Mention GPU acceleration and parallel processing options
- embedded device type: Emphasize embedded-friendly approaches, mention Jetson/RPi specifics

OUTPUT FORMAT:
Return ONLY the personalized markdown content. Do not include explanations, meta-commentary, or wrap in code blocks."""


# =============================================================================
# Response Models
# =============================================================================

class PersonalizationResult(BaseModel):
    """
    Result from the personalization agent.
    """
    personalized_content: str
    experience_level: str
    programming_context: List[str]
    hardware_context: dict
    adjustments_made: List[str]


# =============================================================================
# PersonalizeContentAgent Class
# =============================================================================

class PersonalizeContentAgent:
    """
    Agent that personalizes textbook content based on user profile.

    Uses AsyncOpenAI client with Gemini's OpenAI-compatible endpoint,
    following the same pattern as the RAG agent.
    """

    def __init__(self):
        """
        Initialize the PersonalizeContentAgent with AsyncOpenAI client.
        """
        # Match RAG agent pattern exactly:
        base_url = os.getenv(
            "OPENAI_BASE_URL",
            "https://generativelanguage.googleapis.com/v1beta/openai"
        )
        # Remove trailing slash if present
        base_url = base_url.rstrip('/')

        api_key = os.getenv("OPENAI_API_KEY")

        if not api_key:
            raise ValueError("OPENAI_API_KEY environment variable is required")

        self.client = AsyncOpenAI(
            base_url=base_url,
            api_key=api_key
        )

        self.model = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")
        self.max_tokens = Config.PERSONALIZE_MAX_TOKENS
        self.temperature = 0.3  # Lower temperature for consistent, factual output

    def _build_user_prompt(self, content: str, profile: UserProfile) -> str:
        """
        Build the user prompt with profile context.

        Args:
            content: Original chapter content
            profile: User profile data

        Returns:
            Formatted user prompt string
        """
        languages_joined = ", ".join(profile.programming_languages) if profile.programming_languages else "None specified"
        frameworks_joined = ", ".join(profile.frameworks_platforms) if profile.frameworks_platforms else "None specified"

        return f"""Personalize the following chapter content for a user with this background:

**Experience Level**: {profile.experience_level}
**Programming Languages**: {languages_joined}
**Frameworks/Platforms**: {frameworks_joined}
**Device Type**: {profile.device_type}
**Operating System**: {profile.operating_system}
**System Capability**: {profile.system_capability}

---

CHAPTER CONTENT:

{content}"""

    def _build_adjustments_summary(self, profile: UserProfile) -> List[str]:
        """
        Build a summary of adjustments made based on profile.

        Args:
            profile: User profile data

        Returns:
            List of adjustment descriptions
        """
        adjustments = []

        # Experience level adjustment
        adjustments.append(f"Adapted explanations for {profile.experience_level} level")

        # Programming language adjustments
        if "Python" in profile.programming_languages:
            adjustments.append("Added Python-specific context")
        if "C" in profile.programming_languages or "C++" in profile.programming_languages:
            adjustments.append("Added memory/performance considerations")
        if "JavaScript" in profile.programming_languages:
            adjustments.append("Added async/event-driven parallels")

        # Framework adjustments
        if "ROS/ROS 2" in profile.frameworks_platforms or "ROS 2" in profile.frameworks_platforms:
            adjustments.append("Included ROS 2 familiarity assumptions")

        # Hardware adjustments
        if profile.system_capability == "low":
            adjustments.append("Added resource-aware warnings")
        if profile.system_capability == "high":
            adjustments.append("Mentioned GPU acceleration options")
        if profile.device_type == "embedded":
            adjustments.append("Emphasized embedded-friendly approaches")

        # OS adjustments
        if profile.operating_system != "linux":
            adjustments.append(f"Added {profile.operating_system}-specific notes")

        return adjustments

    async def personalize(
        self,
        content: str,
        profile: UserProfile
    ) -> PersonalizationResult:
        """
        Personalize chapter content based on user profile.

        Args:
            content: Original chapter content (markdown)
            profile: User profile with background data

        Returns:
            PersonalizationResult with personalized content and summary

        Raises:
            Exception: If LLM call fails
        """
        logger.info(f"Personalizing content for user: {profile.auth_user_id}")
        logger.debug(f"Profile: experience={profile.experience_level}, "
                     f"languages={profile.programming_languages}")

        # Build prompts
        user_prompt = self._build_user_prompt(content, profile)

        try:
            # Call the LLM (following RAG agent pattern)
            response = await self.client.chat.completions.create(
                model=self.model,
                messages=[
                    {"role": "system", "content": PERSONALIZATION_SYSTEM_PROMPT},
                    {"role": "user", "content": user_prompt}
                ],
                max_tokens=self.max_tokens,
                temperature=self.temperature
            )

            personalized_content = response.choices[0].message.content

            if not personalized_content:
                raise ValueError("Empty response from LLM")

            # Build result
            adjustments = self._build_adjustments_summary(profile)

            return PersonalizationResult(
                personalized_content=personalized_content,
                experience_level=profile.experience_level,
                programming_context=profile.programming_languages,
                hardware_context={
                    "system_capability": profile.system_capability,
                    "operating_system": profile.operating_system
                },
                adjustments_made=adjustments
            )

        except Exception as e:
            logger.error(f"Error personalizing content: {e}")
            import traceback
            traceback.print_exc()
            raise


# =============================================================================
# Lazy-initialized Agent Instance
# =============================================================================

_personalize_agent: Optional[PersonalizeContentAgent] = None


def get_personalize_agent() -> PersonalizeContentAgent:
    """
    Get or create the PersonalizeContentAgent instance (lazy initialization).

    Returns:
        PersonalizeContentAgent instance
    """
    global _personalize_agent
    if _personalize_agent is None:
        _personalize_agent = PersonalizeContentAgent()
    return _personalize_agent
