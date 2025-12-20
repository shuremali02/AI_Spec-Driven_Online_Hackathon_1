"""
Rate limiting utility for the Personalization API.

Implements an in-memory sliding window rate limiter.
"""

import time
from dataclasses import dataclass, field
from typing import Dict, Optional, Tuple
from config import Config


@dataclass
class RateLimitEntry:
    """
    Entry in the rate limit store tracking request count and reset time.
    """
    count: int = 0
    reset_time: float = 0.0


class RateLimiter:
    """
    In-memory rate limiter using sliding window algorithm.

    Tracks requests per user and enforces rate limits.
    """

    def __init__(
        self,
        max_requests: Optional[int] = None,
        window_seconds: Optional[int] = None
    ):
        """
        Initialize rate limiter.

        Args:
            max_requests: Maximum requests per window (default from config)
            window_seconds: Window duration in seconds (default from config)
        """
        self.max_requests = max_requests or Config.PERSONALIZE_RATE_LIMIT
        self.window_seconds = window_seconds or Config.PERSONALIZE_RATE_LIMIT_WINDOW
        self._store: Dict[str, RateLimitEntry] = {}

    def check(self, user_id: str) -> Tuple[bool, Optional[int]]:
        """
        Check if a request is allowed for the given user.

        Args:
            user_id: Unique identifier for the user

        Returns:
            Tuple of (is_allowed, retry_after_seconds)
            - If allowed: (True, None)
            - If rate limited: (False, seconds_until_reset)
        """
        now = time.time()
        entry = self._store.get(user_id)

        # No entry or window expired - create new window
        if not entry or now > entry.reset_time:
            self._store[user_id] = RateLimitEntry(
                count=1,
                reset_time=now + self.window_seconds
            )
            return (True, None)

        # Check if over limit
        if entry.count >= self.max_requests:
            retry_after = int(entry.reset_time - now) + 1  # Add 1 to ensure we're past reset
            return (False, retry_after)

        # Increment count
        entry.count += 1
        return (True, None)

    def get_remaining(self, user_id: str) -> int:
        """
        Get remaining requests for the user in current window.

        Args:
            user_id: Unique identifier for the user

        Returns:
            Number of remaining requests
        """
        now = time.time()
        entry = self._store.get(user_id)

        if not entry or now > entry.reset_time:
            return self.max_requests

        return max(0, self.max_requests - entry.count)

    def reset(self, user_id: str) -> None:
        """
        Reset rate limit for a user (useful for testing).

        Args:
            user_id: Unique identifier for the user
        """
        if user_id in self._store:
            del self._store[user_id]

    def clear_expired(self) -> int:
        """
        Clear all expired entries from the store.

        Returns:
            Number of entries cleared
        """
        now = time.time()
        expired_keys = [
            key for key, entry in self._store.items()
            if now > entry.reset_time
        ]
        for key in expired_keys:
            del self._store[key]
        return len(expired_keys)


# Global rate limiter instance for personalization endpoint
_personalize_rate_limiter: Optional[RateLimiter] = None


def get_personalize_rate_limiter() -> RateLimiter:
    """
    Get or create the global rate limiter instance.

    Returns:
        RateLimiter instance
    """
    global _personalize_rate_limiter
    if _personalize_rate_limiter is None:
        _personalize_rate_limiter = RateLimiter()
    return _personalize_rate_limiter
