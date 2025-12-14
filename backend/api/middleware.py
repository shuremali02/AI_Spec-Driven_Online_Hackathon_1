from fastapi import Request, HTTPException, status
from fastapi.responses import JSONResponse
from typing import Dict, Optional
from datetime import datetime, timedelta
import time
import logging

logger = logging.getLogger(__name__)

class RateLimiter:
    def __init__(self, requests_limit: int = 100, window_size: int = 60):
        """
        Initialize rate limiter
        :param requests_limit: Number of requests allowed per window
        :param window_size: Time window in seconds
        """
        self.requests_limit = requests_limit
        self.window_size = window_size
        self.requests: Dict[str, list] = {}  # user_id -> list of request timestamps

    def is_allowed(self, user_id: str) -> tuple[bool, Optional[int]]:
        """
        Check if request is allowed for user
        :param user_id: User identifier
        :return: (is_allowed, seconds_to_wait)
        """
        now = time.time()
        window_start = now - self.window_size

        if user_id not in self.requests:
            self.requests[user_id] = []

        # Remove old requests outside the window
        self.requests[user_id] = [req_time for req_time in self.requests[user_id] if req_time > window_start]

        # Check if limit is exceeded
        if len(self.requests[user_id]) >= self.requests_limit:
            # Calculate wait time
            oldest_request = min(self.requests[user_id])
            wait_time = int(oldest_request + self.window_size - now)
            return False, max(1, wait_time)

        # Add current request
        self.requests[user_id].append(now)
        return True, None

# Global rate limiter instance
rate_limiter = RateLimiter(
    requests_limit=int(__import__('os').environ.get('RATE_LIMIT_REQUESTS', '100')),
    window_size=int(__import__('os').environ.get('RATE_LIMIT_WINDOW', '60'))
)

async def rate_limit_middleware(request: Request, call_next):
    """
    Rate limiting middleware
    """
    # Extract user ID from headers or session
    user_id = request.headers.get('x-user-id') or 'anonymous'

    # If API key is available, use it as identifier
    auth_header = request.headers.get('authorization', '')
    if auth_header.startswith('Bearer '):
        user_id = auth_header.split(' ')[1][:12]  # Use first 12 chars of API key as identifier

    is_allowed, wait_time = rate_limiter.is_allowed(user_id)

    if not is_allowed:
        return JSONResponse(
            status_code=status.HTTP_429_TOO_MANY_REQUESTS,
            content={
                "error": {
                    "code": "RATE_LIMIT_EXCEEDED",
                    "message": f"Rate limit exceeded. Please try again in {wait_time} seconds.",
                    "details": {
                        "user_id": user_id,
                        "requests_limit": rate_limiter.requests_limit,
                        "window_size": rate_limiter.window_size,
                        "wait_time_seconds": wait_time
                    },
                    "timestamp": datetime.utcnow().isoformat()
                }
            }
        )

    # Add rate limit headers to response
    response = await call_next(request)

    # Calculate remaining requests
    now = time.time()
    window_start = now - rate_limiter.window_size
    if user_id in rate_limiter.requests:
        current_requests = [req_time for req_time in rate_limiter.requests[user_id] if req_time > window_start]
        remaining = rate_limiter.requests_limit - len(current_requests)
    else:
        remaining = rate_limiter.requests_limit

    response.headers["X-RateLimit-Remaining"] = str(remaining)
    response.headers["X-RateLimit-Limit"] = str(rate_limiter.requests_limit)
    response.headers["X-RateLimit-Reset"] = str(int(now + rate_limiter.window_size))

    return response

def get_client_ip(request: Request) -> str:
    """
    Get client IP address from request
    """
    # Check for forwarded headers first (for reverse proxies)
    forwarded_for = request.headers.get("x-forwarded-for")
    if forwarded_for:
        return forwarded_for.split(",")[0].strip()

    real_ip = request.headers.get("x-real-ip")
    if real_ip:
        return real_ip.strip()

    # Fallback to direct client address
    return request.client.host if request.client else "unknown"