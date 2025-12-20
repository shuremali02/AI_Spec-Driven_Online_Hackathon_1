# Research: Chapter Content Personalization

**Feature**: personalized-content
**Date**: 2025-12-19
**Status**: Complete

---

## Research Questions Resolved

### Q1: How does the existing RAG agent integrate with Gemini?

**Decision**: Use AsyncOpenAI client with Gemini's OpenAI-compatible endpoint

**Rationale**: The existing RAG agent (`backend/agents/rag_agent.py`) establishes the pattern:
- Uses `AsyncOpenAI` from `openai` package
- Base URL: `https://generativelanguage.googleapis.com/v1beta/openai`
- API key from `OPENAI_API_KEY` environment variable
- Model from `GEMINI_MODEL` (default: `gemini-2.0-flash`)
- Trailing slash must be stripped from base URL

**Alternatives Considered**:
- Google's official `google-generativeai` SDK: Rejected (doesn't align with OpenAI Agents SDK requirement)
- Direct HTTP calls to Gemini API: Rejected (more complex, no async support OOTB)

**Code Pattern**:
```python
base_url = os.getenv("OPENAI_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai")
base_url = base_url.rstrip('/')
api_key = os.getenv("OPENAI_API_KEY")

self.client = AsyncOpenAI(
    base_url=base_url,
    api_key=api_key
)
```

---

### Q2: How should session validation work with Better-Auth?

**Decision**: Forward session cookie to auth backend's `/api/auth/get-session` endpoint

**Rationale**:
- Auth backend is the single source of truth for sessions
- Avoids duplicating session validation logic
- Better-Auth handles session expiry automatically
- Consistent with how frontend validates sessions

**Alternatives Considered**:
- Direct database lookup: Rejected (bypasses Better-Auth abstractions, potential security issues)
- JWT verification: Rejected (Better-Auth uses cookie-based sessions, not JWT)

**Implementation**:
```python
async def validate_session(session_cookie: str) -> Optional[dict]:
    async with httpx.AsyncClient() as client:
        response = await client.get(
            f"{AUTH_BACKEND_URL}/api/auth/get-session",
            cookies={"better-auth.session_token": session_cookie}
        )
        if response.status_code == 200:
            return response.json()
    return None
```

---

### Q3: Where should the personalization endpoint be hosted?

**Decision**: Add to existing FastAPI backend (`backend/api/personalize.py`)

**Rationale**:
- Spec mandates FastAPI backend
- Existing backend has database connection to Neon
- Colocated with RAG agent for consistent LLM client configuration
- Simpler deployment (single backend service)

**Alternatives Considered**:
- Keep in auth backend (current TypeScript implementation): Rejected (spec mandates FastAPI + OpenAI Agents SDK)
- New microservice: Rejected (over-engineering for hackathon scope)

---

### Q4: How should rate limiting be implemented?

**Decision**: In-memory rate limiter with sliding window

**Rationale**:
- Simple implementation suitable for hackathon scope
- 5 requests per user per minute (from spec)
- Per-process memory is acceptable for single-instance deployment
- Easy to upgrade to Redis later if needed

**Alternatives Considered**:
- Redis-based rate limiter: Rejected (adds infrastructure dependency)
- Database-backed rate limiter: Rejected (unnecessary DB load)

**Implementation**:
```python
rate_limit_store: Dict[str, RateLimitEntry] = {}

@dataclass
class RateLimitEntry:
    count: int
    reset_time: float
```

---

### Q5: How should profile data be queried?

**Decision**: Direct PostgreSQL query using existing async connection pool

**Rationale**:
- Profile data is in `user_profiles` table (already exists)
- Backend already has `asyncpg` connection to Neon
- Single query by `auth_user_id` is efficient
- No need for ORM complexity

**Query**:
```sql
SELECT
    auth_user_id,
    experience_level,
    programming_languages,
    frameworks_platforms,
    device_type,
    operating_system,
    system_capability
FROM user_profiles
WHERE auth_user_id = $1
```

---

### Q6: What is the optimal prompt engineering strategy?

**Decision**: Use structured system prompt with explicit MUST DO / MUST NOT rules

**Rationale**:
- Clear constraints prevent hallucinations
- Experience level adaptations are explicit
- Code block preservation is critical (spec requirement)
- Output format is unambiguous (raw markdown only)

**Key Prompt Elements**:
1. Clear role definition
2. Explicit personalization rules by profile field
3. Hard constraints on what must NOT change
4. Output format specification (no meta-commentary)

---

### Q7: How should content caching work on frontend?

**Decision**: React component state with session-scoped cache

**Rationale**:
- Cache per chapter, per session
- Invalidate on navigation away from chapter
- Instant toggle between original/personalized
- No server-side caching needed (on-demand generation)

**Implementation**:
```typescript
const [personalizedContent, setPersonalizedContent] = useState<string | null>(null);
const [showPersonalized, setShowPersonalized] = useState(false);

// Toggle is instant when cached
const toggleContent = () => {
  if (personalizedContent) {
    setShowPersonalized(!showPersonalized);
  }
};
```

---

## Best Practices Applied

### OpenAI Agents SDK with Gemini

1. **Trailing Slash**: Always strip trailing slash from base URL
2. **Temperature**: Use 0.3 for consistent, factual output
3. **Max Tokens**: Set appropriate limit (8192 for long chapters)
4. **Error Handling**: Wrap in try/except, return graceful fallback

### FastAPI Async Patterns

1. **Async Database**: Use `asyncpg` for non-blocking queries
2. **Async HTTP**: Use `httpx.AsyncClient` for session validation
3. **Dependency Injection**: Use FastAPI's `Depends` for reusable auth
4. **Pydantic Models**: Type-safe request/response validation

### Better-Auth Integration

1. **Cookie Forwarding**: Pass session cookie to auth backend
2. **Error Handling**: Handle 401/403/404 from auth service
3. **User ID**: Extract from session response, not cookie directly

---

## Technology Versions Confirmed

| Technology | Version | Source |
|------------|---------|--------|
| Python | 3.11+ | `backend/pyproject.toml` |
| FastAPI | 0.100+ | Existing backend |
| OpenAI SDK | 1.x | `openai` package |
| asyncpg | 0.28+ | Existing backend |
| Gemini | gemini-2.0-flash | Environment variable |

---

## External Dependencies

| Dependency | Purpose | URL |
|------------|---------|-----|
| Gemini API | LLM provider | `generativelanguage.googleapis.com` |
| Neon PostgreSQL | User profiles | Existing connection |
| Better-Auth | Session validation | Auth backend service |

---

**END OF RESEARCH**
