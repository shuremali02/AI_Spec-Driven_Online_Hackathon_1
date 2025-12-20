# Implementation Plan: Chapter Content Personalization (Bonus Feature)

**Branch**: `personalized-feature`
**Date**: 2025-12-19
**Spec**: `specs/personalized-content/spec.md`
**Input**: Feature specification v2.0

---

## Summary

Implement a chapter content personalization feature that transforms textbook content based on user profile data (experience level, programming languages, frameworks, hardware background). The feature is triggered via a button at the start of each chapter, available only to authenticated users with complete profiles. The implementation MUST use FastAPI backend with OpenAI Agents SDK and Gemini LLM, following the existing RAG agent pattern.

---

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript (frontend)
**Primary Dependencies**: FastAPI, OpenAI SDK (AsyncOpenAI), Pydantic, asyncpg
**Storage**: Neon PostgreSQL (existing `user_profiles` table)
**Testing**: pytest with pytest-asyncio
**Target Platform**: Linux server (Vercel/Railway deployment)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: < 60 seconds for 5000-word chapter personalization
**Constraints**: Rate limit 5 req/min/user, < 100ms toggle response, 60s timeout
**Scale/Scope**: Single user per request, per-chapter scope, no persistence

---

## Constitution Check

*GATE: Must pass before implementation. Re-checked after design.*

| Gate | Requirement | Status |
|------|-------------|--------|
| Core Principle V | Authentication-first access control | ✅ PASS - Button requires auth |
| Key Standard VI | Personalization based on user background data | ✅ PASS - Uses 6 profile fields |
| Key Standard VII | Authentication via Better-Auth | ✅ PASS - Uses existing auth |
| Part-2 Core Principle I | Gemini LLM via OpenAI-compatible endpoint | ✅ PASS - Uses AsyncOpenAI + Gemini |
| Constraint II | No hallucinations (preserve technical accuracy) | ✅ PASS - Code/facts preserved |
| Constraint V | API compliance (Gemini ToS) | ✅ PASS - Standard API usage |

**Violations**: None

---

## High-Level Architecture Flow

```
[User clicks "Personalize Content" button]
           │
           ▼
[Frontend: PersonalizeButton Component]
           │ POST /api/personalize
           │ Body: { chapter_id, chapter_content }
           │ Cookie: Better-Auth session
           ▼
[FastAPI Backend: /api/personalize endpoint]
           │
           │ 1. Extract session token from cookie
           │ 2. Validate session via Better-Auth API call
           │ 3. Query user_profiles from Neon DB
           │ 4. Validate profile completeness (6 fields)
           │ 5. Check rate limit (5/min/user)
           ▼
[PersonalizeContentAgent (OpenAI Agents SDK)]
           │
           │ 6. Build system prompt + user prompt
           │ 7. Call Gemini LLM via AsyncOpenAI client
           │ 8. Extract personalized markdown
           │ 9. Build adjustments summary
           ▼
[FastAPI Backend]
           │
           │ 10. Format response with summary
           ▼
[Frontend]
           │
           │ 11. Render personalized content
           │ 12. Cache for toggle
           ▼
[User sees personalized chapter]
```

---

## Backend Implementation Steps

### Step 1: Create PersonalizeContentAgent

**File**: `backend/agents/personalize_agent.py`

**Responsibilities**:
- Initialize AsyncOpenAI client with Gemini endpoint
- Load configuration from environment variables
- Build system prompt with personalization rules
- Build user prompt with profile context
- Call Gemini LLM and extract response
- Return personalized content with adjustments summary

**Pattern Requirements** (from RAG agent):
```python
# Match RAG agent pattern exactly:
base_url = os.getenv("OPENAI_BASE_URL", "https://generativelanguage.googleapis.com/v1beta/openai")
base_url = base_url.rstrip('/')  # Remove trailing slash
api_key = os.getenv("OPENAI_API_KEY")
model = os.getenv("GEMINI_MODEL", "gemini-2.5-flash")

self.client = AsyncOpenAI(
    base_url=base_url,
    api_key=api_key
)
```

**System Prompt Strategy**:
```
MUST DO:
- Adapt explanation depth to experience level
- Add language-specific context
- Include hardware-aware notes
- Preserve document structure

MUST NOT:
- Modify code blocks (preserve exactly)
- Change command-line examples
- Alter technical terms
- Introduce new topics
- Change factual meaning
```

### Step 2: Create FastAPI Endpoint

**File**: `backend/api/personalize.py`

**Responsibilities**:
- Define POST /api/personalize route
- Parse request body (chapter_id, chapter_content)
- Validate session via Better-Auth
- Query user profile from Neon DB
- Validate profile completeness
- Enforce rate limiting
- Invoke PersonalizeContentAgent
- Return formatted response

**Request Validation**:
- `chapter_id`: string, required
- `chapter_content`: string, required

**Error Codes**:
- 401: AUTH_REQUIRED
- 404: PROFILE_NOT_FOUND
- 400: PROFILE_INCOMPLETE
- 429: RATE_LIMITED
- 500: PERSONALIZATION_FAILED

### Step 3: Add Route to FastAPI Main

**File**: `backend/main.py`

**Changes**:
- Import personalize router
- Include router with prefix `/api`
- Add CORS configuration for frontend origin

### Step 4: Database Integration

**Use existing**: `backend/db/postgres_client.py`

**Query**:
```python
SELECT * FROM user_profiles WHERE auth_user_id = $1
```

**Fields used**:
- experience_level
- programming_languages (array)
- frameworks_platforms (array)
- device_type
- operating_system
- system_capability

### Step 5: Session Validation

**Approach**: Call Better-Auth API to validate session

**Options**:
1. Forward session cookie to auth backend `/api/auth/get-session`
2. Direct database lookup via session table

**Recommended**: Option 1 (maintains auth backend as single source of truth)

---

## PersonalizeContentAgent Design

### Class Structure

```python
class PersonalizeContentAgent:
    def __init__(self):
        # Initialize AsyncOpenAI client (Gemini)
        # Load model config from env
        # Set max_tokens, temperature

    async def personalize(
        self,
        content: str,
        profile: UserProfile
    ) -> PersonalizationResponse:
        # Build prompts
        # Call LLM
        # Parse response
        # Return result
```

### System Prompt

```
You are a content personalization assistant for a Physical AI & Humanoid Robotics textbook.

Your task is to adapt chapter content based on the user's background while following these rules:

MUST DO:
- Adjust explanation depth based on experience level
- Use wording appropriate for the experience level
- Add relevant context based on known programming languages
- Include hardware-aware notes based on system capability
- Preserve the exact document structure (headings, sections, lists)

MUST NOT:
- Change any code blocks (preserve exactly as-is)
- Modify command-line examples
- Alter technical term definitions
- Introduce new topics not in the original
- Change factual meaning of any content
- Modify URLs or links
- Alter Mermaid diagram code

EXPERIENCE LEVEL ADAPTATIONS:
- beginner: Simpler language, more context, step-by-step, define acronyms
- intermediate: Balanced, assume basics, practical focus
- advanced: Concise technical language, skip basics
- expert: Direct prose, assume deep knowledge

OUTPUT FORMAT:
Return ONLY the personalized markdown content. No meta-commentary.
```

### User Prompt Template

```
Personalize the following chapter content for a user with this background:

**Experience Level**: {experience_level}
**Programming Languages**: {languages_joined}
**Frameworks/Platforms**: {frameworks_joined}
**Device Type**: {device_type}
**Operating System**: {operating_system}
**System Capability**: {system_capability}

---

CHAPTER CONTENT:

{chapter_content}
```

---

## Auth + Profile Validation Flow

### Session Validation

```python
async def validate_session(request: Request) -> Optional[str]:
    """Extract and validate session, return user_id or None."""

    # 1. Extract session cookie
    session_token = request.cookies.get("better-auth.session_token")
    if not session_token:
        return None

    # 2. Call auth backend to validate
    async with httpx.AsyncClient() as client:
        response = await client.get(
            f"{AUTH_BACKEND_URL}/api/auth/get-session",
            cookies={"better-auth.session_token": session_token}
        )
        if response.status_code != 200:
            return None

        data = response.json()
        return data.get("user", {}).get("id")
```

### Profile Validation

```python
def validate_profile(profile: UserProfile) -> tuple[bool, list[str]]:
    """Check all 6 required fields are present."""
    missing = []

    if not profile.experience_level:
        missing.append("experience_level")
    if not profile.programming_languages:
        missing.append("programming_languages")
    if not profile.frameworks_platforms:
        missing.append("frameworks_platforms")
    if not profile.device_type:
        missing.append("device_type")
    if not profile.operating_system:
        missing.append("operating_system")
    if not profile.system_capability:
        missing.append("system_capability")

    return (len(missing) == 0, missing)
```

---

## Frontend Integration Steps

### Step 1: Update PersonalizeButton API Call

**File**: `book-write/src/components/Personalization/PersonalizeButton.tsx`

**Changes**:
- Update API URL to point to FastAPI backend (if different from current)
- Ensure chapter_content is sent in request body
- Handle all error codes (401, 404, 400, 429, 500)

### Step 2: Ensure ChapterPersonalizer Integration

**File**: `book-write/src/components/Personalization/ChapterPersonalizer.tsx`

**Verify**:
- Component is integrated in MDX pages
- Auth check before showing button
- Profile check before showing button
- Toggle between original/personalized works
- Cache invalidation on navigation

### Step 3: Update API Configuration

**File**: `book-write/src/components/Personalization/utils.ts`

**Verify**:
- API base URL is configurable via env
- Credentials mode includes cookies
- Error response parsing is correct

---

## Error Handling & Rate Limiting Strategy

### Rate Limiting Implementation

```python
# In-memory rate limiter (per-process)
rate_limit_store: Dict[str, RateLimitEntry] = {}

RATE_LIMIT_MAX = 5  # requests per window
RATE_LIMIT_WINDOW = 60  # seconds

def check_rate_limit(user_id: str) -> tuple[bool, Optional[int]]:
    """Returns (allowed, retry_after_seconds)."""
    now = time.time()
    entry = rate_limit_store.get(user_id)

    if not entry or now > entry.reset_time:
        rate_limit_store[user_id] = RateLimitEntry(
            count=1,
            reset_time=now + RATE_LIMIT_WINDOW
        )
        return (True, None)

    if entry.count >= RATE_LIMIT_MAX:
        retry_after = int(entry.reset_time - now)
        return (False, retry_after)

    entry.count += 1
    return (True, None)
```

### Error Handling Strategy

| Error | HTTP Code | Response | Frontend Action |
|-------|-----------|----------|-----------------|
| No session cookie | 401 | AUTH_REQUIRED | Show sign-in prompt |
| Invalid session | 401 | AUTH_REQUIRED | Show sign-in prompt |
| No profile | 404 | PROFILE_NOT_FOUND | Link to profile setup |
| Incomplete profile | 400 | PROFILE_INCOMPLETE + fields | List missing fields |
| Rate limited | 429 | RATE_LIMITED + retry_after | Show countdown |
| LLM failure | 500 | PERSONALIZATION_FAILED | Show retry button |
| Timeout | 500 | PERSONALIZATION_FAILED | Show retry button |

### Graceful Fallback

On ANY error:
1. Display user-friendly error message
2. Keep original content visible
3. Show retry option (where applicable)
4. Never leave user with blank content

---

## Project Structure

### Documentation (this feature)

```text
specs/personalized-content/
├── spec.md              # Feature specification (done)
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output
├── quickstart.md        # Phase 1 output
├── contracts/           # Phase 1 output
│   └── personalize-api.md
└── tasks.md             # Phase 2 output (via /sp.tasks)
```

### Source Code (repository root)

```text
backend/
├── agents/
│   ├── rag_agent.py           # Existing - DO NOT MODIFY
│   ├── system_prompt.py       # Existing - DO NOT MODIFY
│   └── personalize_agent.py   # NEW - PersonalizeContentAgent
├── api/
│   ├── routes_chat.py         # Existing - DO NOT MODIFY
│   └── personalize.py         # NEW - FastAPI endpoint
├── db/
│   └── postgres_client.py     # Existing - use for profile query
├── config.py                  # Existing - add personalization config
└── main.py                    # Existing - add personalize router

book-write/
└── src/
    └── components/
        └── Personalization/   # Existing - verify integration
            ├── PersonalizeButton.tsx
            ├── PersonalizedContent.tsx
            ├── ChapterPersonalizer.tsx
            ├── utils.ts
            ├── types.ts
            └── index.ts
```

---

## Environment Variables

### Existing (from RAG agent)

```bash
OPENAI_API_KEY=<gemini-api-key>
OPENAI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai
GEMINI_MODEL=gemini-2.5-flash
DATABASE_URL=<neon-postgres-url>
```

### New (for personalization)

```bash
PERSONALIZE_MAX_TOKENS=8192
PERSONALIZE_RATE_LIMIT=5
PERSONALIZE_TIMEOUT=60
AUTH_BACKEND_URL=<auth-service-url>
```

---

## Final Checklist: 50-Point Bonus Eligibility

| # | Requirement | Verification | Implementation |
|---|-------------|--------------|----------------|
| 1 | User can sign in via Better-Auth | Sign in test | Existing |
| 2 | User profile with software/hardware data | DB check | Existing |
| 3 | "Personalize Content" button at chapter start | Visual | Frontend |
| 4 | Button NOT visible when signed out | Sign out test | Frontend |
| 5 | Button triggers personalization | Click test | API + Agent |
| 6 | Uses experience_level | Compare outputs | Agent prompt |
| 7 | Uses programming_languages | Check notes | Agent prompt |
| 8 | Uses hardware background | Check warnings | Agent prompt |
| 9 | Personalized content displayed | UI shows change | Frontend |
| 10 | "Show Original" toggle works | Click test | Frontend |
| 11 | Code blocks unchanged | Diff test | Agent prompt |
| 12 | Technical facts preserved | Review | Agent prompt |
| 13 | FastAPI backend | Endpoint location | NEW |
| 14 | OpenAI Agents SDK | Agent pattern | NEW |
| 15 | Gemini LLM via OpenAI-compatible | Client config | NEW |

### Critical Implementation Tasks

1. **Create PersonalizeContentAgent** (`backend/agents/personalize_agent.py`)
   - Follow RAG agent pattern exactly
   - Use AsyncOpenAI + Gemini
   - Implement personalize() method

2. **Create FastAPI endpoint** (`backend/api/personalize.py`)
   - POST /api/personalize
   - Session validation via Better-Auth
   - Profile query via Neon DB
   - Rate limiting

3. **Wire up in main.py**
   - Import and include router
   - CORS configuration

4. **Verify frontend integration**
   - API URL configuration
   - Error handling
   - Toggle functionality

---

## Complexity Tracking

No constitution violations requiring justification.

---

## Next Steps

1. Run `/sp.tasks` to generate task breakdown
2. Implement PersonalizeContentAgent
3. Implement FastAPI endpoint
4. Wire up in main.py
5. Test end-to-end flow
6. Verify all 15 checklist items

---

**END OF IMPLEMENTATION PLAN**
