# Feature Specification: Chapter Content Personalization (Bonus Feature)

**Feature Name**: Chapter Content Personalization
**Specification File**: `specs/personalized-content/spec.md`
**Status**: Final
**Priority**: Bonus (50 points)
**Version**: 2.0
**Created**: 2025-12-19

---

## 1. Feature Overview

### 1.1 What Personalization Means in This System

Chapter content personalization transforms the explanation style, depth, and contextual examples of textbook content to match an individual user's technical background. The system adapts content presentation while preserving all technical accuracy, code blocks, and structural elements.

Personalization considers:
- **Experience level**: Adjusts explanation complexity (beginner → expert)
- **Programming languages**: Adds relevant language-specific context
- **Frameworks/platforms**: Includes framework-aware notes and analogies
- **Hardware profile**: Provides system-capability-aware recommendations

### 1.2 Why Button-Triggered Personalization

| Reason | Explanation |
|--------|-------------|
| User Agency | Users explicitly choose when to personalize, maintaining control |
| Performance | On-demand processing avoids unnecessary API calls |
| Transparency | Users clearly distinguish original vs. personalized content |
| Reversibility | Users can toggle back to original content instantly |
| Resource Efficiency | Only processes content when explicitly requested |

### 1.3 Hackathon Bonus Relevance

This feature qualifies for **50 bonus points** by meeting the original requirement:

> "Participants can receive up to 50 extra bonus points if the logged user can personalise the content in the chapters by pressing a button at the start of each chapter."

**Qualification criteria (ALL required)**:
1. User MUST be authenticated
2. Button MUST appear at chapter start
3. Personalization MUST use collected user background data
4. Original content MUST remain accessible via toggle

---

## 2. Access Control

### 2.1 Authentication Dependency

This feature depends on the existing Better-Auth authentication system.

| Dependency | Status | Location |
|------------|--------|----------|
| Better-Auth | Implemented | `/auth/` |
| User sessions | Implemented | Neon PostgreSQL |
| User profiles | Implemented | `user_profiles` table |

### 2.2 Visibility Rules for Personalization Button

The "Personalize Content" button MUST render ONLY when ALL conditions are true:

1. User has a valid Better-Auth session
2. User has a complete profile in `user_profiles` table
3. Profile contains all required fields (see Section 2.4)

The button MUST NOT render when:
- User is not logged in
- User session is expired
- User profile does not exist
- User profile is incomplete

### 2.3 Access Control Matrix

| User State | See Button | Trigger API | View Result |
|------------|------------|-------------|-------------|
| Anonymous (not logged in) | NO | NO | NO |
| Authenticated + No Profile | NO | NO | NO |
| Authenticated + Incomplete Profile | NO | NO | NO |
| Authenticated + Complete Profile | YES | YES | YES |

### 2.4 Required Profile Fields

All of the following fields MUST be present in `user_profiles` for personalization to be available:

**Software Background**:
- `experience_level` (beginner | intermediate | advanced | expert)
- `programming_languages` (array, at least one)
- `frameworks_platforms` (array, at least one)

**Hardware Background**:
- `device_type` (desktop | laptop | tablet | mobile | embedded)
- `operating_system` (windows | macos | linux | other)
- `system_capability` (low | medium | high)

---

## 3. UI & Interaction Requirements

### 3.1 Button Placement

| Attribute | Specification |
|-----------|---------------|
| Position | Start of each chapter, AFTER chapter title and metadata |
| Container | Inline with other chapter action buttons (translation, etc.) |
| Visibility | Conditional on authentication + profile status |
| z-index | Above content, below modals |

### 3.2 Button States

| State | Label | Visual | Behavior |
|-------|-------|--------|----------|
| **Default** | "Personalize Content" | Primary/accent color, enabled | Clickable, triggers API |
| **Loading** | "Personalizing..." | Disabled, spinner visible | Non-clickable |
| **Personalized** | "Show Original" | Secondary style | Toggles to original |
| **Error** | "Retry Personalization" | Error color (red) | Clickable, retries API |
| **Cached** | "Show Personalized" | Outline style | Toggles to cached personalized |

### 3.3 Toggle Behavior

**After successful personalization**:
```
Original Content → [Click "Personalize Content"] → Personalized Content
Personalized Content → [Click "Show Original"] → Original Content
Original Content → [Click "Show Personalized"] → Cached Personalized Content
```

**Key toggle rules**:
- Toggle MUST be instant when content is cached
- Original content MUST always be available
- Personalized content MUST be cached per session
- Cache MUST be invalidated on navigation away from chapter

### 3.4 Loading State Behavior

During personalization (loading state):
- Button MUST show spinner and "Personalizing..." text
- Button MUST be disabled (non-clickable)
- Original content MUST remain visible
- User MUST NOT be blocked from scrolling or reading

---

## 4. Personalization Behavior

### 4.1 What MUST Be Personalized

#### 4.1.1 Based on Experience Level

| Level | Personalization |
|-------|-----------------|
| `beginner` | Simpler vocabulary, more context, step-by-step explanations, define acronyms |
| `intermediate` | Balanced depth, assume basic knowledge, practical focus |
| `advanced` | Concise language, skip fundamentals, focus on nuanced concepts |
| `expert` | Direct technical prose, assume deep domain expertise, edge case discussions |

#### 4.1.2 Based on Programming Languages

| User's Languages | Personalization |
|------------------|-----------------|
| Python | Reference Python idioms, PyPI packages, pythonic patterns |
| C/C++ | Add memory considerations, performance notes, pointer context |
| JavaScript/TypeScript | Draw async/event-driven parallels, npm ecosystem references |
| Java | Reference OOP patterns, JVM context |
| Go | Mention goroutines/channels where relevant |
| Rust | Add ownership/borrowing context for memory discussions |

#### 4.1.3 Based on Frameworks/Platforms

| User's Frameworks | Personalization |
|-------------------|-----------------|
| ROS/ROS 2 | Assume ROS familiarity, use ROS terminology freely |
| TensorFlow/PyTorch | Draw ML/DL parallels where relevant |
| Arduino/Embedded | Add embedded-specific context, mention MCU considerations |
| Web Development | Use web analogies for distributed concepts |
| OpenCV | Reference computer vision concepts |

#### 4.1.4 Based on Hardware Background

| Profile | Personalization |
|---------|-----------------|
| `system_capability: low` | Add resource warnings, suggest lightweight alternatives |
| `system_capability: high` | Mention GPU acceleration, parallel processing options |
| `device_type: embedded` | Emphasize embedded-friendly approaches, Jetson/RPi notes |
| `operating_system: windows` | Add Windows-specific path notes, installation hints |
| `operating_system: linux` | Can use Linux-native examples without extra explanation |

### 4.2 What MUST NOT Change

| Element | Reason |
|---------|--------|
| **Code blocks** | Code MUST remain byte-for-byte identical |
| **Command-line examples** | Commands MUST work as documented |
| **Technical facts** | Factual accuracy MUST be preserved |
| **API names/signatures** | Technical identifiers MUST not change |
| **Chapter structure** | Headings, sections, order MUST remain intact |
| **URLs and links** | All links MUST remain functional |
| **Technical term definitions** | Definitions MUST be accurate |
| **Mermaid diagrams** | Diagram code MUST remain unchanged |
| **Images and media** | References MUST not be altered |

### 4.3 Per-Chapter Personalization Scope

- Personalization is scoped to ONE chapter at a time
- Personalizing Chapter 1 MUST NOT affect Chapter 2
- Each chapter maintains independent personalization state
- State resets when navigating to a different chapter

---

## 5. Data Flow

### 5.1 End-to-End Flow

```
[User clicks "Personalize Content"]
         │
         ▼
[Frontend: PersonalizeButton Component]
         │ POST /api/personalize
         │ Body: { chapter_id, chapter_content }
         │ Cookies: Better-Auth session
         ▼
[FastAPI Backend: /api/personalize endpoint]
         │
         │ 1. Validate session via Better-Auth
         │ 2. Query user_profiles from Neon DB
         │ 3. Validate profile completeness
         ▼
[PersonalizeContentAgent (OpenAI Agents SDK)]
         │
         │ 4. Build prompt with profile context
         │ 5. Call Gemini LLM via AsyncOpenAI
         │ 6. Return personalized markdown
         ▼
[FastAPI Backend]
         │
         │ 7. Format response with summary
         ▼
[Frontend]
         │
         │ 8. Render personalized content
         │ 9. Cache for toggle
         ▼
[User sees personalized chapter]
```

### 5.2 User Profile Retrieval

**Source**: Neon PostgreSQL `user_profiles` table

**Query**: By `auth_user_id` from validated session

**Fields retrieved**:
- `experience_level`
- `programming_languages` (array)
- `frameworks_platforms` (array)
- `device_type`
- `operating_system`
- `system_capability`

### 5.3 Agent Invocation

The backend MUST invoke the PersonalizeContentAgent with:

**Input**:
- Original chapter content (markdown)
- User profile data (all 6 fields)

**Output**:
- Personalized markdown content
- Summary of adjustments made

---

## 6. Agent Architecture

### 6.1 PersonalizeContentAgent

A dedicated agent MUST be created using the OpenAI Agents SDK pattern.

**Agent Responsibility**:
Transform chapter content based on user profile while preserving technical accuracy.

### 6.2 Agent Design Requirements

| Requirement | Specification |
|-------------|---------------|
| Framework | OpenAI Agents SDK (Python) |
| LLM Provider | Gemini via OpenAI-compatible endpoint |
| Client | `AsyncOpenAI` from `openai` package |
| Model | `gemini-2.5-flash` (configurable via env) |
| Base URL | `https://generativelanguage.googleapis.com/v1beta/openai` |

### 6.3 Alignment with Existing RAG Agent Pattern

The PersonalizeContentAgent MUST follow the same pattern as the existing RAG agent:

**Pattern Requirements**:
```
1. Use AsyncOpenAI client with Gemini base URL
2. Load API key from OPENAI_API_KEY environment variable
3. Load model name from GEMINI_MODEL environment variable
4. Use temperature 0.3 for consistent output
5. Strip trailing slash from base_url
6. Handle errors with graceful fallback
```

### 6.4 Agent System Prompt Requirements

The system prompt MUST instruct the agent to:

**DO**:
- Adapt explanation depth to experience level
- Add relevant context from user's known languages/frameworks
- Include hardware-aware notes where applicable
- Preserve document structure (headings, lists, tables)

**DO NOT**:
- Modify code blocks (preserve exactly)
- Change command-line examples
- Alter technical term definitions
- Introduce topics not in original content
- Change factual meaning
- Modify URLs or links
- Alter Mermaid diagram code

### 6.5 Agent Location

**File**: `backend/agents/personalize_agent.py`

This location places it alongside the existing `rag_agent.py`.

---

## 7. API Contract

### 7.1 Personalization Endpoint

**Endpoint**: `POST /api/personalize`

**Location**: FastAPI backend (`backend/api/personalize.py`)

### 7.2 Request Structure

```
POST /api/personalize
Content-Type: application/json
Cookie: better-auth.session_token=<session>

{
  "chapter_id": "module-1/chapter-01-intro-physical-ai",
  "chapter_content": "<full markdown content of chapter>"
}
```

**Fields**:
- `chapter_id` (string, required): Unique identifier for the chapter
- `chapter_content` (string, required): Full markdown content to personalize

### 7.3 Success Response

```
HTTP/1.1 200 OK
Content-Type: application/json

{
  "success": true,
  "personalized_content": "<personalized markdown>",
  "chapter_id": "module-1/chapter-01-intro-physical-ai",
  "personalization_summary": {
    "experience_level": "intermediate",
    "programming_context": ["Python", "JavaScript"],
    "hardware_context": {
      "system_capability": "medium",
      "operating_system": "linux"
    },
    "adjustments_made": [
      "Adapted explanations for intermediate level",
      "Added Python-specific context",
      "Included Linux-specific notes"
    ]
  },
  "timestamp": "2025-12-19T10:30:00Z"
}
```

### 7.4 Error Responses

**401 Unauthorized** (AUTH_REQUIRED):
```
{
  "success": false,
  "error": "AUTH_REQUIRED",
  "message": "Authentication required to personalize content"
}
```

**404 Not Found** (PROFILE_NOT_FOUND):
```
{
  "success": false,
  "error": "PROFILE_NOT_FOUND",
  "message": "Please complete your profile to personalize content"
}
```

**400 Bad Request** (PROFILE_INCOMPLETE):
```
{
  "success": false,
  "error": "PROFILE_INCOMPLETE",
  "message": "Your profile is missing required fields",
  "missing_fields": ["experience_level", "device_type"]
}
```

**429 Too Many Requests** (RATE_LIMITED):
```
{
  "success": false,
  "error": "RATE_LIMITED",
  "message": "Too many requests. Please try again later.",
  "retry_after": 60
}
```

**500 Internal Server Error** (PERSONALIZATION_FAILED):
```
{
  "success": false,
  "error": "PERSONALIZATION_FAILED",
  "message": "Personalization service encountered an error",
  "retry_after": 5
}
```

---

## 8. Non-Functional Requirements

### 8.1 Performance

| Metric | Target |
|--------|--------|
| Button render time | < 100ms after page load |
| Loading state display | < 500ms after click |
| Personalization completion | < 60 seconds for 5000-word chapter |
| Toggle response (cached) | < 100ms |
| API cold start | < 2 seconds |

### 8.2 Security

| Requirement | Specification |
|-------------|---------------|
| Authentication | Server-side session validation REQUIRED |
| Authorization | Users access ONLY their own profile data |
| Input sanitization | Chapter content sanitized before LLM processing |
| Output sanitization | Personalized content sanitized before render |
| XSS prevention | No raw HTML injection allowed |
| CORS | Configured for allowed origins only |

### 8.3 Reliability

| Requirement | Specification |
|-------------|---------------|
| Availability | 99% uptime for personalization service |
| Error recovery | Graceful fallback to original content on any error |
| Timeout handling | 60-second timeout with clear error message |
| Retry logic | Exponential backoff with max 3 retries |
| Session handling | Handle expired sessions gracefully |

### 8.4 Rate Limiting

| Parameter | Value |
|-----------|-------|
| Limit | 5 requests per user per minute |
| Window | 60 seconds sliding window |
| Scope | Per authenticated user |
| Response | HTTP 429 with `retry_after` header |

---

## 9. Bonus Point Eligibility

### 9.1 Judge Verification Checklist

Judges MUST verify ALL of the following to award 50 points:

| # | Requirement | Verification Method |
|---|-------------|---------------------|
| 1 | User can sign in via Better-Auth | Sign in with test credentials |
| 2 | User profile exists with software/hardware data | Check database or profile page |
| 3 | "Personalize Content" button visible at chapter start | Visual inspection |
| 4 | Button NOT visible when signed out | Sign out and verify |
| 5 | Clicking button triggers personalization | Click and observe loading state |
| 6 | Personalization uses experience_level | Compare beginner vs expert output |
| 7 | Personalization uses programming_languages | Check for language-specific notes |
| 8 | Personalization uses hardware background | Check for capability-specific notes |
| 9 | Personalized content displayed to user | Content visibly changes |
| 10 | "Show Original" returns to original content | Toggle works |
| 11 | Code blocks remain unchanged | Compare code blocks |
| 12 | Technical facts preserved | Verify accuracy |

### 9.2 Partial Implementation = ZERO Points

**CRITICAL**: If ANY of the following are true, the bonus is NOT awarded:

| Failure | Result |
|---------|--------|
| No authentication required | 0 points |
| Button visible to anonymous users | 0 points |
| Personalization ignores profile data | 0 points |
| Only uses experience_level (ignores hardware) | 0 points |
| Cannot toggle to original content | 0 points |
| Code blocks are modified | 0 points |
| Factual meaning is changed | 0 points |
| Agent not using OpenAI Agents SDK | 0 points |
| Not using Gemini LLM | 0 points |

---

## 10. Acceptance Criteria

### 10.1 Access Control Tests

**Test PC-AC-001**: Button hidden for anonymous users
- **Given**: User is NOT logged in
- **When**: User views any chapter page
- **Then**: "Personalize Content" button is NOT visible

**Test PC-AC-002**: Button hidden for users without profile
- **Given**: User IS logged in BUT has NO profile
- **When**: User views any chapter page
- **Then**: "Personalize Content" button is NOT visible

**Test PC-AC-003**: Button visible for authenticated users with profile
- **Given**: User IS logged in AND has complete profile
- **When**: User views any chapter page
- **Then**: "Personalize Content" button IS visible at chapter start

**Test PC-AC-004**: API rejects unauthenticated requests
- **Given**: No session cookie provided
- **When**: POST /api/personalize is called
- **Then**: Response is 401 AUTH_REQUIRED

### 10.2 Personalization Behavior Tests

**Test PC-PB-001**: Beginner experience personalization
- **Given**: User with experience_level = "beginner"
- **When**: User personalizes a chapter
- **Then**: Content has simpler explanations and more context

**Test PC-PB-002**: Expert experience personalization
- **Given**: User with experience_level = "expert"
- **When**: User personalizes a chapter
- **Then**: Content has concise, technical language

**Test PC-PB-003**: Programming language context
- **Given**: User with programming_languages = ["Python", "C++"]
- **When**: User personalizes a chapter
- **Then**: Content includes Python/C++ relevant notes

**Test PC-PB-004**: Hardware capability warnings
- **Given**: User with system_capability = "low"
- **When**: User personalizes a chapter with resource-heavy topics
- **Then**: Content includes resource warnings

**Test PC-PB-005**: Code blocks preserved
- **Given**: Chapter contains code blocks
- **When**: Chapter is personalized
- **Then**: Code blocks are byte-for-byte identical

**Test PC-PB-006**: Structure preserved
- **Given**: Chapter has headings H1-H4
- **When**: Chapter is personalized
- **Then**: All headings remain in same order and level

**Test PC-PB-007**: No new topics introduced
- **Given**: Original chapter does not mention "Kubernetes"
- **When**: Chapter is personalized
- **Then**: Personalized chapter does not mention "Kubernetes"

### 10.3 Toggle Tests

**Test PC-TG-001**: Toggle to original
- **Given**: User viewing personalized content
- **When**: User clicks "Show Original"
- **Then**: Original content is displayed

**Test PC-TG-002**: Toggle to cached personalized
- **Given**: User viewing original (after personalization)
- **When**: User clicks "Show Personalized"
- **Then**: Cached personalized content is displayed instantly

**Test PC-TG-003**: Independent chapter state
- **Given**: User has personalized Chapter 1
- **When**: User navigates to Chapter 2
- **Then**: Chapter 2 shows original with "Personalize Content" button

### 10.4 Error Handling Tests

**Test PC-EH-001**: Graceful error recovery
- **Given**: Personalization API fails
- **When**: Error occurs
- **Then**: User sees error message, original content remains visible, retry available

**Test PC-EH-002**: Rate limit handling
- **Given**: User exceeds 5 requests per minute
- **When**: Next request is made
- **Then**: HTTP 429 returned with retry_after value

**Test PC-EH-003**: Incomplete profile handling
- **Given**: User profile missing experience_level
- **When**: User tries to personalize
- **Then**: Error message lists missing fields

---

## 11. Out of Scope

The following are **explicitly NOT included** in this specification:

### Content Modifications
- Automatic/background personalization (user MUST click button)
- Translation of content (separate feature)
- Modification of original markdown files
- Editing of code blocks or technical facts

### Chatbot Integration
- Modification of existing RAG chatbot
- Personalized chatbot responses
- Chatbot-triggered personalization

### Persistence Features
- Saving personalized content to database
- Cross-session personalization persistence
- Personalization history or versioning

### Bulk Operations
- Multi-chapter personalization
- Batch personalization API
- Pre-generated personalized content

### Analytics
- Personalization usage tracking
- A/B testing of personalization strategies
- Quality metrics dashboard

---

## 12. Implementation Constraints

### 12.1 Technology Stack (MANDATORY)

| Component | Technology | Requirement |
|-----------|------------|-------------|
| Frontend | Docusaurus (React) | Existing |
| Authentication | Better-Auth | Existing |
| Database | Neon PostgreSQL | Existing |
| Backend | FastAPI (Python) | REQUIRED |
| Agent Framework | OpenAI Agents SDK | REQUIRED |
| LLM Provider | Gemini via OpenAI-compatible API | REQUIRED |
| LLM Client | AsyncOpenAI | REQUIRED |
| Model | gemini-2.5-flash | REQUIRED |

### 12.2 File Structure

```
backend/
├── agents/
│   ├── rag_agent.py           # Existing - DO NOT MODIFY
│   └── personalize_agent.py   # NEW - PersonalizeContentAgent
├── api/
│   └── personalize.py         # NEW - FastAPI endpoint
└── config.py                  # Existing - add personalization config

book-write/
└── src/
    └── components/
        └── Personalization/
            ├── PersonalizeButton.tsx
            ├── PersonalizedContent.tsx
            └── types.ts
```

### 12.3 Environment Variables

```
# Existing (from RAG agent)
OPENAI_API_KEY=<gemini-api-key>
OPENAI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai
GEMINI_MODEL=gemini-2.5-flash

# New (for personalization)
PERSONALIZE_MAX_TOKENS=8192
PERSONALIZE_RATE_LIMIT=5
PERSONALIZE_TIMEOUT=60
```

---

## 13. Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-18 | Claude Code | Initial specification |
| 2.0 | 2025-12-19 | Claude Code | Updated to mandate OpenAI Agents SDK + FastAPI + Gemini pattern |

---

**END OF SPECIFICATION**
