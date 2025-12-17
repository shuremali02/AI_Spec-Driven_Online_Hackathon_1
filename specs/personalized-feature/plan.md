# Implementation Plan: Chapter Content Personalization

**Branch**: `personalization-feature` | **Date**: 2025-12-18 | **Spec**: [specs/personalized-feature/spec.md](./spec.md)
**Input**: Feature specification from `/specs/personalized-feature/spec.md`

---

## Summary

Implement a bonus feature (50 points) that allows authenticated users to personalize chapter content based on their software and hardware background by pressing a button at the start of each chapter. The feature uses the user's profile data (experience level, programming languages, frameworks, device type, OS, system capability) to adapt explanations, add relevant context, and include hardware-aware notes while preserving code blocks and technical accuracy.

**Key Technical Approach**:
- Frontend: React components (PersonalizeButton, PersonalizedContent) integrated with Docusaurus
- Backend: New `/api/personalize` endpoint in auth service with session + profile validation
- AI: Google Gemini API for content personalization (via Vercel AI SDK pattern)
- State: Component-level React state with caching (no database persistence)

---

## Technical Context

**Language/Version**: TypeScript 5.x (frontend), TypeScript/Node.js 18+ (backend)
**Primary Dependencies**: React 18, Docusaurus 3.x, Hono, Better-Auth, Google Gemini API
**Storage**: None (stateless - no database for personalized content)
**Testing**: Manual testing, acceptance criteria from spec
**Target Platform**: Web (Desktop, Tablet, Mobile responsive)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <45s personalization time, <100ms toggle response
**Constraints**: Must use user profile data, auth required, preserve code blocks
**Scale/Scope**: ~5 chapters initially, ~4000 words per chapter

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Pre-Design Check

| Gate | Requirement | Status | Evidence |
|------|-------------|--------|----------|
| **Key Standard VI** | Personalization based on user background | PASS | Spec Section 5.1 uses all profile fields |
| **Core Principle V** | Only authenticated users can access | PASS | Spec Section 3.2 access matrix |
| **Key Standard VI** | Software + hardware background used | PASS | Spec Sections 5.1.1-5.1.4 |
| **Success Criteria V** | Button-triggered personalization | PASS | Spec Section 4.1 button at chapter start |
| **Success Criteria V** | Profile data observable post-login | PASS | Spec Section 6.2 runtime generation |
| **Core Principle I** | Technical accuracy preserved | PASS | Spec Section 5.3 MUST NOT rules |

### Post-Design Check

| Gate | Requirement | Status | Evidence |
|------|-------------|--------|----------|
| **Constitution VI** | All profile fields used | PASS | API uses all 6 profile fields |
| **No Hallucinations** | Grounded in spec | PASS | All decisions from spec.md |
| **Consistency** | Uniform patterns | PASS | Follows translation-feature patterns |

---

## Project Structure

### Documentation (this feature)

```text
specs/personalized-feature/
├── spec.md              # Feature specification (created)
├── plan.md              # This file
├── research.md          # Phase 0 output (TBD)
├── data-model.md        # Phase 1 output (TBD)
├── quickstart.md        # Phase 1 output (TBD)
├── contracts/
│   └── personalize-api-contract.md  # API contract (TBD)
└── tasks.md             # Phase 2 output (TBD - /sp.tasks)
```

### Source Code (repository root)

```text
# Frontend (Docusaurus)
book-write/
├── src/
│   ├── components/
│   │   ├── Translation/           # EXISTING - reference pattern
│   │   │   ├── TranslateButton.tsx
│   │   │   └── UrduContent.tsx
│   │   └── Personalization/       # NEW
│   │       ├── PersonalizeButton.tsx
│   │       ├── PersonalizedContent.tsx
│   │       ├── Personalization.module.css
│   │       ├── types.ts
│   │       └── index.ts

# Backend (Auth Service)
auth/
├── src/
│   ├── routes/
│   │   ├── translate.ts           # EXISTING - reference pattern
│   │   └── personalize.ts         # NEW: Personalization endpoint
│   └── index.ts                   # MODIFY: Register personalize route
```

**Structure Decision**: Mirrors the translation-feature structure for consistency. Frontend components in `/book-write/src/components/Personalization/`. Backend route in existing `/auth/` service to reuse session validation and profile access.

---

## Existing Infrastructure Analysis

### Auth Backend (Reusable)

| Component | Location | Can Reuse |
|-----------|----------|-----------|
| Session validation | `auth/src/middleware/auth.ts` | YES - `requireAuth` middleware |
| Profile fetching | `auth/src/routes/profile.ts` | YES - DB query pattern |
| Error handling | `auth/src/utils/errors.ts` | YES - error response helpers |
| Rate limiting | `auth/src/routes/translate.ts` | YES - rate limit pattern |

### Frontend (Reusable)

| Component | Location | Can Reuse |
|-----------|----------|-----------|
| Auth context | `book-write/src/components/Auth/AuthProvider.tsx` | YES - `useAuth()` hook with `profile` |
| Button pattern | `book-write/src/components/Translation/TranslateButton.tsx` | YES - state machine pattern |
| Content display | `book-write/src/components/Translation/UrduContent.tsx` | YES - markdown rendering |
| CSS module | `book-write/src/components/Translation/Translation.module.css` | YES - button styling |

### Key Finding: Profile Already Available

The `AuthProvider` already fetches and exposes user profile via `useAuth()`:
```typescript
const { user, profile, isLoading } = useAuth();
// profile contains: software_background, hardware_background
```

This means we can check `profile !== null` to determine if user has profile data.

---

## Complexity Tracking

No constitution violations requiring justification. Design follows minimal viable approach:

| Decision | Simpler Alternative | Why Current Approach |
|----------|--------------------|--------------------|
| AI API vs static rules | Could use hardcoded templates | AI produces natural, context-aware personalization |
| Backend API vs frontend-only | Frontend could call AI directly | Auth validation needed, profile access server-side |
| Component state vs global state | Could use Redux | Per-chapter scope doesn't need global state |
| Separate endpoint vs extend translate | Could add to /translate | Different purpose, different processing logic |

---

## Architecture Decisions

### ADR-001: Personalization Backend Placement

**Decision**: Add personalization endpoint to existing `/auth/` backend

**Rationale**:
- Reuses existing session validation middleware
- Direct access to user profile via database
- No new service to deploy/maintain
- Consistent with translation-feature pattern

**Alternatives Rejected**:
- New microservice: Overkill for single endpoint
- Frontend-only: Need server-side profile access and AI API key protection

### ADR-002: AI Provider for Personalization

**Decision**: Use Google Gemini API (gemini-2.0-flash) for content personalization

**Rationale**:
- Already used in RAG chatbot (team familiar)
- Free tier available for hackathon
- Good at following instructions for content transformation
- Fast response time

**Alternatives Rejected**:
- OpenAI: Cost concerns for hackathon
- Claude: Additional API key management
- Hardcoded rules: Less natural personalization

### ADR-003: State Management

**Decision**: Component-level React state (useState) with caching

**Rationale**:
- Personalization state is per-chapter, per-session
- No persistence requirement (out of scope)
- Simple toggle logic
- Caching in component state enables instant toggle

**Alternatives Rejected**:
- Redux: Overkill for chapter-scoped state
- localStorage: Out of scope per spec
- Database: Explicitly excluded in spec

### ADR-004: Profile Validation

**Decision**: Require complete profile (all 6 fields) for personalization

**Rationale**:
- Spec requires using BOTH software AND hardware background
- Partial profile would produce incomplete personalization
- AuthProvider already fetches profile

**Validation Logic**:
```typescript
const hasCompleteProfile = (profile: ProfileResponse | null): boolean => {
  if (!profile) return false;
  const { software_background, hardware_background } = profile;
  return !!(
    software_background?.experience_level &&
    software_background?.programming_languages?.length > 0 &&
    software_background?.frameworks_platforms?.length > 0 &&
    hardware_background?.device_type &&
    hardware_background?.operating_system &&
    hardware_background?.system_capability
  );
};
```

---

## Component Design

### Frontend Components

```
┌─────────────────────────────────────────────────────────────┐
│ ChapterPage (Docusaurus)                                    │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ Button Container                                      │  │
│  │   ├── TranslateButton (existing)                      │  │
│  │   └── PersonalizeButton (NEW)                         │  │
│  │       Props: chapterId, chapterContent                │  │
│  │       State: status, showPersonalized, error          │  │
│  │       Renders: Only if authenticated + hasProfile     │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ Content Display                                       │  │
│  │   Conditional:                                        │  │
│  │   - showPersonalized=false → Original MDX content     │  │
│  │   - showPersonalized=true  → PersonalizedContent      │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ PersonalizedContent                                   │  │
│  │   Props: content, personalizationSummary              │  │
│  │   Display: Badge + adapted content                    │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Backend Endpoint

```
POST /api/personalize
├── Middleware: requireAuth (validates Better-Auth session)
├── Validation: chapter_id (required)
├── Processing:
│   ├── Fetch user profile from database
│   ├── Validate profile is complete
│   ├── Fetch chapter content from filesystem
│   ├── Call Gemini API with personalization prompt
│   └── Return personalized content
└── Response: { success, personalized_content, chapter_id, personalization_summary, timestamp }
```

---

## Data Flow

```
User Action: Click "Personalize Content"
        │
        ▼
PersonalizeButton.handlePersonalize()
        │
        ├── Check: isAuthenticated AND hasProfile? (from useAuth)
        │         No → Button not rendered (never reaches here)
        │
        ├── Set state: status = 'loading'
        │
        ▼
fetch('/api/personalize', { method: 'POST', body: { chapter_id } })
        │
        ▼
Backend: /api/personalize
        │
        ├── requireAuth middleware → validates session
        │
        ├── Fetch user profile from DB (via auth_user_id)
        │
        ├── Validate profile completeness
        │
        ├── Fetch chapter content from filesystem
        │
        ├── Build personalization prompt with:
        │   - Original content
        │   - experience_level
        │   - programming_languages[]
        │   - frameworks_platforms[]
        │   - device_type
        │   - operating_system
        │   - system_capability
        │
        ├── Call Gemini API
        │         │
        │         ├── System prompt: Personalization rules
        │         ├── User prompt: Content + profile context
        │         └── Response: Personalized markdown
        │
        ▼
Response: { success: true, personalized_content: "...", personalization_summary: {...} }
        │
        ▼
PersonalizeButton receives response
        │
        ├── Set state: status = 'success', showPersonalized = true
        │
        ├── Cache personalized content in component state
        │
        ▼
PersonalizedContent renders with visual indicator
```

---

## Personalization Prompt Design

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
- beginner: Simpler language, more context, step-by-step guidance
- intermediate: Balanced explanations, assume basics, practical focus
- advanced: Concise technical language, skip basics, advanced concepts
- expert: Direct technical prose, assume deep knowledge

Return ONLY the personalized markdown content. Do not include explanations or meta-commentary.
```

### User Prompt Template

```
Personalize the following chapter content for a user with this background:

**Experience Level**: {experience_level}
**Programming Languages**: {programming_languages}
**Frameworks/Platforms**: {frameworks_platforms}
**Device Type**: {device_type}
**Operating System**: {operating_system}
**System Capability**: {system_capability}

---

CHAPTER CONTENT:

{chapter_content}
```

---

## Integration Points

### 1. AuthProvider Integration (Profile Check)

```typescript
// PersonalizeButton uses existing useAuth hook
import { useAuth } from '@site/src/components/Auth/AuthProvider';

const { user, profile, isLoading } = useAuth();

// Don't render if no user OR no profile
if (!user || !profile) return null;

// Validate profile completeness
if (!hasCompleteProfile(profile)) return null;
```

### 2. Docusaurus Layout Integration

Same approach as TranslateButton - import into chapter MDX files:

```mdx
import { PersonalizeButton } from '@site/src/components/Personalization';
import { TranslateButton } from '@site/src/components/Translation';

<div className="chapter-actions">
  <TranslateButton chapterId="chapter-01" chapterContent={rawContent} />
  <PersonalizeButton chapterId="chapter-01" chapterContent={rawContent} />
</div>

... chapter content ...
```

### 3. Auth Backend Integration

```typescript
// auth/src/index.ts
import personalizeRoutes from './routes/personalize.js';
app.route('/api', personalizeRoutes); // Adds /api/personalize
```

---

## API Contract

### Request

```
POST /api/personalize
Content-Type: application/json
Cookie: Better-Auth session cookie

{
  "chapter_id": "module-1/chapter-01-intro-physical-ai"
}
```

Note: Chapter content is fetched server-side to avoid sending large payloads from client.

### Response (Success)

```json
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
      "Added Python-specific examples",
      "Included Linux-specific path notes"
    ]
  },
  "timestamp": "2025-12-18T10:30:00Z"
}
```

### Response (Errors)

| Status | Error Code | Message |
|--------|------------|---------|
| 401 | AUTH_REQUIRED | Authentication required to personalize content |
| 404 | PROFILE_NOT_FOUND | Please complete your profile to personalize content |
| 400 | PROFILE_INCOMPLETE | Your profile is missing required background information |
| 400 | INVALID_REQUEST | Chapter ID is required |
| 404 | CHAPTER_NOT_FOUND | Chapter not found |
| 429 | RATE_LIMITED | Too many personalization requests. Please try again later. |
| 500 | PERSONALIZATION_FAILED | Personalization service encountered an error |

---

## Implementation Phases

### Phase 1: Backend API (Priority)

1. Create `/api/personalize` route file
2. Add rate limiting (reuse translate.ts pattern)
3. Implement profile fetching from DB
4. Implement profile completeness validation
5. Implement chapter content fetching from filesystem
6. Integrate Google Gemini API call
7. Build personalization prompt with user profile
8. Register route in auth/index.ts

### Phase 2: Frontend Components

9. Create `PersonalizeButton` component (mirror TranslateButton)
10. Create `PersonalizedContent` component
11. Create CSS module for styling
12. Add TypeScript types
13. Create index.ts barrel export

### Phase 3: Integration

14. Add PersonalizeButton to chapter pages
15. Wire up state management (toggle)
16. Implement caching for instant toggle
17. Add error handling and retry logic

### Phase 4: Polish & Testing

18. Responsive design testing
19. Test with different profile types
20. Verify code blocks preserved
21. All chapters verification
22. End-to-end testing

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Gemini API unavailable | Graceful error message, retry button, fallback to original |
| Large chapters timeout | Set 60s timeout, progress indicator, consider chunking |
| Profile incomplete | Validate profile completeness, show helpful error |
| Code blocks altered | Explicit system prompt rules, post-processing verification |
| Auth race condition | Check isLoading before render |

---

## Success Criteria Mapping

| Spec Criteria | Implementation |
|---------------|----------------|
| Button at chapter start | PersonalizeButton component positioned after title |
| Auth + profile required | useAuth check + requireAuth + profile validation |
| Uses software background | Prompt includes programming_languages, frameworks_platforms, experience_level |
| Uses hardware background | Prompt includes device_type, operating_system, system_capability |
| Code blocks preserved | System prompt rules + post-processing check |
| Toggle functionality | showPersonalized state toggle |
| Original accessible | Toggle back to original content |

---

## Environment Variables

Add to auth service `.env`:

```bash
# Existing
DATABASE_URL=...
BETTER_AUTH_SECRET=...
FRONTEND_URL=...

# New for personalization
GEMINI_API_KEY=<google-gemini-api-key>
CHAPTER_CONTENT_PATH=/path/to/book-write/docs
```

---

## Files to Create/Modify

### New Files

| File | Purpose |
|------|---------|
| `auth/src/routes/personalize.ts` | Personalization API endpoint |
| `book-write/src/components/Personalization/PersonalizeButton.tsx` | Button component |
| `book-write/src/components/Personalization/PersonalizedContent.tsx` | Content display |
| `book-write/src/components/Personalization/Personalization.module.css` | Styling |
| `book-write/src/components/Personalization/types.ts` | TypeScript types |
| `book-write/src/components/Personalization/index.ts` | Barrel export |

### Modified Files

| File | Change |
|------|--------|
| `auth/src/index.ts` | Register personalize route |
| `auth/.env.example` | Add GEMINI_API_KEY, CHAPTER_CONTENT_PATH |
| Chapter MDX files | Import and add PersonalizeButton |

---

## Next Steps

1. Run `/sp.tasks` to generate detailed task breakdown
2. Implement backend API first (Phase 1)
3. Test API with curl/Postman
4. Implement frontend components (Phase 2)
5. Integrate into chapters (Phase 3)
6. Test acceptance criteria
7. Submit for bonus point evaluation

---

**END OF PLAN**
