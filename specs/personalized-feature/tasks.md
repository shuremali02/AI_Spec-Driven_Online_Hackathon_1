# Task Breakdown: Chapter Content Personalization

**Feature**: personalized-feature | **Date**: 2025-12-18
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

---

## Task Overview

| Phase | Tasks | Est. Effort |
|-------|-------|-------------|
| Phase 1: Backend API | 8 tasks | High |
| Phase 2: Frontend Components | 6 tasks | Medium |
| Phase 3: Integration | 4 tasks | Medium |
| Phase 4: Testing & Polish | 4 tasks | Low |
| **Total** | **22 tasks** | - |

---

## Phase 1: Backend API

### TASK-001: Create personalize route file structure

**Priority**: P0 (Blocker)
**Estimate**: 15 min
**Dependencies**: None

**Description**:
Create the base route file for the personalization API endpoint.

**File**: `auth/src/routes/personalize.ts`

**Acceptance Criteria**:
- [ ] File created at `auth/src/routes/personalize.ts`
- [ ] Basic Hono router setup
- [ ] Export default router

**Test Cases**:
```
Given: Route file exists
When: Imported in index.ts
Then: No TypeScript errors
```

---

### TASK-002: Implement rate limiting for personalization

**Priority**: P0 (Blocker)
**Estimate**: 20 min
**Dependencies**: TASK-001

**Description**:
Add rate limiting to prevent API abuse. Limit: 5 requests per minute per user.

**Reference**: `auth/src/routes/translate.ts` (lines 6-32)

**Acceptance Criteria**:
- [ ] Rate limit store created (in-memory Map)
- [ ] `checkRateLimit(userId)` function implemented
- [ ] Returns `{ allowed: boolean, retryAfter?: number }`
- [ ] Limit: 5 requests/minute/user

**Test Cases**:
```
Given: User has made 5 requests in 1 minute
When: User makes 6th request
Then: Returns { allowed: false, retryAfter: <seconds> }
```

---

### TASK-003: Implement profile fetching from database

**Priority**: P0 (Blocker)
**Estimate**: 25 min
**Dependencies**: TASK-001

**Description**:
Fetch user profile from Neon DB using auth_user_id from session.

**Reference**: `auth/src/routes/profile.ts` (lines 82-122)

**Acceptance Criteria**:
- [ ] Query `user_profiles` table by `auth_user_id`
- [ ] Return full profile object with software_background and hardware_background
- [ ] Handle profile not found case (return null)

**Test Cases**:
```
Given: User has profile in database
When: fetchUserProfile(authUserId) called
Then: Returns complete profile object

Given: User has no profile
When: fetchUserProfile(authUserId) called
Then: Returns null
```

---

### TASK-004: Implement profile completeness validation

**Priority**: P0 (Blocker)
**Estimate**: 15 min
**Dependencies**: TASK-003

**Description**:
Validate that all 6 required profile fields are present.

**Required Fields**:
- `experience_level`
- `programming_languages[]` (non-empty)
- `frameworks_platforms[]` (non-empty)
- `device_type`
- `operating_system`
- `system_capability`

**Acceptance Criteria**:
- [ ] `validateProfileCompleteness(profile)` function implemented
- [ ] Returns `{ valid: boolean, missingFields: string[] }`
- [ ] Checks all 6 fields

**Test Cases**:
```
Given: Profile with all fields populated
When: validateProfileCompleteness(profile) called
Then: Returns { valid: true, missingFields: [] }

Given: Profile missing experience_level
When: validateProfileCompleteness(profile) called
Then: Returns { valid: false, missingFields: ['experience_level'] }
```

---

### TASK-005: Implement chapter content fetching

**Priority**: P0 (Blocker)
**Estimate**: 30 min
**Dependencies**: TASK-001

**Description**:
Fetch chapter markdown content from filesystem based on chapter_id.

**Path Pattern**: `{CHAPTER_CONTENT_PATH}/{chapter_id}.md` or `{CHAPTER_CONTENT_PATH}/{chapter_id}/index.md`

**Acceptance Criteria**:
- [ ] `fetchChapterContent(chapterId)` function implemented
- [ ] Reads from `CHAPTER_CONTENT_PATH` env variable
- [ ] Handles both `chapter-01.md` and `chapter-01/index.md` patterns
- [ ] Returns markdown content string
- [ ] Handles chapter not found (returns null)
- [ ] Strips frontmatter from content

**Test Cases**:
```
Given: Chapter file exists at path
When: fetchChapterContent('module-1/chapter-01') called
Then: Returns markdown content without frontmatter

Given: Chapter file does not exist
When: fetchChapterContent('invalid-chapter') called
Then: Returns null
```

---

### TASK-006: Integrate Google Gemini API

**Priority**: P0 (Blocker)
**Estimate**: 45 min
**Dependencies**: TASK-001

**Description**:
Integrate Google Gemini API for content personalization.

**Model**: `gemini-2.0-flash-exp` (or `gemini-1.5-flash`)
**API**: Google Generative AI SDK

**Acceptance Criteria**:
- [ ] Install `@google/generative-ai` package
- [ ] `personalizeWithGemini(content, profile)` function implemented
- [ ] System prompt includes MUST/MUST NOT rules from spec
- [ ] User prompt includes all 6 profile fields
- [ ] Returns personalized markdown content
- [ ] Handles API errors gracefully

**System Prompt** (from plan.md):
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

**Test Cases**:
```
Given: Valid content and profile
When: personalizeWithGemini(content, profile) called
Then: Returns personalized markdown

Given: Gemini API fails
When: personalizeWithGemini(content, profile) called
Then: Throws error with descriptive message
```

---

### TASK-007: Build POST /api/personalize endpoint

**Priority**: P0 (Blocker)
**Estimate**: 45 min
**Dependencies**: TASK-002, TASK-003, TASK-004, TASK-005, TASK-006

**Description**:
Assemble the complete personalization endpoint combining all components.

**Endpoint Flow**:
1. Apply `requireAuth` middleware
2. Check rate limit
3. Validate chapter_id in request body
4. Fetch user profile
5. Validate profile completeness
6. Fetch chapter content
7. Call Gemini API for personalization
8. Build personalization summary
9. Return response

**Acceptance Criteria**:
- [ ] `POST /api/personalize` endpoint implemented
- [ ] Uses `requireAuth` middleware
- [ ] Validates `chapter_id` in request body
- [ ] Returns 401 if not authenticated
- [ ] Returns 404 if profile not found
- [ ] Returns 400 if profile incomplete
- [ ] Returns 404 if chapter not found
- [ ] Returns 429 if rate limited
- [ ] Returns 500 if personalization fails
- [ ] Returns 200 with personalized content on success

**Response Format** (success):
```json
{
  "success": true,
  "personalized_content": "<markdown>",
  "chapter_id": "module-1/chapter-01",
  "personalization_summary": {
    "experience_level": "intermediate",
    "programming_context": ["Python"],
    "hardware_context": {
      "system_capability": "medium",
      "operating_system": "linux"
    },
    "adjustments_made": ["..."]
  },
  "timestamp": "2025-12-18T10:30:00Z"
}
```

**Test Cases**:
```
Given: Authenticated user with complete profile
When: POST /api/personalize with valid chapter_id
Then: Returns 200 with personalized content

Given: Unauthenticated user
When: POST /api/personalize
Then: Returns 401 AUTH_REQUIRED

Given: User without profile
When: POST /api/personalize
Then: Returns 404 PROFILE_NOT_FOUND

Given: User with incomplete profile
When: POST /api/personalize
Then: Returns 400 PROFILE_INCOMPLETE
```

---

### TASK-008: Register personalize route in index.ts

**Priority**: P0 (Blocker)
**Estimate**: 10 min
**Dependencies**: TASK-007

**Description**:
Register the personalize route in the auth service entry point.

**File**: `auth/src/index.ts`

**Changes**:
```typescript
import personalizeRoutes from './routes/personalize.js';
// ...
app.route('/api', personalizeRoutes);
```

**Acceptance Criteria**:
- [ ] Import added for personalize routes
- [ ] Route registered with `app.route('/api', personalizeRoutes)`
- [ ] `/api/personalize` endpoint accessible
- [ ] Root endpoint updated to list personalize endpoint

**Test Cases**:
```
Given: Auth service running
When: GET / called
Then: Response includes personalize endpoint in list

Given: Auth service running
When: POST /api/personalize called
Then: Route is handled (not 404)
```

---

## Phase 2: Frontend Components

### TASK-009: Create TypeScript types for personalization

**Priority**: P1
**Estimate**: 20 min
**Dependencies**: None

**Description**:
Create TypeScript type definitions for personalization components.

**File**: `book-write/src/components/Personalization/types.ts`

**Types to Define**:
- `PersonalizationState`
- `PersonalizeButtonProps`
- `PersonalizedContentProps`
- `PersonalizeResponse`
- `PersonalizationErrorType`
- Constants: `BUTTON_LABELS`, `ERROR_MESSAGES`, `PERSONALIZATION_CONFIG`

**Reference**: `book-write/src/components/Translation/types.ts`

**Acceptance Criteria**:
- [ ] All types exported
- [ ] Matches API response format
- [ ] Constants for button labels and error messages

---

### TASK-010: Create CSS module for personalization styling

**Priority**: P1
**Estimate**: 25 min
**Dependencies**: None

**Description**:
Create CSS module with button states and personalized content styling.

**File**: `book-write/src/components/Personalization/Personalization.module.css`

**Styles Needed**:
- `.personalizeButton` - base button style (accent color, distinct from translate)
- `.buttonIdle`, `.buttonLoading`, `.buttonSuccess`, `.buttonError` - state variants
- `.loadingSpinner` - animation
- `.errorMessage` - error display
- `.personalizedContent` - wrapper with visual indicator
- `.personalizationBadge` - "Personalized for you" badge

**Reference**: `book-write/src/components/Translation/Translation.module.css`

**Acceptance Criteria**:
- [ ] Button states styled (idle, loading, success, error)
- [ ] Distinct color from translate button (accent vs primary)
- [ ] Loading spinner animation
- [ ] Personalization badge styling
- [ ] Responsive design

---

### TASK-011: Implement PersonalizeButton component

**Priority**: P0 (Blocker)
**Estimate**: 60 min
**Dependencies**: TASK-009, TASK-010

**Description**:
Create the PersonalizeButton React component with full state management.

**File**: `book-write/src/components/Personalization/PersonalizeButton.tsx`

**Component Requirements**:
- Only render if `user` AND `profile` exist (from `useAuth`)
- Validate profile completeness before rendering
- State machine: idle → loading → success/error
- Toggle between original and personalized
- Cache personalized content for instant toggle
- Error handling with retry

**Props**:
```typescript
interface PersonalizeButtonProps {
  chapterId: string;
  chapterContent: string;
  onPersonalizationComplete?: (content: string) => void;
  onToggle?: (showPersonalized: boolean) => void;
  className?: string;
}
```

**Reference**: `book-write/src/components/Translation/TranslateButton.tsx`

**Acceptance Criteria**:
- [ ] Does not render for unauthenticated users
- [ ] Does not render for users without profile
- [ ] Does not render for users with incomplete profile
- [ ] Shows "Personalize Content" initially
- [ ] Shows loading spinner during API call
- [ ] Shows "Show Original" after successful personalization
- [ ] Shows error with retry option on failure
- [ ] Toggles between original and personalized
- [ ] Caches personalized content

**Test Cases**:
```
Given: User not authenticated
When: Component renders
Then: Returns null (not visible)

Given: User authenticated without profile
When: Component renders
Then: Returns null (not visible)

Given: User with complete profile clicks button
When: API returns success
Then: Shows personalized content, button shows "Show Original"
```

---

### TASK-012: Implement PersonalizedContent component

**Priority**: P1
**Estimate**: 30 min
**Dependencies**: TASK-009, TASK-010

**Description**:
Create component to display personalized markdown content with visual indicator.

**File**: `book-write/src/components/Personalization/PersonalizedContent.tsx`

**Component Requirements**:
- Render markdown content
- Show "Personalized for you" badge
- Optionally show personalization summary
- Match original content styling

**Props**:
```typescript
interface PersonalizedContentProps {
  content: string;
  personalizationSummary?: {
    experience_level: string;
    adjustments_made: string[];
  };
  className?: string;
}
```

**Acceptance Criteria**:
- [ ] Renders markdown content correctly
- [ ] Shows personalization badge at top
- [ ] Optional summary expansion
- [ ] Matches chapter content styling

---

### TASK-013: Implement hasCompleteProfile utility

**Priority**: P0 (Blocker)
**Estimate**: 15 min
**Dependencies**: TASK-009

**Description**:
Create utility function to check if user profile has all required fields.

**File**: `book-write/src/components/Personalization/utils.ts`

**Function**:
```typescript
export function hasCompleteProfile(profile: ProfileResponse | null): boolean {
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
}
```

**Acceptance Criteria**:
- [ ] Returns false for null profile
- [ ] Returns false if any field missing
- [ ] Returns false if arrays empty
- [ ] Returns true only when all 6 fields present

---

### TASK-014: Create barrel export index.ts

**Priority**: P2
**Estimate**: 5 min
**Dependencies**: TASK-011, TASK-012, TASK-013

**Description**:
Create index.ts to export all personalization components.

**File**: `book-write/src/components/Personalization/index.ts`

**Exports**:
```typescript
export { PersonalizeButton } from './PersonalizeButton';
export { PersonalizedContent } from './PersonalizedContent';
export { hasCompleteProfile } from './utils';
export * from './types';
```

**Acceptance Criteria**:
- [ ] All components exported
- [ ] Types exported
- [ ] Utility functions exported

---

## Phase 3: Integration

### TASK-015: Add GEMINI_API_KEY to environment

**Priority**: P0 (Blocker)
**Estimate**: 10 min
**Dependencies**: TASK-006

**Description**:
Add required environment variables for personalization.

**Files**:
- `auth/.env` (add actual values)
- `auth/.env.example` (add placeholders)

**Variables**:
```bash
GEMINI_API_KEY=<your-gemini-api-key>
CHAPTER_CONTENT_PATH=../book-write/docs
```

**Acceptance Criteria**:
- [ ] GEMINI_API_KEY added to .env
- [ ] CHAPTER_CONTENT_PATH added to .env
- [ ] .env.example updated with placeholders
- [ ] API key works with Gemini API

---

### TASK-016: Integrate PersonalizeButton into chapter wrapper

**Priority**: P0 (Blocker)
**Estimate**: 45 min
**Dependencies**: TASK-014

**Description**:
Add PersonalizeButton to chapter pages alongside TranslateButton.

**Approach Options**:
1. **Option A**: MDX import per chapter (simple, explicit)
2. **Option B**: Swizzle DocItem layout (DRY, automatic)

**Recommended**: Start with Option A for MVP

**Example MDX Integration**:
```mdx
import { TranslateButton } from '@site/src/components/Translation';
import { PersonalizeButton } from '@site/src/components/Personalization';

<div className="chapter-actions">
  <TranslateButton chapterId="chapter-01" chapterContent={/* content */} />
  <PersonalizeButton chapterId="chapter-01" chapterContent={/* content */} />
</div>
```

**Acceptance Criteria**:
- [ ] PersonalizeButton visible on at least 1 chapter
- [ ] Button positioned next to TranslateButton
- [ ] Correct chapterId passed
- [ ] Works for authenticated users with profile

---

### TASK-017: Implement content toggle state management

**Priority**: P1
**Estimate**: 30 min
**Dependencies**: TASK-011, TASK-016

**Description**:
Wire up state to toggle between original and personalized content display.

**Implementation**:
- PersonalizeButton manages showPersonalized state
- When true, hide original MDX content, show PersonalizedContent
- When false, show original MDX content

**Acceptance Criteria**:
- [ ] Original content hidden when personalized shown
- [ ] Personalized content hidden when original shown
- [ ] Toggle is instant (cached content)
- [ ] Content state independent per chapter

---

### TASK-018: Add error handling and retry logic

**Priority**: P1
**Estimate**: 20 min
**Dependencies**: TASK-011

**Description**:
Implement user-friendly error handling with retry capability.

**Error Types**:
- AUTH_REQUIRED → Show sign-in link
- PROFILE_NOT_FOUND → Show profile setup link
- PROFILE_INCOMPLETE → Show profile update link
- PERSONALIZATION_FAILED → Show retry button
- NETWORK_ERROR → Show retry button
- TIMEOUT → Show retry button

**Acceptance Criteria**:
- [ ] Error messages are user-friendly
- [ ] Retry button works for transient errors
- [ ] Original content remains visible on error
- [ ] Error is dismissible

---

## Phase 4: Testing & Polish

### TASK-019: Test with different profile types

**Priority**: P1
**Estimate**: 30 min
**Dependencies**: TASK-016

**Description**:
Test personalization output with different profile configurations.

**Test Profiles**:
1. Beginner + Python + ROS 2 + Low capability
2. Expert + C++ + Arduino + High capability
3. Intermediate + JavaScript + Web Dev + Medium capability

**Acceptance Criteria**:
- [ ] Beginner profile gets simpler explanations
- [ ] Expert profile gets concise technical content
- [ ] Programming language context is added
- [ ] Hardware capability warnings appear for low systems

---

### TASK-020: Verify code blocks preserved

**Priority**: P0 (Blocker)
**Estimate**: 20 min
**Dependencies**: TASK-019

**Description**:
Verify that code blocks are not modified during personalization.

**Test Cases**:
- Python code blocks
- Bash/shell commands
- YAML configuration
- Mermaid diagrams

**Acceptance Criteria**:
- [ ] All code blocks identical before/after personalization
- [ ] Inline code preserved
- [ ] Command-line examples unchanged
- [ ] Mermaid diagrams unchanged

---

### TASK-021: Test anonymous user experience

**Priority**: P0 (Blocker)
**Estimate**: 15 min
**Dependencies**: TASK-016

**Description**:
Verify that anonymous users do not see personalization UI.

**Acceptance Criteria**:
- [ ] PersonalizeButton not visible when logged out
- [ ] No console errors for anonymous users
- [ ] Chapter content displays normally
- [ ] No trace of personalization feature in DOM

---

### TASK-022: End-to-end testing

**Priority**: P1
**Estimate**: 45 min
**Dependencies**: All previous tasks

**Description**:
Complete end-to-end test of the personalization flow.

**Test Scenario**:
1. Sign up with profile data
2. Sign in
3. Navigate to chapter
4. Verify PersonalizeButton visible
5. Click "Personalize Content"
6. Wait for personalization
7. Verify personalized content displayed
8. Click "Show Original"
9. Verify original content displayed
10. Sign out
11. Verify button not visible

**Acceptance Criteria**:
- [ ] Full flow works without errors
- [ ] Toggle between original/personalized works
- [ ] Anonymous user sees no button
- [ ] Multiple chapters work independently

---

## Task Dependencies Graph

```
Phase 1 (Backend):
TASK-001 ──┬── TASK-002 ──┐
           ├── TASK-003 ──┼── TASK-007 ── TASK-008
           ├── TASK-004 ──┤
           ├── TASK-005 ──┤
           └── TASK-006 ──┘

Phase 2 (Frontend):
TASK-009 ──┬── TASK-011 ──┬── TASK-014
           │              │
TASK-010 ──┤              │
           │              │
TASK-013 ──┘              │
                          │
TASK-012 ─────────────────┘

Phase 3 (Integration):
TASK-015 ─── TASK-016 ─── TASK-017 ─── TASK-018

Phase 4 (Testing):
TASK-019 ─── TASK-020
TASK-021
TASK-022 (depends on all)
```

---

## Critical Path

The minimum tasks required to achieve a working demo:

1. **TASK-001**: Create route file
2. **TASK-003**: Profile fetching
3. **TASK-004**: Profile validation
4. **TASK-005**: Chapter content fetching
5. **TASK-006**: Gemini integration
6. **TASK-007**: POST endpoint
7. **TASK-008**: Register route
8. **TASK-009**: TypeScript types
9. **TASK-010**: CSS styling
10. **TASK-011**: PersonalizeButton
11. **TASK-015**: Environment variables
12. **TASK-016**: Chapter integration

**Critical Path**: 12 tasks must complete for MVP

---

## Bonus Point Verification Checklist

From spec Section 9.1, judges will verify:

| # | Requirement | Task |
|---|-------------|------|
| 1 | User authentication functional | Pre-existing |
| 2 | User profile exists | Pre-existing |
| 3 | Button at chapter start | TASK-016 |
| 4 | Button ONLY for authenticated | TASK-011, TASK-021 |
| 5 | Clicking triggers API | TASK-011 |
| 6 | Uses software background | TASK-006, TASK-007 |
| 7 | Uses hardware background | TASK-006, TASK-007 |
| 8 | Content changes visibly | TASK-019 |
| 9 | Toggle to original | TASK-017 |
| 10 | Source files unchanged | By design |
| 11 | Code blocks preserved | TASK-020 |

---

**END OF TASKS**
