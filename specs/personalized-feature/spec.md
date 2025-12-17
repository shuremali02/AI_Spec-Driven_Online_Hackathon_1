# Feature Specification: Chapter Content Personalization (Bonus Feature)

**Feature Branch**: `personalization-feature`
**Created**: 2025-12-18
**Status**: Final
**Priority**: Bonus (50 points)
**Version**: 1.0

---

## 1. Feature Overview

### 1.1 Summary

This specification defines the chapter content personalization bonus feature for the Physical AI & Humanoid Robotics textbook. The feature enables authenticated users to personalize chapter content based on their software and hardware background data (collected at signup) by pressing a button at the start of each chapter.

### 1.2 Original Requirement

> "Participants can receive up to 50 extra bonus points if the logged user can personalise the content in the chapters by pressing a button at the start of each chapter."

### 1.3 Feature Purpose

**What "Chapter Content Personalization" Means**

Personalization adapts the chapter content to match the user's technical background:
- Users with **beginner** experience receive simpler explanations with more context
- Users with **advanced** experience receive concise, technical explanations
- Content examples are adjusted to reflect the user's known programming languages
- Hardware-specific notes are included based on the user's system capability

**Why Manual Button-Triggered Approach**

- **User Control**: Users decide when to personalize, maintaining agency over their learning experience
- **Performance**: On-demand personalization avoids unnecessary processing
- **Transparency**: Users clearly see when content is personalized vs. original
- **Reversibility**: Users can toggle back to original content at any time

**Hackathon Bonus Qualification**

This feature qualifies for 50 bonus points because:
1. It requires authentication (leverages existing Better-Auth system)
2. It uses collected user background data (software + hardware)
3. It provides a manual button trigger at chapter start
4. It personalizes content in a meaningful, user-specific way

### 1.4 Bonus Point Eligibility

**Total Available**: 50 bonus points

**Award Conditions** (ALL must be satisfied):
- User MUST be authenticated (logged in via Better-Auth)
- "Personalize Content" button MUST exist at the start of each chapter
- Button MUST be visible ONLY to authenticated users
- Personalization MUST use the user's software background data
- Personalization MUST use the user's hardware background data
- Original English content MUST remain accessible via toggle
- Personalized content MUST NOT alter factual meaning or code blocks

**Partial Implementation**: NO bonus points awarded

---

## 2. Dependencies

### 2.1 Required Existing Systems

| Dependency | Status | Location | Notes |
|------------|--------|----------|-------|
| Better-Auth Authentication | Implemented | `/auth/` | Signup/Signin required |
| User Profile Data | Implemented | Neon DB `user_profiles` table | Software & hardware background |
| `personalize-content` Skill | Exists | `.claude/skills/personalize-content/` | MUST use this skill |
| Docusaurus Frontend | Exists | `/book-write/` | Chapter pages |

### 2.2 User Profile Data Available

From `user_profiles` table (already collected at signup):

**Software Background:**
| Field | Values |
|-------|--------|
| `programming_languages` | Python, JavaScript/TypeScript, C/C++, Java, Go, Rust, Other |
| `frameworks_platforms` | ROS/ROS 2, TensorFlow, PyTorch, OpenCV, Arduino/Embedded, Web Development, Mobile Development, Other |
| `experience_level` | beginner, intermediate, advanced, expert |

**Hardware Background:**
| Field | Values |
|-------|--------|
| `device_type` | desktop, laptop, tablet, mobile, embedded |
| `operating_system` | windows, macos, linux, other |
| `system_capability` | low, medium, high |

### 2.3 Critical Constraints

1. **MUST use user's collected background data** - No generic personalization
2. **MUST require authentication** - Anonymous users cannot access personalization
3. **MUST NOT modify original content** - Source markdown files remain unchanged
4. **MUST preserve technical accuracy** - No factual changes to content

---

## 3. Access Control Requirements

### 3.1 Authentication Dependency

This feature requires the Authentication system (defined in `specs/auth-personalization/spec.md`) to be fully functional.

### 3.2 Access Control Matrix

| User State | See Personalize Button | Trigger Personalization | View Personalized Content |
|------------|------------------------|-------------------------|---------------------------|
| Anonymous (not logged in) | NO | NO | NO |
| Authenticated (logged in) | YES | YES | YES |
| Authenticated (no profile) | NO | NO | NO |

### 3.3 Session Validation

**Before rendering the personalization button:**
```
1. Check Better-Auth session state
2. If no valid session → DO NOT render personalization button
3. If valid session → Check if user has profile data
4. If no profile → DO NOT render personalization button
5. If profile exists → Render personalization button
```

**Before processing personalization request:**
```
1. Validate Better-Auth session via API
2. If invalid → Return 401 Unauthorized
3. Fetch user profile from database
4. If profile not found → Return 404 Profile Not Found
5. If profile exists → Proceed with personalization using profile data
```

### 3.4 Graceful Degradation

**When user is NOT authenticated:**
- Personalization button MUST NOT be visible
- No UI element suggesting personalization exists
- Chapter page renders normally with original English content only

**When user IS authenticated but has no profile:**
- Personalization button MUST NOT be visible
- User should complete profile setup first

**When user IS authenticated with complete profile:**
- Personalization button MUST be clearly visible
- Button positioned at chapter start
- Clear affordance for interaction

---

## 4. UI Requirements

### 4.1 Personalization Button Specification

#### 4.1.1 Button Placement

| Attribute | Value |
|-----------|-------|
| Position | Start of each chapter, AFTER the chapter title and metadata |
| Container | Dedicated component at chapter top, NEXT TO translation button if present |
| Visibility | Only for authenticated users with profile data |

#### 4.1.2 Button States

| State | Label | Appearance | Behavior |
|-------|-------|------------|----------|
| Default | "Personalize Content" | Primary button style | Clickable |
| Loading | "Personalizing..." | Disabled, spinner | Non-clickable |
| Personalized | "Show Original" | Secondary style | Toggles back to original |
| Error | "Personalization Failed - Retry" | Error style | Clickable to retry |
| Reset | "Personalize Again" | Outline style | Re-triggers personalization |

#### 4.1.3 Button Styling

| Property | Value |
|----------|-------|
| Background (default) | Secondary brand color (distinct from translate button) |
| Background (hover) | Darker shade |
| Text Color | White or contrasting |
| Font | System UI, readable |
| Padding | 12px 24px |
| Border Radius | 8px |
| Icon | Optional: User/person icon |

#### 4.1.4 Visual Distinction from Translation Button

If both personalization and translation buttons are present:
- Personalization button: Secondary/accent color
- Translation button: Primary color
- Clear visual separation between the two buttons
- Tooltip on hover explaining each button's purpose

### 4.2 Interaction Flow

#### 4.2.1 Personalization Trigger Flow

```
1. User views chapter page
2. System checks authentication state
3. System checks if user has profile data
4. If authenticated WITH profile → Display "Personalize Content" button
5. User clicks button
6. Button enters "Loading" state with "Personalizing..."
7. System fetches user profile data
8. System invokes personalize-content skill with chapter content + profile data
9. On success → Display personalized content, button changes to "Show Original"
10. On failure → Display error message, button shows "Retry"
```

#### 4.2.2 Toggle Flow

```
1. User has personalized content displayed
2. Button shows "Show Original"
3. User clicks button
4. System displays original English content
5. Button changes to "Personalize Again"
6. Previously personalized content is cached for instant toggle back
```

### 4.3 Per-Chapter Scope

- Personalization MUST be scoped to the currently opened chapter ONLY
- Personalizing Chapter 1 MUST NOT affect Chapter 2
- Each chapter manages its own personalization state independently

---

## 5. Personalization Behavior

### 5.1 What MUST Be Personalized (Based on User Profile)

#### 5.1.1 Based on Experience Level

| Experience Level | Personalization Behavior |
|------------------|-------------------------|
| `beginner` | Simpler language, more explanations, foundational context, step-by-step guidance |
| `intermediate` | Balanced explanations, assume basic knowledge, focus on practical application |
| `advanced` | Concise technical language, skip basics, focus on advanced concepts |
| `expert` | Direct technical prose, assume deep domain knowledge, reference advanced topics |

#### 5.1.2 Based on Programming Languages

| User's Languages | Personalization Behavior |
|------------------|-------------------------|
| Includes Python | Examples reference Python idioms where applicable |
| Includes C/C++ | Add notes about memory management, performance considerations |
| Includes JavaScript | Draw parallels to async/event-driven patterns where relevant |
| Includes Java | Reference OOP concepts familiar to Java developers |

#### 5.1.3 Based on Frameworks/Platforms

| User's Frameworks | Personalization Behavior |
|-------------------|-------------------------|
| Includes ROS/ROS 2 | Assume ROS familiarity, can use ROS-specific terminology freely |
| Includes TensorFlow/PyTorch | Draw parallels to ML concepts where relevant |
| Includes Arduino/Embedded | Add hardware-specific context and embedded system notes |
| Includes Web Development | Use web analogies where applicable |

#### 5.1.4 Based on Hardware Background

| Hardware Profile | Personalization Behavior |
|------------------|-------------------------|
| `system_capability: low` | Add warnings about resource-heavy operations, suggest lightweight alternatives |
| `system_capability: high` | Can mention GPU acceleration, parallel processing options |
| `device_type: embedded` | Emphasize embedded-friendly approaches, mention Jetson/RPi specifics |
| `operating_system: windows` | Add Windows-specific path notes, installation differences |
| `operating_system: linux` | Can use Linux-native examples without extra explanation |

### 5.2 What MAY Be Personalized

- **Explanation depth**: Expand or condense based on experience level
- **Wording complexity**: Beginner-friendly or technical prose
- **Relevant examples**: Contextualized to user's known technologies
- **Hardware-aware notes**: Warnings or optimizations based on system capability
- **Analogies and metaphors**: Tailored to user's familiar domains
- **Learning path suggestions**: Based on user's background

### 5.3 What MUST NOT Be Personalized

| Element | Reason |
|---------|--------|
| **Factual content** | Technical accuracy must be preserved |
| **Code blocks** | Code must remain unchanged for correctness |
| **Command-line examples** | Must work as documented |
| **API names and signatures** | Technical accuracy |
| **New topics** | Cannot introduce content not in original |
| **Chapter structure** | Headings, sections must remain intact |
| **URLs and links** | Must function correctly |
| **Technical term definitions** | Definitions must be accurate |
| **Mermaid diagrams** | Technical diagrams must remain unchanged |

### 5.4 Personalization Rules Summary

```
MUST:
- Adapt explanation depth to experience level
- Adjust wording complexity
- Add relevant context based on known languages/frameworks
- Include hardware-aware notes where applicable

MUST NOT:
- Change factual meaning of any content
- Modify code blocks or commands
- Introduce topics not in original
- Remove or reorder sections
- Change technical term definitions
- Alter diagram specifications
```

---

## 6. Personalization Logic

### 6.1 Source of Truth

- **Original English chapter content** is always the source of truth
- Personalization is a transformation of original content
- Original content is NEVER modified on disk

### 6.2 Runtime Generation

Personalized content is generated at runtime:
```
1. User clicks "Personalize Content"
2. Frontend sends request with chapter_id
3. Backend fetches:
   a. Original chapter content (from Docusaurus /docs/)
   b. User profile data (from Neon DB)
4. Backend invokes personalize-content skill with:
   - Original content
   - User's experience_level
   - User's programming_languages
   - User's frameworks_platforms
   - User's device_type
   - User's operating_system
   - User's system_capability
5. Skill returns personalized content
6. Backend returns personalized content to frontend
7. Frontend displays personalized content
```

### 6.3 Original Content Access

- User can ALWAYS toggle back to original content
- Toggle is instant (original content is already loaded)
- Button state changes to "Show Original" when personalized content is displayed
- Button state changes to "Personalize Again" when original is displayed (if previously personalized)

### 6.4 Scope Per Chapter

| Aspect | Specification |
|--------|---------------|
| Scope | Each chapter personalized independently |
| State | Personalization state is per-chapter |
| Cache | Optional: Cache personalized content per chapter per session |
| Persistence | Personalization is NOT persisted to database |

---

## 7. Data Flow

### 7.1 Request Flow

```
[User] → [Docusaurus Frontend]
           │
           │ 1. Click "Personalize Content"
           ▼
       [PersonalizeButton Component]
           │
           │ 2. POST /api/personalize
           │    Body: { chapter_id }
           │    Cookie: Better-Auth session
           ▼
       [Auth Backend /api/personalize]
           │
           │ 3. Validate session via Better-Auth
           │ 4. Fetch user profile from Neon DB
           │ 5. Fetch chapter content from file system
           ▼
       [Personalize-Content Skill Logic]
           │
           │ 6. Transform content based on profile
           ▼
       [Auth Backend]
           │
           │ 7. Return personalized content
           ▼
       [Docusaurus Frontend]
           │
           │ 8. Render personalized content
           ▼
       [User sees personalized chapter]
```

### 7.2 Data Retrieval

#### User Profile Data

```
Source: Neon PostgreSQL user_profiles table
Method: Query by auth_user_id from session
Fields used:
  - experience_level
  - programming_languages[]
  - frameworks_platforms[]
  - device_type
  - operating_system
  - system_capability
```

#### Chapter Content

```
Source: Docusaurus /book-write/docs/ directory
Method: Read markdown file by chapter_id
Format: Raw markdown with frontmatter
```

### 7.3 Output Format

The personalized content is returned as:
```json
{
  "success": true,
  "personalized_content": "<transformed markdown content>",
  "chapter_id": "module-1/chapter-01-intro-physical-ai",
  "personalization_summary": {
    "experience_level": "intermediate",
    "adjustments_made": [
      "Adjusted explanation depth for intermediate level",
      "Added Python-specific notes",
      "Included ROS 2 context where applicable"
    ]
  },
  "timestamp": "2025-12-18T10:30:00Z"
}
```

### 7.4 No Permanent Modifications

| Constraint | Specification |
|------------|---------------|
| Source files | NEVER modified by personalization |
| Database storage | Personalized content NOT stored in DB |
| Session storage | Optional client-side cache only |
| Regeneration | User can re-personalize at any time |

---

## 8. Non-Functional Requirements

### 8.1 Performance

| Metric | Target |
|--------|--------|
| Button render time | <100ms after auth and profile check |
| Personalization initiation | <500ms to show loading state |
| Personalization completion | <45 seconds for average chapter (4000 words) |
| Toggle response | <100ms (instant for cached content) |

### 8.2 Reliability

| Requirement | Specification |
|-------------|---------------|
| Success rate | >90% for well-formed chapter content |
| Error recovery | Graceful fallback to original content |
| Session handling | Tolerate session expiry during personalization |
| Profile validation | Handle missing or incomplete profile gracefully |

### 8.3 Error Handling

#### 8.3.1 Error Categories

| Error Type | Cause | User Message | Recovery Action |
|------------|-------|--------------|-----------------|
| AUTH_REQUIRED | User not authenticated | "Please sign in to personalize content" | Show sign-in link |
| PROFILE_NOT_FOUND | User has no profile | "Complete your profile to personalize" | Link to profile setup |
| PROFILE_INCOMPLETE | Missing background data | "Update your profile with background info" | Link to profile update |
| PERSONALIZATION_FAILED | Skill invocation failed | "Personalization failed. Please try again." | Retry button |
| CONTENT_TOO_LONG | Chapter exceeds limit | "Chapter is too long to personalize at once" | Suggest section-by-section |
| NETWORK_ERROR | Connection issue | "Connection error. Check your internet." | Retry button |
| TIMEOUT | Personalization took too long | "Personalization timed out. Please retry." | Retry button |

#### 8.3.2 Error Display

- Error messages MUST be user-friendly (no technical jargon)
- Error messages MUST be actionable (tell user what to do)
- Error state MUST be dismissible
- Error MUST NOT break chapter page functionality
- On error, original content remains displayed

#### 8.3.3 Graceful Fallback

If personalization fails at any point:
```
1. Display error message to user
2. Keep original content visible
3. Offer retry option
4. Do NOT leave user with blank content
```

### 8.4 Security and Privacy

| Requirement | Specification |
|-------------|---------------|
| Authentication check | Server-side validation before personalization |
| Profile data access | Only user's own profile data accessed |
| Content sanitization | Personalized content sanitized before render |
| XSS prevention | No raw HTML injection in personalized content |
| Rate limiting | Prevent personalization abuse (5 requests/minute/user) |
| Privacy | Profile data used only for personalization, not logged verbosely |

---

## 9. Bonus Point Eligibility Rules

### 9.1 Full 50 Points Requirements

To qualify for the full 50 bonus points, ALL of the following MUST be true:

| # | Requirement | Verification |
|---|-------------|--------------|
| 1 | User authentication via Better-Auth is functional | User can sign in |
| 2 | User profile with software/hardware background exists | Profile data in DB |
| 3 | "Personalize Content" button appears at chapter start | Visual inspection |
| 4 | Button is ONLY visible to authenticated users | Anonymous check |
| 5 | Clicking button triggers personalization process | Action triggers API |
| 6 | Personalization uses user's software background | Check API uses profile |
| 7 | Personalization uses user's hardware background | Check API uses profile |
| 8 | Personalized content is displayed to user | Content changes visibly |
| 9 | User can toggle back to original content | Toggle works |
| 10 | Original content is not modified | Source files unchanged |
| 11 | Code blocks and technical terms preserved | No code changes |

### 9.2 Partial Implementation = 0 Points

| Missing Component | Result |
|-------------------|--------|
| No authentication | 0 points |
| Button visible to anonymous users | 0 points |
| Personalization doesn't use profile data | 0 points |
| Only uses experience level (ignores hardware) | 0 points |
| Cannot toggle to original | 0 points |
| Code blocks are modified | 0 points |
| Factual meaning is changed | 0 points |

### 9.3 What Judges Will Verify

1. Sign in as a test user
2. Verify user has profile data (software + hardware background)
3. Navigate to any chapter
4. Confirm "Personalize Content" button is visible
5. Click button and wait for personalization
6. Verify content is different (adapted to user's background)
7. Click "Show Original" and verify original content returns
8. Sign out and verify button disappears
9. Check that code blocks remain unchanged
10. Check that factual content is preserved

---

## 10. Acceptance Criteria

### 10.1 Access Control Tests

**Test PC-AUTH-001**: Button not visible for anonymous users
- **Given**: User is NOT logged in
- **When**: User views any chapter page
- **Then**: "Personalize Content" button is NOT rendered

**Test PC-AUTH-002**: Button visible for authenticated users with profile
- **Given**: User IS logged in via Better-Auth AND has complete profile
- **When**: User views any chapter page
- **Then**: "Personalize Content" button IS visible at chapter start

**Test PC-AUTH-003**: Button not visible for authenticated users without profile
- **Given**: User IS logged in but has NO profile data
- **When**: User views any chapter page
- **Then**: "Personalize Content" button is NOT rendered

**Test PC-AUTH-004**: Personalization blocked for expired session
- **Given**: User's session has expired
- **When**: User clicks "Personalize Content"
- **Then**: 401 error returned, user prompted to sign in

### 10.2 Personalization Behavior Tests

**Test PC-PERS-001**: Successful personalization for beginner
- **Given**: Authenticated user with experience_level = "beginner"
- **When**: User clicks "Personalize Content" on Chapter 1
- **Then**: Content is adapted with simpler explanations, more context

**Test PC-PERS-002**: Successful personalization for expert
- **Given**: Authenticated user with experience_level = "expert"
- **When**: User clicks "Personalize Content" on Chapter 1
- **Then**: Content is adapted with concise, technical language

**Test PC-PERS-003**: Programming language context added
- **Given**: User has programming_languages = ["Python", "C++"]
- **When**: User personalizes a chapter
- **Then**: Content includes Python-relevant context and C++ performance notes where applicable

**Test PC-PERS-004**: Hardware capability warnings
- **Given**: User has system_capability = "low"
- **When**: User personalizes a chapter with resource-heavy topics
- **Then**: Content includes warnings about resource requirements

**Test PC-PERS-005**: Code blocks preserved
- **Given**: Chapter contains Python code blocks
- **When**: Chapter is personalized
- **Then**: Code blocks remain exactly unchanged

**Test PC-PERS-006**: Technical terms preserved
- **Given**: Chapter mentions "ROS 2", "URDF", "DDS"
- **When**: Chapter is personalized
- **Then**: Technical terms remain unchanged, definitions accurate

**Test PC-PERS-007**: Structure preserved
- **Given**: Chapter has headings, lists, tables
- **When**: Chapter is personalized
- **Then**: Document structure is maintained

**Test PC-PERS-008**: No new topics introduced
- **Given**: Original chapter does not mention topic X
- **When**: Chapter is personalized
- **Then**: Personalized chapter does not introduce topic X

### 10.3 Toggle Tests

**Test PC-TOGGLE-001**: Toggle to original
- **Given**: User has personalized content displayed
- **When**: User clicks "Show Original"
- **Then**: Original English content is displayed

**Test PC-TOGGLE-002**: Toggle back to personalized
- **Given**: User is viewing original content (after personalization)
- **When**: User clicks "Personalize Again"
- **Then**: Previously personalized content is displayed (from cache)

**Test PC-TOGGLE-003**: Independent chapter state
- **Given**: User has personalized Chapter 1
- **When**: User navigates to Chapter 2
- **Then**: Chapter 2 shows original content with "Personalize Content" button

### 10.4 Error Handling Tests

**Test PC-ERROR-001**: Personalization failure recovery
- **Given**: Personalization skill fails
- **When**: Error occurs
- **Then**: User sees friendly error message with retry option, original content remains

**Test PC-ERROR-002**: Network error handling
- **Given**: Network connection lost during personalization
- **When**: Request fails
- **Then**: User sees connection error with retry option

**Test PC-ERROR-003**: Incomplete profile handling
- **Given**: User's profile is missing experience_level
- **When**: User tries to personalize
- **Then**: Error message prompts user to complete profile

---

## 11. API Contract

### 11.1 Personalization Endpoint

#### Request

```
POST /api/personalize
Content-Type: application/json
Cookie: Better-Auth session cookie

{
  "chapter_id": "module-1/chapter-01-intro-physical-ai"
}
```

Note: Chapter content is fetched server-side by chapter_id, NOT sent by client.

#### Response (Success)

```
HTTP/1.1 200 OK
Content-Type: application/json

{
  "success": true,
  "personalized_content": "<personalized markdown content>",
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

#### Response (Error - Unauthorized)

```
HTTP/1.1 401 Unauthorized
Content-Type: application/json

{
  "success": false,
  "error": "AUTH_REQUIRED",
  "message": "Authentication required to personalize content"
}
```

#### Response (Error - Profile Not Found)

```
HTTP/1.1 404 Not Found
Content-Type: application/json

{
  "success": false,
  "error": "PROFILE_NOT_FOUND",
  "message": "Please complete your profile to personalize content"
}
```

#### Response (Error - Personalization Failed)

```
HTTP/1.1 500 Internal Server Error
Content-Type: application/json

{
  "success": false,
  "error": "PERSONALIZATION_FAILED",
  "message": "Personalization service encountered an error",
  "retry_after": 5
}
```

### 11.2 Endpoint Placement

| Option | Location | Notes |
|--------|----------|-------|
| Recommended | `/auth/src/routes/personalize.ts` | Within existing auth backend |

Recommendation: Add to existing auth backend to reuse session validation and profile access.

---

## 12. Frontend Component Specification

### 12.1 Component: PersonalizeButton

#### Location

```
/book-write/src/components/Personalization/PersonalizeButton.tsx
```

#### Props

```typescript
interface PersonalizeButtonProps {
  chapterId: string;           // Unique chapter identifier
  isAuthenticated: boolean;    // From auth context
  hasProfile: boolean;         // User has complete profile
}
```

#### Behavior

- Render only if `isAuthenticated === true` AND `hasProfile === true`
- On click: call personalization API with chapter_id
- Manage loading/success/error states
- Toggle between original and personalized content

### 12.2 Component: PersonalizedContent

#### Location

```
/book-write/src/components/Personalization/PersonalizedContent.tsx
```

#### Props

```typescript
interface PersonalizedContentProps {
  content: string;                    // Personalized markdown content
  personalizationSummary?: {
    experience_level: string;
    adjustments_made: string[];
  };
  className?: string;
}
```

#### Behavior

- Display personalized content with visual indicator
- Show personalization summary badge (optional)
- Maintain same styling as original content
- Code blocks render identically to original

### 12.3 Integration with Chapter Template

```
ChapterPage
├── ChapterHeader
│   ├── Title
│   ├── Metadata
│   ├── TranslateButton (if authenticated)
│   └── PersonalizeButton (if authenticated + has profile)
├── ChapterContent
│   ├── OriginalContent (default)
│   └── PersonalizedContent (if personalized)
└── ChapterFooter
```

---

## 13. State Management

### 13.1 Personalization State Per Chapter

```typescript
interface PersonalizationState {
  isAuthenticated: boolean;           // From Better-Auth
  hasProfile: boolean;                // User has profile data
  personalizationState: 'idle' | 'loading' | 'success' | 'error';
  showPersonalized: boolean;          // Toggle between original/personalized
  personalizedContent: string | null; // Cached personalized content
  personalizationSummary: object | null;
  errorMessage: string | null;
}
```

### 13.2 State Transitions

```
idle → loading       (User clicks "Personalize Content")
loading → success    (API returns personalized content)
loading → error      (API fails or times out)
success → idle       (User clicks "Show Original", keep cache)
idle → success       (User clicks "Personalize Again", use cache)
error → loading      (User clicks "Retry")
```

### 13.3 Caching

| Behavior | Specification |
|----------|---------------|
| Cache scope | Per chapter, per session |
| Cache storage | Component state (React useState) |
| Cache invalidation | On page navigation away OR session change |
| Benefit | Instant toggle between original and personalized |

---

## 14. Out of Scope

The following are **explicitly NOT included** in this specification:

### Personalization Features
- Bulk personalization of multiple chapters at once
- Persistent storage of personalized content in database
- Personalization preferences/settings page
- User-editable personalization rules
- A/B testing of personalization strategies

### Content Features
- PDF export of personalized content
- Personalized recommendations for next chapters
- Adaptive learning paths
- Quiz/assessment personalization

### System Features
- Admin panel for personalization analytics
- Personalization quality metrics dashboard
- Machine learning model for personalization
- Crowd-sourced personalization feedback

---

## 15. Implementation Order

1. Create Personalization components directory structure
2. Implement profile check logic in auth context
3. Implement `PersonalizeButton` component with auth + profile check
4. Implement `PersonalizedContent` component
5. Add personalization API endpoint in auth backend (`/api/personalize`)
6. Implement personalize-content skill invocation logic
7. Add component to chapter template (conditional on auth + profile)
8. Implement state management for personalization toggle
9. Add caching for personalized content
10. Add error handling and retry logic
11. Integration testing across chapters
12. End-to-end testing with authentication and profile flow

---

## 16. Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-18 | Claude Code | Initial specification |

---

**END OF SPECIFICATION**
