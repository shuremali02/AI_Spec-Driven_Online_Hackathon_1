# Feature Specification: Chapter Content Translation to Urdu (Bonus Feature)

**Feature Branch**: `translation-feature`
**Created**: 2025-12-17
**Status**: Final
**Priority**: Bonus (50 points)
**Version**: 1.0

---

## 1. Feature Overview

### 1.1 Summary

This specification defines the chapter content translation bonus feature for the Physical AI & Humanoid Robotics textbook. The feature enables authenticated users to translate chapter content from English to Urdu using the existing "Translate to Urdu" skill.

### 1.2 Original Requirement

> "Participants can receive up to 50 extra bonus points if the logged user can translate the content in Urdu in the chapters by pressing a button at the start of each chapter."

### 1.3 Bonus Point Eligibility

**Total Available**: 50 bonus points

**Award Conditions** (ALL must be satisfied):
- User MUST be authenticated (logged in via Better-Auth)
- Translation button MUST exist at the start of each chapter
- Button MUST trigger translation using the existing `translate-to-urdu` skill
- Urdu content MUST be properly displayed with correct RTL styling
- Original English content MUST remain accessible

**Partial Implementation**: NO bonus points awarded

---

## 2. Dependencies

### 2.1 Required Existing Systems

| Dependency | Status | Location | Notes |
|------------|--------|----------|-------|
| Better-Auth Authentication | Implemented | `/auth/` | Signup/Signin required |
| `translate-to-urdu` Skill | Exists | `.claude/skills/translate-to-urdu/` | MUST use this skill |
| Docusaurus Frontend | Exists | `/book-write/` | Chapter pages |

### 2.2 Critical Constraints

1. **MUST use existing `translate-to-urdu` skill** - No alternative translation mechanisms allowed
2. **MUST require authentication** - Anonymous users cannot access translation
3. **MUST NOT modify original content** - English content remains unchanged

---

## 3. Access Control Requirements

### 3.1 Authentication Dependency

This feature requires the Authentication system (defined in `specs/auth-personalization/spec.md`) to be fully functional.

### 3.2 Access Control Matrix

| User State | See Translation Button | Trigger Translation | View Urdu Content |
|------------|------------------------|---------------------|-------------------|
| Anonymous (not logged in) | NO | NO | NO |
| Authenticated (logged in) | YES | YES | YES |

### 3.3 Session Validation

Before rendering the translation button:
```
1. Check Better-Auth session state
2. If no valid session → DO NOT render translation button
3. If valid session → Render translation button
```

Before processing translation request:
```
1. Validate Better-Auth session via API
2. If invalid → Return 401 Unauthorized
3. If valid → Proceed with translation
```

### 3.4 Graceful Degradation

- When user is NOT authenticated:
  - Translation button MUST NOT be visible
  - No UI element suggesting translation exists
  - Chapter page renders normally with English content only

- When user IS authenticated:
  - Translation button MUST be clearly visible
  - Button positioned at chapter start
  - Clear affordance for interaction

---

## 4. UI & Interaction Requirements

### 4.1 Translation Button Specification

#### 4.1.1 Button Placement

| Attribute | Value |
|-----------|-------|
| Position | Start of each chapter, AFTER the chapter title and metadata |
| Container | Dedicated component at chapter top |
| Visibility | Only for authenticated users |

#### 4.1.2 Button States

| State | Label | Appearance | Behavior |
|-------|-------|------------|----------|
| Default | "Translate to Urdu" or "اردو میں ترجمہ کریں" | Primary button style | Clickable |
| Loading | "Translating..." | Disabled, spinner | Non-clickable |
| Translated | "Show Original" | Secondary style | Toggles back |
| Error | "Translation Failed - Retry" | Error style | Clickable |

#### 4.1.3 Button Styling

```
| Property | Value |
|----------|-------|
| Background (default) | Primary brand color |
| Background (hover) | Darker shade |
| Text Color | White |
| Font | System UI, readable |
| Padding | 12px 24px |
| Border Radius | 8px |
| Icon | Optional: Urdu script icon (ا) |
```

### 4.2 Interaction Flow

#### 4.2.1 Translation Trigger Flow

```
1. User views chapter page
2. System checks authentication state
3. If authenticated → Display "Translate to Urdu" button
4. User clicks button
5. Button enters "Loading" state
6. System invokes translate-to-urdu skill with chapter content
7. On success → Display translated content, button changes to "Show Original"
8. On failure → Display error message, button shows "Retry"
```

#### 4.2.2 Toggle Flow

```
1. User has translated content displayed
2. Button shows "Show Original" / "اصل مواد دکھائیں"
3. User clicks button
4. System displays original English content
5. Button reverts to "Translate to Urdu"
6. Previously translated content is cached (optional optimization)
```

### 4.3 Per-Chapter Scope

- Translation MUST be scoped to the currently opened chapter ONLY
- Translating Chapter 1 MUST NOT affect Chapter 2
- Each chapter manages its own translation state independently

---

## 5. Translation Logic Requirements

### 5.1 Skill Integration

The translation MUST follow the rules defined in the existing `translate-to-urdu` skill located at `.claude/skills/translate-to-urdu/SKILL.md`.

**Implementation Note**: Since Claude Code skills are agent-side and cannot be directly invoked via HTTP API, the backend `/api/translate` endpoint implements the skill's translation rules programmatically. The backend parses markdown, preserves code blocks and technical terms, and applies translation logic consistent with the skill's behavior.

#### 5.1.1 Translation Input/Output

```
Input:
  - Full chapter content (markdown)

Output:
  - Translated Urdu text
  - Same structure as input
  - Code blocks unchanged
  - Technical terms preserved
```

#### 5.1.2 Content Processing Rules

| Content Type | Translation Behavior |
|--------------|---------------------|
| Headings (H1-H6) | Translate text, preserve markdown syntax |
| Body paragraphs | Fully translate to Urdu |
| Bullet points / Lists | Translate text, preserve list structure |
| Bold / Italic | Translate text, preserve formatting |
| Code blocks | DO NOT translate - keep original |
| Inline code | DO NOT translate - keep original |
| Technical terms (ROS 2, URDF, Python, etc.) | DO NOT translate |
| URLs / Links | DO NOT translate |
| Image captions | Translate to Urdu |
| Callouts / Admonitions | Translate content, preserve Docusaurus syntax |

### 5.2 Prohibited Translation Mechanisms

The following are EXPLICITLY PROHIBITED:
- Google Translate API
- DeepL API
- Any external translation service
- Custom translation logic
- Machine translation models other than the skill

### 5.3 Translation Accuracy Requirements

- **Meaning Preservation**: Translated content MUST convey the same meaning as original
- **Structure Preservation**: Document structure (headings, lists, sections) MUST remain intact
- **Technical Accuracy**: Technical concepts MUST NOT be misrepresented
- **Fluency**: Urdu text MUST be natural and readable

---

## 6. Urdu Content Display Requirements

### 6.1 Text Rendering

#### 6.1.1 RTL (Right-to-Left) Support

| Property | Value |
|----------|-------|
| Direction | `rtl` |
| Text Alignment | Right-aligned for Urdu text |
| Mixed Content | Bidirectional handling for embedded English/code |

#### 6.1.2 CSS Requirements

```css
/* Urdu content container */
.urdu-content {
  direction: rtl;
  text-align: right;
  font-family: "Noto Nastaliq Urdu", "Jameel Noori Nastaleeq", "Nastaliq", serif;
  font-size: 1.125rem; /* Slightly larger for readability */
  line-height: 2; /* Generous line spacing for Urdu script */
}

/* Code blocks within Urdu content remain LTR */
.urdu-content pre,
.urdu-content code {
  direction: ltr;
  text-align: left;
  font-family: monospace;
}

/* Technical terms in English within Urdu text */
.urdu-content .technical-term {
  direction: ltr;
  display: inline-block;
}
```

### 6.2 Typography Requirements

| Property | Value | Rationale |
|----------|-------|-----------|
| Font Family | Noto Nastaliq Urdu (primary), fallback: system Nastaliq | Standard Urdu display font |
| Font Size | 18px (1.125rem) | Urdu script needs larger size for clarity |
| Line Height | 2.0 | Urdu script has taller ascenders/descenders |
| Letter Spacing | 0 | Nastaliq joins letters naturally |
| Word Spacing | Normal | Standard spacing |

### 6.3 Visual Distinction

Translated content MUST be visually distinguishable from original:

| Element | Specification |
|---------|---------------|
| Background | Subtle different shade (e.g., light cream) |
| Border | Optional left border indicator |
| Badge | "اردو ترجمہ" (Urdu Translation) label at top |
| Icon | Optional Urdu script indicator |

### 6.4 Responsive Design

| Viewport | Behavior |
|----------|----------|
| Desktop (>1024px) | Full-width Urdu content |
| Tablet (768-1024px) | Adjusted padding, same font size |
| Mobile (<768px) | Full-width, touch-friendly button, readable font |

---

## 7. Content Scope

### 7.1 What Gets Translated

| Element | Translated |
|---------|------------|
| Chapter title | YES |
| Chapter introduction | YES |
| Section headings | YES |
| Body text paragraphs | YES |
| List items | YES |
| Table cell content | YES |
| Image alt text | YES |
| Admonition content | YES |

### 7.2 What Does NOT Get Translated

| Element | Reason |
|---------|--------|
| Code blocks | Technical accuracy |
| Inline code | Technical accuracy |
| URLs | Functionality |
| File paths | Technical accuracy |
| Command-line examples | Technical accuracy |
| API names | Technical accuracy |
| Technical terms (ROS 2, URDF, DDS, etc.) | Domain terminology |
| Mermaid diagram code | Technical syntax |

### 7.3 Original Content Preservation

| Requirement | Specification |
|-------------|---------------|
| Original storage | Original markdown MUST remain in Docusaurus `/docs/` |
| No modification | Translation MUST NOT alter source files |
| Toggle access | User can switch between original and translated at any time |

---

## 8. State Management

### 8.1 Translation State Per Chapter

Each chapter page maintains independent state:

```
State: {
  isAuthenticated: boolean       // From Better-Auth
  translationState: 'idle' | 'loading' | 'success' | 'error'
  showTranslated: boolean        // Toggle between original/translated
  translatedContent: string | null  // Cached translated content
  errorMessage: string | null    // Error details if failed
}
```

### 8.2 State Transitions

```
idle → loading    (User clicks "Translate to Urdu")
loading → success (Skill returns translated content)
loading → error   (Skill fails or times out)
success → idle    (User clicks "Show Original")
error → loading   (User clicks "Retry")
```

### 8.3 Caching (Optional Optimization)

| Behavior | Specification |
|----------|---------------|
| Cache scope | Per chapter, per session |
| Cache storage | Component state (React useState/useReducer) |
| Cache invalidation | On page navigation away from chapter |
| Benefit | Avoid re-translation on toggle |

---

## 9. Error Handling

### 9.1 Error Categories

| Error Type | Cause | User Message | Recovery Action |
|------------|-------|--------------|-----------------|
| AUTH_REQUIRED | User not authenticated | "Please sign in to translate content" | Show sign-in link |
| TRANSLATION_FAILED | Skill invocation failed | "Translation failed. Please try again." | Retry button |
| CONTENT_TOO_LONG | Chapter exceeds limit | "Chapter is too long to translate at once" | Suggest sections |
| NETWORK_ERROR | Connection issue | "Connection error. Check your internet." | Retry button |
| TIMEOUT | Translation took too long | "Translation timed out. Please retry." | Retry button |

### 9.2 Error Display

- Error messages MUST be user-friendly (no technical jargon)
- Error messages MUST be actionable (tell user what to do)
- Error state MUST be dismissible
- Error MUST NOT break chapter page functionality

### 9.3 Logging

- All translation errors SHOULD be logged for debugging
- Logs MUST NOT contain user PII
- Logs SHOULD include: timestamp, error type, chapter ID

---

## 10. Non-Functional Requirements

### 10.1 Performance

| Metric | Target |
|--------|--------|
| Button render time | <100ms after auth check |
| Translation initiation | <500ms to show loading state |
| Translation completion | <30 seconds for average chapter |
| Toggle response | <100ms (instant for cached) |

### 10.2 Reliability

| Requirement | Specification |
|-------------|---------------|
| Translation success rate | >95% for well-formed content |
| Error recovery | Graceful degradation to original content |
| Session handling | Tolerate session expiry during translation |
| Concurrent users | Support multiple users translating simultaneously |

### 10.3 Accessibility

| Requirement | Specification |
|-------------|---------------|
| Button accessibility | ARIA labels for screen readers |
| RTL support | Proper RTL announcement |
| Keyboard navigation | Button focusable and activatable via keyboard |
| Loading state | ARIA live region for status updates |

### 10.4 Security

| Requirement | Specification |
|-------------|---------------|
| Authentication check | Server-side validation before translation |
| Content sanitization | Translated content sanitized before render |
| XSS prevention | No raw HTML injection |
| Rate limiting | Prevent translation abuse (10 requests/minute/user) |

---

## 11. API Contract

### 11.1 Translation Endpoint

This endpoint integrates with the `translate-to-urdu` skill.

#### Request

```
POST /api/translate
Content-Type: application/json
Cookie: Better-Auth session cookie

{
  "chapter_id": "module-1/chapter-01-intro-physical-ai",
  "content": "<full chapter markdown content>"
}
```

#### Response (Success)

```
HTTP/1.1 200 OK
Content-Type: application/json

{
  "success": true,
  "translated_content": "<urdu translated markdown>",
  "chapter_id": "module-1/chapter-01-intro-physical-ai",
  "word_count": 4200,
  "timestamp": "2025-12-17T10:30:00Z"
}
```

#### Response (Error - Unauthorized)

```
HTTP/1.1 401 Unauthorized
Content-Type: application/json

{
  "success": false,
  "error": "AUTH_REQUIRED",
  "message": "Authentication required to translate content"
}
```

#### Response (Error - Translation Failed)

```
HTTP/1.1 500 Internal Server Error
Content-Type: application/json

{
  "success": false,
  "error": "TRANSLATION_FAILED",
  "message": "Translation service encountered an error",
  "retry_after": 5
}
```

### 11.2 Endpoint Placement

| Option | Location | Notes |
|--------|----------|-------|
| Option A (Recommended) | `/auth/src/routes/translate.ts` | Within existing auth backend |
| Option B | New dedicated service | Separate microservice |

Recommendation: Add to existing auth backend to reuse session validation.

---

## 12. Frontend Component Specification

### 12.1 Component: TranslateButton

#### Location

```
/book-write/src/components/Translation/TranslateButton.tsx
```

#### Props

```typescript
interface TranslateButtonProps {
  chapterId: string;           // Unique chapter identifier
  chapterContent: string;      // Original markdown content
  isAuthenticated: boolean;    // From auth context
}
```

#### Behavior

- Render only if `isAuthenticated === true`
- On click: call translation API
- Manage loading/success/error states
- Toggle between original and translated

### 12.2 Component: UrduContent

#### Location

```
/book-write/src/components/Translation/UrduContent.tsx
```

#### Props

```typescript
interface UrduContentProps {
  content: string;             // Translated Urdu markdown
  className?: string;          // Additional styling
}
```

#### Behavior

- Apply RTL styling
- Render Urdu-friendly fonts
- Handle mixed LTR/RTL content (code blocks)

### 12.3 Integration with Chapter Template

The translation components integrate with existing Docusaurus chapter layout:

```
ChapterPage
├── ChapterHeader
│   ├── Title
│   ├── Metadata
│   └── TranslateButton (if authenticated)
├── ChapterContent
│   ├── OriginalContent (default)
│   └── UrduContent (if translated)
└── ChapterFooter
```

---

## 13. Acceptance Criteria

### 13.1 Access Control Tests

**Test AUTH-001**: Button not visible for anonymous users
- **Given**: User is NOT logged in
- **When**: User views any chapter page
- **Then**: "Translate to Urdu" button is NOT rendered

**Test AUTH-002**: Button visible for authenticated users
- **Given**: User IS logged in via Better-Auth
- **When**: User views any chapter page
- **Then**: "Translate to Urdu" button IS visible at chapter start

**Test AUTH-003**: Translation blocked for expired session
- **Given**: User's session has expired
- **When**: User clicks "Translate to Urdu"
- **Then**: 401 error returned, user prompted to sign in

### 13.2 Translation Tests

**Test TRANS-001**: Successful translation
- **Given**: Authenticated user on Chapter 1
- **When**: User clicks "Translate to Urdu"
- **Then**: Chapter content is translated to Urdu using translate-to-urdu skill

**Test TRANS-002**: Code blocks preserved
- **Given**: Chapter contains Python code blocks
- **When**: Chapter is translated
- **Then**: Code blocks remain in original form (not translated)

**Test TRANS-003**: Technical terms preserved
- **Given**: Chapter mentions "ROS 2", "URDF", "rclpy"
- **When**: Chapter is translated
- **Then**: Technical terms remain in English within Urdu text

**Test TRANS-004**: Structure preserved
- **Given**: Chapter has headings, lists, tables
- **When**: Chapter is translated
- **Then**: Document structure is maintained

### 13.3 Display Tests

**Test DISPLAY-001**: RTL rendering
- **Given**: Translated Urdu content displayed
- **When**: User views content
- **Then**: Text flows right-to-left, properly aligned

**Test DISPLAY-002**: Font rendering
- **Given**: Translated Urdu content displayed
- **When**: User views on supported browser
- **Then**: Urdu Nastaliq font renders correctly

**Test DISPLAY-003**: Toggle functionality
- **Given**: User has translated a chapter
- **When**: User clicks "Show Original"
- **Then**: Original English content is displayed

**Test DISPLAY-004**: Responsive display
- **Given**: Translated content on mobile device
- **When**: User views content
- **Then**: Content is readable with proper RTL and sizing

### 13.4 Error Handling Tests

**Test ERROR-001**: Translation failure recovery
- **Given**: Translation skill fails
- **When**: Error occurs
- **Then**: User sees friendly error message with retry option

**Test ERROR-002**: Network error handling
- **Given**: Network connection lost during translation
- **When**: Request fails
- **Then**: User sees connection error with retry option

---

## 14. Out of Scope

The following are **explicitly NOT included** in this specification:

### Translation Features
- Bulk translation of multiple chapters at once
- Translation caching in database (server-side)
- Translation to languages other than Urdu
- User-editable translations
- Translation quality rating system

### Content Features
- PDF export of translated content
- Audio narration of Urdu content
- Translation memory/glossary management
- Machine learning model training/fine-tuning

### System Features
- Admin panel for translation management
- Translation analytics dashboard
- A/B testing of translations
- Crowdsourced translation corrections

---

## 15. Implementation Order

1. Create Translation components directory structure
2. Implement `TranslateButton` component with auth check
3. Implement `UrduContent` component with RTL styling
4. Add translation API endpoint in auth backend
5. Integrate translate-to-urdu skill invocation
6. Add component to chapter template (conditional on auth)
7. Implement state management for translation toggle
8. Add error handling and retry logic
9. Add CSS for Urdu typography
10. Integration testing across all chapters
11. Responsive design testing
12. End-to-end testing with authentication flow

---

## 16. Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-17 | Claude Code | Initial specification |

---

**END OF SPECIFICATION**
