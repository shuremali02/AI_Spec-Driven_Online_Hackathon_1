# Implementation Plan: Chapter Content Translation to Urdu

**Branch**: `translation-feature` | **Date**: 2025-12-17 | **Spec**: [specs/translation-feature/spec.md](./spec.md)
**Input**: Feature specification from `/specs/translation-feature/spec.md`

---

## Summary

Implement a bonus feature (50 points) that allows authenticated users to translate chapter content from English to Urdu by pressing a button at the start of each chapter. The feature uses the existing `translate-to-urdu` skill, requires Better-Auth authentication, and displays translated content with proper RTL styling.

**Key Technical Approach**:
- Frontend: React components (TranslateButton, UrduContent) integrated with Docusaurus
- Backend: New `/api/translate` endpoint in auth service with session validation
- Styling: CSS-only RTL with Google Fonts (Noto Nastaliq Urdu)
- State: Component-level React state (no database persistence)

---

## Technical Context

**Language/Version**: TypeScript 5.x (frontend), TypeScript/Node.js 18+ (backend)
**Primary Dependencies**: React 18, Docusaurus 3.x, Hono, Better-Auth
**Storage**: None (stateless - no database for translation)
**Testing**: Manual testing, acceptance criteria from spec
**Target Platform**: Web (Desktop, Tablet, Mobile responsive)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: <30s translation time, <100ms toggle response
**Constraints**: Must use existing translate-to-urdu skill, auth required
**Scale/Scope**: ~5 chapters initially, ~4000 words per chapter

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Pre-Design Check

| Gate | Requirement | Status | Evidence |
|------|-------------|--------|----------|
| **Key Standard VIII** | Translation triggered by button at chapter start | PASS | Spec Section 4.1 defines button placement |
| **Key Standard VIII** | Only available to logged-in users | PASS | Spec Section 3.2 access matrix |
| **Key Standard VIII** | Uses existing translate_to_urdu skill | PASS | Spec Section 5.1 skill integration |
| **Key Standard VIII** | Technical terms and code blocks preserved | PASS | Spec Section 5.1.2 content rules |
| **Key Standard VIII** | Original English content accessible | PASS | Spec Section 7.3 preservation |
| **Core Principle V** | Anonymous users cannot access | PASS | Spec Section 3.2 access matrix |
| **Success Criteria V** | User-triggered translation | PASS | Spec Section 4.2 interaction flow |

### Post-Design Check

| Gate | Requirement | Status | Evidence |
|------|-------------|--------|----------|
| **Key Standard VIII** | All requirements met | PASS | Design artifacts complete |
| **No Hallucinations** | Grounded in spec | PASS | All decisions from research.md |
| **Consistency** | Uniform patterns | PASS | Follows auth-personalization patterns |

---

## Project Structure

### Documentation (this feature)

```text
specs/translation-feature/
├── spec.md              # Feature specification (created)
├── plan.md              # This file
├── research.md          # Phase 0 output (created)
├── data-model.md        # Phase 1 output (created)
├── quickstart.md        # Phase 1 output (created)
├── contracts/
│   └── translate-api-contract.md  # API contract (created)
└── tasks.md             # Phase 2 output (TBD - /sp.tasks)
```

### Source Code (repository root)

```text
# Frontend (Docusaurus)
book-write/
├── src/
│   ├── components/
│   │   └── Translation/           # NEW
│   │       ├── TranslateButton.tsx
│   │       ├── UrduContent.tsx
│   │       ├── Translation.module.css
│   │       └── index.ts
│   └── css/
│       └── custom.css             # ADD: Urdu font import
├── docusaurus.config.ts           # MODIFY: Add Google Font link

# Backend (Auth Service)
auth/
├── src/
│   ├── routes/
│   │   └── translate.ts           # NEW: Translation endpoint
│   └── index.ts                   # MODIFY: Register translate route
```

**Structure Decision**: Extends existing web application structure. Frontend components added to `/book-write/src/components/`. Backend route added to existing `/auth/` service to reuse session validation.

---

## Complexity Tracking

No constitution violations requiring justification. Design follows minimal viable approach:

| Decision | Simpler Alternative | Why Current Approach |
|----------|--------------------|--------------------|
| Backend API vs frontend-only | Frontend could call skill directly | Skills require agent context, auth validation needed server-side |
| Component state vs global state | Could use Redux | Per-chapter scope doesn't need global state |
| CSS-only RTL vs library | Could use react-rtl or similar | CSS `direction: rtl` is sufficient |

---

## Architecture Decisions

### ADR-001: Translation Backend Placement

**Decision**: Add translation endpoint to existing `/auth/` backend

**Rationale**:
- Reuses existing session validation middleware
- No new service to deploy/maintain
- Single point of authentication
- Consistent with auth-personalization pattern

**Alternatives Rejected**:
- New microservice: Overkill for single endpoint
- Frontend-only: Skills need agent context, auth validation

### ADR-002: State Management

**Decision**: Component-level React state (useState)

**Rationale**:
- Translation state is per-chapter, per-session
- No persistence requirement (out of scope)
- Simple toggle logic
- Caching in component state is sufficient

**Alternatives Rejected**:
- Redux: Overkill for chapter-scoped state
- Context API: Not needed, no cross-component sharing
- localStorage: Out of scope per spec

### ADR-003: RTL Implementation

**Decision**: CSS-only with scoped `.urdu-content` class

**Rationale**:
- Native CSS `direction: rtl` handles text flow
- Scoped class prevents global layout issues
- Code blocks override with `direction: ltr`
- No additional dependencies

**Alternatives Rejected**:
- HTML dir attribute: Harder to scope
- Third-party RTL library: Unnecessary complexity

---

## Component Design

### Frontend Components

```
┌─────────────────────────────────────────────────────────────┐
│ ChapterPage (Docusaurus)                                    │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ TranslateButton                                       │  │
│  │   Props: chapterId, chapterContent, onTranslate       │  │
│  │   State: status, showTranslated, error                │  │
│  │   Renders: Only if authenticated (useAuth)            │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ Content Display                                       │  │
│  │   Conditional:                                        │  │
│  │   - showTranslated=false → Original MDX content       │  │
│  │   - showTranslated=true  → UrduContent component      │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
│  ┌───────────────────────────────────────────────────────┐  │
│  │ UrduContent                                           │  │
│  │   Props: content (translated markdown)                │  │
│  │   Styling: RTL, Noto Nastaliq Urdu font               │  │
│  └───────────────────────────────────────────────────────┘  │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Backend Endpoint

```
POST /api/translate
├── Middleware: requireAuth (validates Better-Auth session)
├── Validation: chapter_id, content (required), content length
├── Processing: Parse markdown, apply translation rules
└── Response: { success, translated_content, chapter_id, timestamp }
```

---

## Data Flow

```
User Action: Click "Translate to Urdu"
        │
        ▼
TranslateButton.handleTranslate()
        │
        ├── Check: isAuthenticated? (from useAuth)
        │         No → Button not rendered (never reaches here)
        │
        ├── Set state: status = 'loading'
        │
        ▼
fetch('/api/translate', { method: 'POST', body: { chapter_id, content } })
        │
        ▼
Backend: /api/translate
        │
        ├── requireAuth middleware → validates session
        │
        ├── Validate request body
        │
        ├── translateToUrdu(content)
        │         │
        │         ├── Extract code blocks (preserve)
        │         ├── Extract inline code (preserve)
        │         ├── Apply translation to text
        │         └── Reconstruct markdown
        │
        ▼
Response: { success: true, translated_content: "..." }
        │
        ▼
TranslateButton.handleTranslate() continues
        │
        ├── Set state: status = 'success', showTranslated = true
        │
        ├── Call onTranslate(translated_content)
        │
        ▼
UrduContent renders with RTL styling
```

---

## Integration Points

### 1. AuthProvider Integration

```typescript
// TranslateButton uses existing useAuth hook
import { useAuth } from '@site/src/components/Auth/AuthProvider';

const { user, isLoading } = useAuth();
if (!user) return null; // Don't render button
```

### 2. Docusaurus Layout Integration

Options for integrating TranslateButton into chapter pages:

**Option A: MDX Import** (per chapter)
```mdx
import TranslateButton from '@site/src/components/Translation/TranslateButton';

<TranslateButton chapterId="chapter-01" chapterContent={rawContent} />
```

**Option B: Theme Swizzle** (global)
```typescript
// Swizzle DocItem/Layout to inject TranslateButton
// Automatic for all doc pages
```

**Decision**: Start with Option A for MVP, consider Option B for DRY.

### 3. Auth Backend Integration

```typescript
// auth/src/index.ts
import translateRoutes from './routes/translate.js';
app.route('/api', translateRoutes); // Adds /api/translate
```

---

## Implementation Phases

### Phase 1: Core Components (MVP)

1. Create `TranslateButton` component
2. Create `UrduContent` component
3. Add CSS for RTL styling
4. Add Google Font (Noto Nastaliq Urdu)

### Phase 2: Backend API

5. Create `/api/translate` route
6. Implement `requireAuth` validation
7. Implement basic translation logic (preserve code blocks)
8. Register route in auth/index.ts

### Phase 3: Integration

9. Integrate TranslateButton into chapter pages
10. Wire up state management (toggle)
11. Error handling and retry logic

### Phase 4: Polish

12. Responsive design testing
13. Cross-browser testing
14. All chapters verification

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Translation skill unavailable | Graceful error message, retry button |
| Large chapters timeout | Set 30s timeout, progress indicator |
| RTL rendering issues | Test across browsers, use standard CSS |
| Auth race condition | Check isLoading before render |

---

## Success Criteria Mapping

| Spec Criteria | Implementation |
|---------------|----------------|
| Button at chapter start | TranslateButton component positioned after title |
| Auth required | useAuth check + requireAuth middleware |
| Uses translate-to-urdu skill | Backend applies skill rules |
| Code blocks preserved | Regex extraction/restoration |
| RTL display | CSS direction: rtl |
| Toggle functionality | showTranslated state toggle |

---

## Next Steps

1. Run `/sp.tasks` to generate detailed task breakdown
2. Implement following quickstart.md
3. Test acceptance criteria
4. Submit for bonus point evaluation

---

**END OF PLAN**
