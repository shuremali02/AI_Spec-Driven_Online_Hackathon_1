# Tasks: Chapter Content Translation to Urdu (Bonus Feature)

**Input**: Design documents from `/specs/translation-feature/`
**Prerequisites**: plan.md (required), spec.md (required), data-model.md, contracts/translate-api-contract.md

**Tests**: Manual testing via acceptance criteria. No automated tests requested.

**Organization**: Tasks grouped by functional area to enable incremental implementation. All tasks contribute to the 50-point bonus feature.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story/functional area this task belongs to
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `book-write/src/components/`, `book-write/src/css/`
- **Backend**: `auth/src/routes/`, `auth/src/`
- **Config**: `book-write/docusaurus.config.ts`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create directory structure and configuration for translation feature

- [X] T001 Create Translation components directory at book-write/src/components/Translation/
- [X] T002 [P] Add Google Font (Noto Nastaliq Urdu) link to book-write/docusaurus.config.ts headTags
- [X] T003 [P] Create index.ts barrel export file at book-write/src/components/Translation/index.ts

**Checkpoint**: Directory structure ready for component development

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: CSS styles that all components depend on

**⚠️ CRITICAL**: These styles must be complete before UI components can be tested

- [X] T004 Create Translation.module.css at book-write/src/components/Translation/Translation.module.css with button styles
- [X] T005 [P] Add Urdu RTL styles to Translation.module.css (.urdu-content, .urduText, .urduBadge classes)
- [X] T006 [P] Add responsive media queries for mobile/tablet to Translation.module.css
- [X] T007 [P] Add dark mode support styles to Translation.module.css

**Checkpoint**: Styles ready - component implementation can begin

---

## Phase 3: User Story 1 - Translation Button Component (Priority: P1) 🎯 MVP

**Goal**: Create TranslateButton component that only renders for authenticated users

**Independent Test**: Sign in → navigate to chapter → verify button visible. Sign out → verify button hidden.

### Implementation for User Story 1

- [X] T008 [US1] Create TranslateButtonProps interface in book-write/src/components/Translation/types.ts
- [X] T009 [US1] Implement TranslateButton component skeleton in book-write/src/components/Translation/TranslateButton.tsx
- [X] T010 [US1] Add useAuth hook integration to check authentication in TranslateButton.tsx
- [X] T011 [US1] Implement conditional rendering (return null if not authenticated) in TranslateButton.tsx
- [X] T012 [US1] Add button states (idle, loading, success, error) UI in TranslateButton.tsx
- [X] T013 [US1] Add loading spinner/disabled state during translation in TranslateButton.tsx
- [X] T014 [US1] Export TranslateButton from book-write/src/components/Translation/index.ts

**Checkpoint**: TranslateButton renders only for authenticated users, shows correct button states

---

## Phase 4: User Story 2 - Backend Translation API (Priority: P1) 🎯 MVP

**Goal**: Create /api/translate endpoint with auth validation and translation logic

**Independent Test**: curl POST /api/translate with valid session → 200 response. Without session → 401.

### Implementation for User Story 2

- [X] T015 [US2] Create translate.ts route file at auth/src/routes/translate.ts
- [X] T016 [US2] Add Hono router setup with requireAuth middleware in translate.ts
- [X] T017 [US2] Implement request body validation (chapter_id, content required) in translate.ts
- [X] T018 [US2] Implement content length validation (max 100,000 chars) in translate.ts
- [X] T019 [US2] Implement translateToUrdu helper function with code block preservation in translate.ts
- [X] T020 [US2] Add regex to extract and preserve code blocks (```...```) in translateToUrdu
- [X] T021 [US2] Add regex to extract and preserve inline code (`...`) in translateToUrdu
- [X] T022 [US2] Add technical terms preservation list in translate.ts
- [X] T023 [US2] Implement translation response with success/error format in translate.ts
- [X] T024 [US2] Register translate route in auth/src/index.ts (import and app.route)
- [X] T025 [US2] Add console logging for translation requests (chapter_id, timestamp) in translate.ts
- [X] T025a [US2] Implement rate limiting middleware (10 requests/minute/user) using in-memory store in translate.ts
- [X] T025b [US2] Return 429 RATE_LIMITED error with retry_after when limit exceeded in translate.ts

**Checkpoint**: /api/translate returns 200 with translated_content for authenticated requests, 401 for unauthenticated, 429 for rate limit exceeded

---

## Phase 5: User Story 3 - Urdu Content Display (Priority: P1) 🎯 MVP

**Goal**: Create UrduContent component with proper RTL styling and font rendering

**Independent Test**: Render UrduContent with sample Urdu text → verify RTL direction, Nastaliq font, code blocks LTR

### Implementation for User Story 3

- [X] T026 [P] [US3] Create UrduContentProps interface in book-write/src/components/Translation/types.ts
- [X] T027 [US3] Implement UrduContent component in book-write/src/components/Translation/UrduContent.tsx
- [X] T028 [US3] Apply .urdu-content RTL styles in UrduContent.tsx
- [X] T029 [US3] Add "اردو ترجمہ" badge display at top in UrduContent.tsx
- [X] T030 [US3] Handle mixed LTR/RTL content (code blocks stay LTR) in UrduContent.tsx
- [X] T031 [US3] Export UrduContent from book-write/src/components/Translation/index.ts

**Checkpoint**: UrduContent renders Urdu text with RTL direction, Nastaliq font, distinct visual styling

---

## Phase 6: User Story 4 - Toggle & State Management (Priority: P2)

**Goal**: Enable toggling between original and translated content

**Independent Test**: Translate chapter → click "Show Original" → see English. Click "Translate" → see cached Urdu.

### Implementation for User Story 4

- [X] T032 [US4] Add TranslationState interface (status, translatedContent, showTranslated) to types.ts
- [X] T033 [US4] Implement useState for translation state in TranslateButton.tsx
- [X] T034 [US4] Add handleTranslate function with API call in TranslateButton.tsx
- [X] T035 [US4] Add handleToggle function for switching views in TranslateButton.tsx
- [X] T036 [US4] Implement caching: skip API call if translatedContent exists in TranslateButton.tsx
- [X] T037 [US4] Add onTranslate callback prop to pass translated content to parent in TranslateButton.tsx
- [X] T038 [US4] Update button label based on state (Translate/Show Original/Loading) in TranslateButton.tsx

**Checkpoint**: User can translate, view Urdu, toggle back to English, toggle to cached Urdu without re-translating

---

## Phase 7: User Story 5 - Error Handling (Priority: P2)

**Goal**: Handle translation errors gracefully with retry option

**Independent Test**: Simulate network error → see error message. Click retry → attempt translation again.

### Implementation for User Story 5

- [X] T039 [US5] Add error state handling in TranslateButton.tsx handleTranslate
- [X] T040 [US5] Display user-friendly error message below button in TranslateButton.tsx
- [X] T041 [US5] Add retry button/link when in error state in TranslateButton.tsx
- [X] T042 [US5] Handle AUTH_REQUIRED error with sign-in prompt in TranslateButton.tsx
- [X] T043 [US5] Add try/catch in backend translate.ts for translation failures
- [X] T044 [US5] Return proper error codes (400, 401, 500) from translate.ts

**Checkpoint**: Errors show clear message and retry option, auth errors prompt sign-in

---

## Phase 8: User Story 6 - Chapter Integration (Priority: P1) 🎯 MVP

**Goal**: Integrate TranslateButton into chapter pages

**Independent Test**: Navigate to Chapter 1 while logged in → see Translate button at chapter start

### Implementation for User Story 6

- [X] T045 [US6] Create TranslationWrapper component to wrap chapter content in book-write/src/components/Translation/TranslationWrapper.tsx
- [X] T046 [US6] Implement conditional content display (original vs translated) in TranslationWrapper.tsx
- [X] T047 [US6] Add TranslateButton import to Module 1 Chapter 1 MDX file book-write/docs/module-1/chapter-01-intro-physical-ai.md
- [X] T048 [P] [US6] Add TranslateButton import to Module 1 Chapter 2 MDX file book-write/docs/module-1/chapter-02-ros2-architecture.md
- [X] T049 [P] [US6] Add TranslateButton import to Module 1 Chapter 3 MDX file book-write/docs/module-1/chapter-03-first-nodes.md
- [X] T050 [P] [US6] Add TranslateButton import to Module 1 Chapter 4 MDX file book-write/docs/module-1/chapter-04-urdf.md
- [X] T051 [P] [US6] Add TranslateButton import to Module 1 Chapter 5 MDX file book-write/docs/module-1/chapter-05-ros2-launch-files.md

**Checkpoint**: All Module 1 chapters show Translate button for authenticated users

---

## Phase 9: Polish & Cross-Cutting Concerns

**Purpose**: Final verification and improvements

- [X] T052 [P] Add ARIA labels for accessibility to TranslateButton.tsx
- [X] T053 [P] Add keyboard navigation support (Enter/Space to activate) to TranslateButton.tsx
- [X] T054 Verify translation works end-to-end on all chapters
- [X] T055 Test responsive design on mobile/tablet viewports
- [X] T056 Test dark mode styling
- [X] T057 Run quickstart.md validation steps

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Phase 1 directory creation
- **US1 TranslateButton (Phase 3)**: Depends on Phase 2 styles
- **US2 Backend API (Phase 4)**: No frontend dependencies - can parallel with US1
- **US3 UrduContent (Phase 5)**: Depends on Phase 2 styles
- **US4 Toggle (Phase 6)**: Depends on US1 + US2 + US3 being complete
- **US5 Error Handling (Phase 7)**: Depends on US4 state management
- **US6 Chapter Integration (Phase 8)**: Depends on US1 being functional
- **Polish (Phase 9)**: Depends on all user stories complete

### User Story Dependencies

```
Phase 1: Setup
    │
    ▼
Phase 2: Foundational (styles)
    │
    ├──────────────────┬──────────────────┐
    ▼                  ▼                  ▼
Phase 3: US1       Phase 4: US2      Phase 5: US3
(TranslateButton)  (Backend API)     (UrduContent)
    │                  │                  │
    └──────────────────┼──────────────────┘
                       │
                       ▼
                 Phase 6: US4
                 (Toggle/State)
                       │
                       ▼
                 Phase 7: US5
                 (Error Handling)
                       │
                       ▼
                 Phase 8: US6
                 (Chapter Integration)
                       │
                       ▼
                 Phase 9: Polish
```

### Parallel Opportunities

**Phase 1-2 (Setup/Foundational):**
```bash
# Can run in parallel after T001:
T002: Google Font link
T003: index.ts barrel

# Can run in parallel after T004:
T005: RTL styles
T006: Responsive styles
T007: Dark mode styles
```

**Phases 3-5 (Core Components):**
```bash
# US1 and US2 can run in parallel (frontend vs backend):
Phase 3: TranslateButton (frontend)
Phase 4: Backend API (backend)

# US3 can run in parallel with US1/US2 after styles ready
Phase 5: UrduContent (frontend)
```

**Phase 8 (Chapter Integration):**
```bash
# All chapter MDX updates can run in parallel:
T048, T049, T050, T051: Chapter 2-5 MDX updates
```

---

## Parallel Example: Initial Development

```bash
# After Phase 1 Setup complete, launch these in parallel:

# Developer A (Frontend):
T004: Button styles in Translation.module.css
T008-T014: TranslateButton component

# Developer B (Backend):
T015-T025b: Backend /api/translate route (includes rate limiting)

# Developer C (Styling):
T005-T007: RTL/Responsive/Dark mode styles
T026-T031: UrduContent component
```

---

## Implementation Strategy

### MVP First (Phases 1-5 + Phase 8)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational styles
3. Complete Phase 3: TranslateButton (auth-gated)
4. Complete Phase 4: Backend API
5. Complete Phase 5: UrduContent
6. Complete Phase 8: Chapter integration (at least one chapter)
7. **STOP and VALIDATE**: Can authenticate user translate Chapter 1?
8. If yes → MVP is functional, bonus points achievable

### Full Implementation

1. Complete MVP phases (1-5, 8)
2. Add Phase 6: Toggle/State management
3. Add Phase 7: Error handling
4. Add remaining chapters to Phase 8
5. Complete Phase 9: Polish

### Incremental Delivery

| Milestone | Deliverable | Bonus Eligibility |
|-----------|-------------|-------------------|
| Phases 1-3 | Button visible for auth users | Partial |
| + Phase 4 | API returns translated content | Partial |
| + Phase 5 | Urdu renders with RTL | Partial |
| + Phase 8 (1 chapter) | **Full E2E works** | **YES - MVP** |
| + Phases 6-7 | Toggle + error handling | Enhanced |
| + Phase 9 | Polish | Complete |

---

## Summary

| Metric | Value |
|--------|-------|
| **Total Tasks** | 59 |
| **Setup Tasks** | 3 |
| **Foundational Tasks** | 4 |
| **US1 (TranslateButton)** | 7 |
| **US2 (Backend API)** | 13 (includes rate limiting) |
| **US3 (UrduContent)** | 6 |
| **US4 (Toggle)** | 7 |
| **US5 (Error Handling)** | 6 |
| **US6 (Chapter Integration)** | 7 |
| **Polish Tasks** | 6 |
| **Parallel Opportunities** | 18 tasks marked [P] |
| **MVP Scope** | Phases 1-5 + Phase 8 (42 tasks) |

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [Story] label maps task to specific user story for traceability
- Each phase has a checkpoint to validate progress
- Auth backend already exists - only adding new translate route
- Existing useAuth hook provides authentication state
- No database changes required (stateless feature)
- Commit after each task or logical group
- Stop at MVP milestone if time-constrained

---

**END OF TASKS**
