# Tasks: Landing Page Features Section

**Input**: Design documents from `/specs/002-landing-features/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, quickstart.md

**Tests**: Manual testing only (no automated tests requested in spec)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `book-write/src/components/`
- **Pages**: `book-write/src/pages/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Create component directory structure

- [x] T001 Create InteractiveFeatures component directory at book-write/src/components/InteractiveFeatures/
- [x] T002 Verify AuthProvider exports useAuth hook correctly in book-write/src/components/Auth/AuthProvider.tsx
- [x] T003 Verify landing page structure in book-write/src/pages/index.tsx

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Create shared styles that all user stories depend on

- [x] T004 Create styles.module.css with responsive grid layout in book-write/src/components/InteractiveFeatures/styles.module.css
- [x] T005 [P] Add dark mode styles to styles.module.css in book-write/src/components/InteractiveFeatures/styles.module.css

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Anonymous User Discovers Features (Priority: P1) MVP

**Goal**: Display 3 feature cards with icons, titles, descriptions, and CTAs visible to all users

**Independent Test**: Visit the landing page without logging in and verify all three feature cards are visible with icons, titles, descriptions, and CTAs.

### Implementation for User Story 1

- [x] T006 [US1] Create InteractiveFeatureCard component structure in book-write/src/components/InteractiveFeatures/InteractiveFeatureCard.tsx
- [x] T007 [US1] Add icon container, title, and description rendering to InteractiveFeatureCard in book-write/src/components/InteractiveFeatures/InteractiveFeatureCard.tsx
- [x] T008 [US1] Add CTA button to InteractiveFeatureCard (static text for now) in book-write/src/components/InteractiveFeatures/InteractiveFeatureCard.tsx
- [x] T009 [US1] Create SVG icon components (PersonalizationIcon, TranslationIcon, ChatbotIcon) in book-write/src/components/InteractiveFeatures/index.tsx
- [x] T010 [US1] Create features data array with 3 feature objects in book-write/src/components/InteractiveFeatures/index.tsx
- [x] T011 [US1] Create InteractiveFeatures section component with header in book-write/src/components/InteractiveFeatures/index.tsx
- [x] T012 [US1] Add feature grid rendering with InteractiveFeatureCard map in book-write/src/components/InteractiveFeatures/index.tsx
- [x] T013 [US1] Import and add InteractiveFeatures to landing page in book-write/src/pages/index.tsx
- [ ] T014 [US1] Manual test: Verify 3 feature cards visible on landing page with icons, titles, descriptions

**Checkpoint**: At this point, User Story 1 should be fully functional - all users see 3 feature cards

---

## Phase 4: User Story 2 - User Clicks CTA to Sign Up (Priority: P2)

**Goal**: CTAs redirect anonymous users to auth page, logged-in users to feature pages

**Independent Test**: Click any feature CTA button - anonymous users go to /auth, logged-in users go to chapter page.

### Implementation for User Story 2

- [x] T015 [US2] Add useAuth hook import and call to InteractiveFeatureCard in book-write/src/components/InteractiveFeatures/InteractiveFeatureCard.tsx
- [x] T016 [US2] Implement auth-aware navigation logic (anonymous -> /auth, logged-in -> featureUrl) in book-write/src/components/InteractiveFeatures/InteractiveFeatureCard.tsx
- [x] T017 [US2] Update CTA button text based on auth state in book-write/src/components/InteractiveFeatures/InteractiveFeatureCard.tsx
- [ ] T018 [US2] Manual test: Verify anonymous user CTA clicks redirect to /auth page
- [ ] T019 [US2] Manual test: Verify logged-in user CTA clicks redirect to feature chapter page

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - cards visible AND CTAs navigate correctly

---

## Phase 5: User Story 3 - Visual Appeal and Engagement (Priority: P3)

**Goal**: Add hover effects, animations, and ensure dark mode support

**Independent Test**: View features section in both light and dark modes, verify hover effects work smoothly.

### Implementation for User Story 3

- [x] T020 [US3] Add hover effect styles (translateY, box-shadow) to card in book-write/src/components/InteractiveFeatures/styles.module.css
- [x] T021 [US3] Add fade-in animation keyframes and staggered animation delay in book-write/src/components/InteractiveFeatures/styles.module.css
- [x] T022 [US3] Add CTA button hover effect (scale) in book-write/src/components/InteractiveFeatures/styles.module.css
- [ ] T023 [US3] Manual test: Verify hover effects on desktop (card lift, button scale)
- [ ] T024 [US3] Manual test: Verify dark mode styling (backgrounds, text, borders)
- [ ] T025 [US3] Manual test: Verify fade-in animation on page load

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final verification and responsive testing

- [ ] T026 [P] Manual test: Verify responsive layout - 3 columns on desktop (>996px)
- [ ] T027 [P] Manual test: Verify responsive layout - 2 columns on tablet (768-996px)
- [ ] T028 [P] Manual test: Verify responsive layout - 1 column on mobile (<768px)
- [ ] T029 Manual test: Verify all icons display correctly in both themes
- [ ] T030 Manual test: Verify cards load within 1 second (no perceptible delay)
- [ ] T031 Run quickstart.md validation steps

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - directory creation and verification
- **Foundational (Phase 2)**: Depends on Phase 1 (directory exists)
- **User Stories (Phase 3+)**: Depends on Phase 2 (styles exist)
  - User Story 1 is MVP - complete first
  - User Story 2 extends US1 (adds auth logic to existing cards)
  - User Story 3 extends US1/US2 (adds animations to existing cards)
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: Depends on Phase 2 styles only - can start immediately after foundation
- **User Story 2 (P2)**: Depends on US1 (card component must exist to add auth logic)
- **User Story 3 (P3)**: Can technically run parallel to US2 (different code areas) but typically done after

### Within Each User Story

- Component structure before content
- Static implementation before dynamic (auth-aware)
- Implementation before manual testing
- Core functionality before edge cases

### Parallel Opportunities

- T004 and T005 (Foundation styles) can run in parallel
- T026, T027, T028 (Responsive tests) can run in parallel
- US3 styling tasks (T020, T021, T022) can run in parallel

---

## Parallel Example: User Story 1

```bash
# Component creation is sequential (same files):
Task T006: Create InteractiveFeatureCard structure
Task T007: Add rendering logic
Task T008: Add CTA button
Task T009-T012: Create index.tsx with icons and section

# Then integration:
Task T013: Add to landing page

# Then test:
Task T014: Manual test verification
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundation (T004-T005)
3. Complete Phase 3: User Story 1 (T006-T014)
4. **STOP and VALIDATE**: Test User Story 1 independently
5. Deploy/demo if ready - basic feature cards visible!

### Incremental Delivery

1. Complete US1 → Deploy (MVP: feature cards visible)
2. Complete US2 → Deploy (CTAs navigate correctly based on auth)
3. Complete US3 → Deploy (animations and polish)
4. Polish phase → Final deploy

### Estimated Time

- Total tasks: 31
- MVP (US1 only): 14 tasks (~20-30 minutes)
- Full implementation: 31 tasks (~45-60 minutes)

---

## Notes

- All implementation happens in a single directory: `book-write/src/components/InteractiveFeatures/`
- Only one existing file modified: `book-write/src/pages/index.tsx`
- No backend changes required - frontend-only feature
- Manual testing is sufficient - no automated tests requested
- Feature follows existing patterns (HomepageFeatures, FeatureCard)
- Commit after each logical group (Setup, Foundation, US1, US2, US3, Polish)
