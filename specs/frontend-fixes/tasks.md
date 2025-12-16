# Tasks: Frontend Fixes & UI Enhancements

**Input**: Design documents from `/specs/frontend-fixes/`
**Prerequisites**: plan.md ✅, spec.md ✅, research.md ✅, data-model.md ✅, contracts/ ✅

**Tests**: Manual testing only (no automated tests per spec)

**Organization**: Tasks grouped by user story for independent implementation and testing.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: US1-US5 maps to user stories from spec.md
- Exact file paths included in all descriptions

## Path Conventions

- **Framework**: Docusaurus (book-write/)
- **Source**: `book-write/src/`
- **Components**: `book-write/src/components/`
- **Pages**: `book-write/src/pages/`
- **Theme**: `book-write/src/theme/`
- **CSS**: `book-write/src/css/`

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify existing infrastructure and prepare for implementation

- [X] T001 Verify Docusaurus dev server runs without errors in book-write/
- [X] T002 [P] Verify existing AuthProvider works in book-write/src/components/Auth/AuthProvider.tsx
- [X] T003 [P] Verify existing auth pages load at /auth and /profile
- [X] T004 Create NavbarAuth component directory at book-write/src/components/NavbarAuth/
- [X] T005 [P] Create FeatureCard component directory at book-write/src/components/FeatureCard/

**Checkpoint**: Project structure ready, all directories created ✅

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before user stories can proceed

**⚠️ CRITICAL**: Navbar auth integration requires AuthProvider at root level

- [X] T006 Extend Root.tsx with AuthProvider wrapper in book-write/src/theme/Root.tsx
- [X] T007 Verify auth context is accessible from any component after T006

**Checkpoint**: Foundation ready - AuthProvider available app-wide ✅

---

## Phase 3: User Story 1 - Sidebar CSS Fix (Priority: P0) 🎯 MVP

**Goal**: Fix sidebar transparency bug in light mode - Critical visual bug

**Independent Test**: Open mobile sidebar in light mode, verify solid white background with no transparency

### Implementation for User Story 1

- [X] T008 [US1] Add light mode sidebar background fix in book-write/src/css/custom.css
- [X] T009 [US1] Add dark mode sidebar background rule in book-write/src/css/custom.css
- [X] T010 [US1] Remove backdrop-filter from sidebar elements in book-write/src/css/custom.css
- [X] T011 [US1] Add explicit opacity:1 to sidebar in book-write/src/css/custom.css
- [ ] T012 [US1] Verify sidebar opaque in light mode (manual test)
- [ ] T013 [US1] Verify sidebar opaque in dark mode (manual test)
- [ ] T014 [US1] Verify sidebar correct after theme switch (manual test)

**Checkpoint**: Sidebar CSS bug fixed - can demo independently ✅

---

## Phase 4: User Story 2 - Navbar Auth Controls (Priority: P1)

**Goal**: Add Signup/Signin buttons to navbar for unauthenticated users, Profile icon for authenticated users

**Independent Test**: Load site unauthenticated → see buttons; login → see profile icon

### Implementation for User Story 2

- [X] T015 [P] [US2] Create NavbarAuth component in book-write/src/components/NavbarAuth/index.tsx
- [X] T016 [P] [US2] Create NavbarAuth styles in book-write/src/components/NavbarAuth/styles.module.css
- [X] T017 [US2] Implement loading skeleton state in NavbarAuth component
- [X] T018 [US2] Implement unauthenticated state (auth buttons) in NavbarAuth component
- [X] T019 [US2] Implement authenticated state (profile icon) in NavbarAuth component
- [X] T020 [US2] Add navbar button hover animations in book-write/src/components/NavbarAuth/styles.module.css
- [X] T021 [US2] Add focus states for accessibility in book-write/src/components/NavbarAuth/styles.module.css
- [X] T022 [US2] Integrate NavbarAuth into navbar via docusaurus.config.ts or theme swizzle
- [ ] T023 [US2] Verify auth buttons visible when unauthenticated (manual test)
- [ ] T024 [US2] Verify profile icon visible when authenticated (manual test)
- [ ] T025 [US2] Verify navigation to correct routes (manual test)

**Checkpoint**: Navbar auth controls working - can demo independently ✅

---

## Phase 5: User Story 3 - Auth Page URL Parameters (Priority: P1)

**Goal**: Auth page reads ?mode=signup or ?mode=signin to display appropriate form

**Independent Test**: Visit /auth?mode=signup → signup form shown; /auth?mode=signin → signin form shown

### Implementation for User Story 3

- [X] T026 [US3] Add URL parameter extraction function in book-write/src/pages/auth.tsx
- [X] T027 [US3] Update initial mode state to use URL param in book-write/src/pages/auth.tsx
- [X] T028 [US3] Add popstate event listener for browser back/forward in book-write/src/pages/auth.tsx
- [ ] T029 [US3] Verify /auth?mode=signup shows signup form (manual test)
- [ ] T030 [US3] Verify /auth?mode=signin shows signin form (manual test)
- [ ] T031 [US3] Verify /auth defaults to signin form (manual test)

**Checkpoint**: Auth page URL parameters working - can demo independently ✅

---

## Phase 6: User Story 4 - Landing Page Feature Cards (Priority: P2)

**Goal**: Convert feature items to animated cards with responsive grid layout

**Independent Test**: Load homepage → see 3 cards with hover animations; resize → see responsive layout

### Implementation for User Story 4

- [X] T032 [P] [US4] Create FeatureCard component in book-write/src/components/FeatureCard/index.tsx
- [X] T033 [P] [US4] Create FeatureCard styles in book-write/src/components/FeatureCard/styles.module.css
- [X] T034 [US4] Implement card container with theme-aware background in FeatureCard styles
- [X] T035 [US4] Implement hover animation (translateY, shadow) in FeatureCard styles
- [X] T036 [US4] Implement entrance animation (fadeInUp) in FeatureCard styles
- [X] T037 [US4] Add staggered animation delay support in FeatureCard component
- [X] T038 [US4] Add reduced motion media query in FeatureCard styles
- [X] T039 [US4] Update HomepageFeatures to use FeatureCard in book-write/src/components/HomepageFeatures/index.tsx
- [X] T040 [US4] Update HomepageFeatures grid layout in book-write/src/components/HomepageFeatures/styles.module.css
- [X] T041 [US4] Add responsive breakpoints (3→2→1 columns) in HomepageFeatures styles
- [ ] T042 [US4] Verify 3 cards display correctly (manual test)
- [ ] T043 [US4] Verify hover animations work (manual test)
- [ ] T044 [US4] Verify responsive layout 3→2→1 columns (manual test)
- [ ] T045 [US4] Verify reduced motion disables animations (manual test)

**Checkpoint**: Landing page cards working - can demo independently ✅

---

## Phase 7: User Story 5 - Mobile Sidebar Auth Section (Priority: P2)

**Goal**: Add auth controls to mobile sidebar with user info for authenticated state

**Independent Test**: Open mobile sidebar unauthenticated → see auth links; authenticated → see user info + sign out

### Implementation for User Story 5

- [X] T046 [US5] Extend NavbarAuth with sidebar variant support in book-write/src/components/NavbarAuth/index.tsx
- [X] T047 [US5] Add sidebar-specific styles (vertical layout) in book-write/src/components/NavbarAuth/styles.module.css
- [X] T048 [US5] Implement sidebar user info display in NavbarAuth component
- [X] T049 [US5] Implement sidebar sign out button in NavbarAuth component
- [X] T050 [US5] Add onNavigate callback for sidebar close after navigation
- [X] T051 [US5] Style sidebar auth section with divider and spacing
- [X] T052 [US5] Ensure 44px minimum touch targets for mobile in sidebar styles
- [X] T053 [US5] Integrate sidebar variant into mobile sidebar (swizzle if needed)
- [ ] T054 [US5] Verify sidebar auth links visible when unauthenticated (manual test)
- [ ] T055 [US5] Verify sidebar user info + sign out when authenticated (manual test)
- [ ] T056 [US5] Verify sign out from sidebar works (manual test)

**Checkpoint**: Mobile sidebar auth working - all user stories complete ✅

---

## Phase 8: Polish & Cross-Cutting Concerns

**Purpose**: Final validation and cleanup

- [X] T057 Run npm run build in book-write/ to verify no build errors
- [ ] T058 Test all features in Chrome browser
- [ ] T059 [P] Test all features in Firefox browser
- [ ] T060 [P] Test all features in Safari browser
- [ ] T061 Test all features on mobile device or emulator
- [ ] T062 Verify no console errors in browser
- [ ] T063 Verify no layout shift during page load
- [ ] T064 Run accessibility audit (keyboard navigation, focus states)
- [ ] T065 Final light/dark mode verification across all components

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - can start immediately
- **Foundational (Phase 2)**: Depends on Phase 1 - T006 BLOCKS navbar auth integration
- **User Story 1 (Phase 3)**: Can start after Phase 1 - CSS-only, no component dependencies
- **User Story 2 (Phase 4)**: Depends on Phase 2 (T006) - needs AuthProvider at root
- **User Story 3 (Phase 5)**: Can start after Phase 1 - page-level change only
- **User Story 4 (Phase 6)**: Can start after Phase 1 - component-only, no auth dependency
- **User Story 5 (Phase 7)**: Depends on Phase 4 (T015-T021) - extends NavbarAuth component
- **Polish (Phase 8)**: Depends on all user stories complete

### User Story Dependencies

- **US1 (P0 - Sidebar CSS)**: No dependencies - CSS only
- **US2 (P1 - Navbar Auth)**: Depends on T006 (Root AuthProvider)
- **US3 (P1 - URL Params)**: No dependencies - page-level change
- **US4 (P2 - Feature Cards)**: No dependencies - component-only
- **US5 (P2 - Sidebar Auth)**: Depends on US2 (extends NavbarAuth)

### Parallel Opportunities

**After Phase 1 completes, these can run in parallel:**
- US1 (Sidebar CSS Fix) - different files
- US3 (Auth Page URL Params) - different files
- US4 (Feature Cards) - different files

**After Phase 2 completes:**
- US2 (Navbar Auth) - depends on T006

**After US2 completes:**
- US5 (Sidebar Auth) - extends NavbarAuth

---

## Parallel Example: Maximum Parallelization

```bash
# After Phase 1, launch these in parallel:
Developer A: US1 (T008-T014) - Sidebar CSS Fix
Developer B: US3 (T026-T031) - Auth Page URL Params
Developer C: US4 (T032-T045) - Feature Cards

# Phase 2 can run in parallel with above:
Developer D: T006-T007 - Root AuthProvider setup

# After Phase 2 and US4:
Developer A or B: US2 (T015-T025) - Navbar Auth

# After US2:
Any developer: US5 (T046-T056) - Sidebar Auth

# After all US complete:
All: Phase 8 (T057-T065) - Polish
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup
2. Complete Phase 3: US1 (Sidebar CSS Fix)
3. **STOP and VALIDATE**: Test sidebar in light/dark mode
4. Deploy/demo if ready - Critical bug fixed!

### Incremental Delivery

1. Phase 1 + Phase 2 → Foundation ready
2. US1 (Sidebar CSS) → Deploy (Critical fix)
3. US2 + US3 (Navbar + URL Params) → Deploy (Auth UX complete)
4. US4 (Feature Cards) → Deploy (Visual enhancement)
5. US5 (Sidebar Auth) → Deploy (Mobile UX complete)
6. Phase 8 (Polish) → Final release

### Suggested Sprint Plan

**Sprint 1 (Priority P0/P1)**:
- T001-T007: Setup + Foundation
- T008-T014: US1 - Sidebar CSS Fix
- T015-T025: US2 - Navbar Auth
- T026-T031: US3 - URL Params

**Sprint 2 (Priority P2)**:
- T032-T045: US4 - Feature Cards
- T046-T056: US5 - Sidebar Auth
- T057-T065: Polish

---

## Task Summary

| Phase | User Story | Priority | Task Count | Parallel Tasks |
|-------|------------|----------|------------|----------------|
| 1 | Setup | - | 5 | 3 |
| 2 | Foundational | - | 2 | 0 |
| 3 | US1 - Sidebar CSS | P0 | 7 | 0 |
| 4 | US2 - Navbar Auth | P1 | 11 | 2 |
| 5 | US3 - URL Params | P1 | 6 | 0 |
| 6 | US4 - Feature Cards | P2 | 14 | 2 |
| 7 | US5 - Sidebar Auth | P2 | 11 | 0 |
| 8 | Polish | - | 9 | 2 |
| **Total** | | | **65** | **9** |

---

## Notes

- [P] tasks = different files, can run in parallel
- [USn] label maps task to specific user story
- Each user story independently completable and testable
- Manual testing only (per spec - no automated tests)
- Commit after each task or logical group
- Stop at any checkpoint to validate story independently
- All file paths relative to repository root (book-write/src/...)
