# Tasks: Personalized Chatbot Greeting

**Input**: Design documents from `/specs/001-chatbot-greeting/`
**Prerequisites**: plan.md (required), spec.md (required for user stories), research.md, quickstart.md

**Tests**: Manual testing only (no automated tests requested in spec)

**Organization**: Tasks are grouped by user story to enable independent implementation and testing of each story.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (e.g., US1, US2, US3)
- Include exact file paths in descriptions

## Path Conventions

- **Frontend**: `book-write/src/components/Chatbot/`
- **Auth**: `book-write/src/components/Auth/` (existing, read-only)

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Verify existing infrastructure is ready for feature implementation

- [x] T001 Verify AuthProvider exports useAuth hook correctly in book-write/src/components/Auth/AuthProvider.tsx
- [x] T002 Verify ChatWindow.tsx current structure matches plan expectations in book-write/src/components/Chatbot/ChatWindow.tsx

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: No foundational work needed - feature uses existing infrastructure

**⚠️ NOTE**: This feature requires NO foundational work. Existing AuthProvider and ChatWindow provide all necessary infrastructure.

**Checkpoint**: Foundation ready - user story implementation can now begin

---

## Phase 3: User Story 1 - Logged-in User Sees Personalized Greeting (Priority: P1) 🎯 MVP

**Goal**: Display personalized greeting with user's name and time-appropriate message when logged-in user opens chatbot

**Independent Test**: Log in as a user with a known name, open the chatbot, verify greeting displays "Good [morning/afternoon/evening], [Name]! How can I help you today?"

### Implementation for User Story 1

- [x] T003 [US1] Add useAuth import to ChatWindow.tsx in book-write/src/components/Chatbot/ChatWindow.tsx
- [x] T004 [US1] Create getPersonalizedGreeting helper function in book-write/src/components/Chatbot/ChatWindow.tsx
- [x] T005 [US1] Add useAuth hook call inside ChatWindow component in book-write/src/components/Chatbot/ChatWindow.tsx
- [x] T006 [US1] Replace static welcome message with personalized greeting in book-write/src/components/Chatbot/ChatWindow.tsx
- [ ] T007 [US1] Manual test: Verify logged-in user sees personalized greeting with name

**Checkpoint**: At this point, User Story 1 should be fully functional - logged-in users see personalized greeting

---

## Phase 4: User Story 2 - Anonymous User Sees Generic Greeting (Priority: P2)

**Goal**: Display time-appropriate greeting without name for users who are not logged in

**Independent Test**: Open chatbot without logging in, verify greeting displays "Good [morning/afternoon/evening]! How can I help you today?"

### Implementation for User Story 2

- [ ] T008 [US2] Verify getPersonalizedGreeting handles null/undefined userName in book-write/src/components/Chatbot/ChatWindow.tsx
- [ ] T009 [US2] Manual test: Verify anonymous user sees generic greeting without name

**Checkpoint**: At this point, User Stories 1 AND 2 should both work - personalized for logged-in, generic for anonymous

---

## Phase 5: User Story 3 - User Name Display from Profile (Priority: P3)

**Goal**: Use user's display name (first name) from profile for natural greeting

**Independent Test**: Log in with users having different name formats, verify chatbot uses appropriate name

### Implementation for User Story 3

- [ ] T010 [US3] Review user.name format from AuthProvider in book-write/src/components/Auth/AuthProvider.tsx
- [ ] T011 [US3] Update getPersonalizedGreeting to extract first name if full name provided in book-write/src/components/Chatbot/ChatWindow.tsx (optional - depends on name format)
- [ ] T012 [US3] Manual test: Verify greeting uses appropriate name format

**Checkpoint**: All user stories should now be independently functional

---

## Phase 6: Polish & Cross-Cutting Concerns

**Purpose**: Final verification and optional styling

- [x] T013 [P] Optional: Add CSS styling for greeting in book-write/src/components/Chatbot/ChatWindow.css
- [ ] T014 Manual test: Verify all time periods work correctly (morning, afternoon, evening)
- [ ] T015 Manual test: Verify greeting renders within 100ms (no perceptible delay)
- [ ] T016 Run quickstart.md validation steps

---

## Dependencies & Execution Order

### Phase Dependencies

- **Setup (Phase 1)**: No dependencies - verification only
- **Foundational (Phase 2)**: N/A - no foundational work needed
- **User Stories (Phase 3+)**: Can start immediately after Setup verification
  - User Story 1 is MVP - complete first
  - User Story 2 depends on US1 implementation (same helper function)
  - User Story 3 is enhancement to US1
- **Polish (Phase 6)**: Depends on all user stories being complete

### User Story Dependencies

- **User Story 1 (P1)**: No dependencies - can start immediately
- **User Story 2 (P2)**: Shares implementation with US1 - verify null handling
- **User Story 3 (P3)**: Enhancement to US1 - may not need changes if name format is simple

### Within Each User Story

- Implementation before manual testing
- Core functionality before edge cases
- Story complete before moving to next priority

### Parallel Opportunities

- T001 and T002 (Setup verification) can run in parallel
- T013 (CSS styling) can run in parallel with any implementation task
- User Story testing (T007, T009, T012) must be sequential after implementation

---

## Parallel Example: User Story 1

```bash
# Implementation is sequential (same file):
Task T003: Add useAuth import
Task T004: Create getPersonalizedGreeting helper
Task T005: Add useAuth hook call
Task T006: Replace welcome message

# Then test:
Task T007: Manual test verification
```

---

## Implementation Strategy

### MVP First (User Story 1 Only)

1. Complete Phase 1: Setup verification
2. Complete Phase 3: User Story 1 (T003-T007)
3. **STOP and VALIDATE**: Test User Story 1 independently
4. Deploy/demo if ready - basic personalization working!

### Incremental Delivery

1. Complete US1 → Deploy (MVP: personalized greeting for logged-in users)
2. Verify US2 works → Deploy (anonymous users also have time-based greeting)
3. Complete US3 if needed → Deploy (name format optimization)
4. Polish phase → Final deploy

### Estimated Time

- Total tasks: 16
- MVP (US1 only): 7 tasks (~15-30 minutes)
- Full implementation: 16 tasks (~30-60 minutes)

---

## Notes

- All implementation happens in a single file: ChatWindow.tsx
- No API contracts needed - frontend-only feature
- Manual testing is sufficient - no automated tests requested
- Feature follows existing patterns (ChapterPersonalizer uses same useAuth hook)
- Commit after each logical group (Setup, US1, US2, US3, Polish)
