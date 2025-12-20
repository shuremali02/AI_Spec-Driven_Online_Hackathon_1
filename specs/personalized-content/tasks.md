# Tasks: Chapter Content Personalization (Bonus Feature)

**Input**: Design documents from `/specs/personalized-content/`
**Prerequisites**: plan.md, spec.md, data-model.md, contracts/personalize-api.md
**Feature Branch**: `personalized-feature`
**Date**: 2025-12-19

---

## Summary

| Metric | Value |
|--------|-------|
| Total Tasks | 22 |
| Setup Phase | 3 tasks |
| Foundational Phase | 4 tasks |
| US1 (Access Control) | 2 tasks |
| US2 (Personalization) | 6 tasks |
| US3 (Toggle) | 2 tasks |
| US4 (Error Handling) | 2 tasks |
| Polish Phase | 3 tasks |
| Parallel Opportunities | 12 tasks |

---

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Which user story this task belongs to (US1, US2, US3, US4)
- Paths use existing project structure from plan.md

---

## Phase 1: Setup (Shared Infrastructure)

**Purpose**: Environment configuration and dependencies

- [X] T001 Add personalization config to backend/config.py (PERSONALIZE_MAX_TOKENS, PERSONALIZE_RATE_LIMIT, PERSONALIZE_TIMEOUT, AUTH_BACKEND_URL)
- [X] T002 [P] Add httpx dependency to backend/pyproject.toml for async HTTP calls to auth backend
- [X] T003 [P] Verify existing environment variables (OPENAI_API_KEY, OPENAI_BASE_URL, GEMINI_MODEL, DATABASE_URL) in .env.example

---

## Phase 2: Foundational (Blocking Prerequisites)

**Purpose**: Core infrastructure that MUST be complete before user story implementation

**⚠️ CRITICAL**: No user story work can begin until this phase is complete

- [X] T004 Create Pydantic models for PersonalizationRequest and PersonalizationResponse in backend/api/personalize_models.py
- [X] T005 [P] Create rate limiting utility with RateLimitEntry dataclass in backend/api/rate_limiter.py
- [X] T006 [P] Create session validation helper using httpx in backend/api/auth_helper.py
- [X] T007 Create profile query function in backend/db/profile_queries.py (SELECT from user_profiles by auth_user_id)

**Checkpoint**: Foundation ready - user story implementation can begin

---

## Phase 3: User Story 1 - Access Control (Priority: P1) 🎯 MVP

**Goal**: Personalization button appears ONLY for authenticated users with complete profiles

**Independent Test**:
- Sign out → button NOT visible
- Sign in (no profile) → button NOT visible
- Sign in (complete profile) → button IS visible

### Implementation for User Story 1

- [X] T008 [US1] Verify PersonalizeButton visibility logic in book-write/src/components/Personalization/PersonalizeButton.tsx (must check isAuthenticated AND hasProfile)
- [X] T009 [US1] Verify ChapterPersonalizer auth + profile checks in book-write/src/components/Personalization/ChapterPersonalizer.tsx

**Checkpoint**: Access control working - button visibility is correct for all user states

---

## Phase 4: User Story 2 - Personalization (Priority: P2)

**Goal**: User clicks button → content is personalized using their profile data → transformed content is displayed

**Independent Test**:
- Authenticated user with profile
- Click "Personalize Content"
- Content transforms with experience-appropriate language
- Programming language notes appear
- Hardware capability warnings appear

### Implementation for User Story 2

- [X] T010 [P] [US2] Create PersonalizeContentAgent class in backend/agents/personalize_agent.py following RAG agent pattern (AsyncOpenAI + Gemini)
- [X] T011 [P] [US2] Define system prompt with MUST DO/MUST NOT rules in backend/agents/personalize_agent.py
- [X] T012 [US2] Implement personalize() method in PersonalizeContentAgent (build user prompt with profile context, call LLM, extract response)
- [X] T013 [US2] Create POST /api/personalize endpoint in backend/api/personalize.py with full request handling
- [X] T014 [US2] Add personalize router to backend/main.py (import and include with /api prefix)
- [X] T015 [US2] Update frontend API URL in book-write/src/components/Personalization/types.ts to support configurable FastAPI backend

**Checkpoint**: Personalization working end-to-end - content transforms based on profile

---

## Phase 5: User Story 3 - Toggle (Priority: P3)

**Goal**: User can toggle between original and personalized content instantly

**Independent Test**:
- After personalization, button shows "Show Original"
- Click → original content appears
- Button shows "Show Personalized"
- Click → cached personalized content appears instantly

### Implementation for User Story 3

- [X] T016 [US3] Verify toggle state management in book-write/src/components/Personalization/PersonalizeButton.tsx (showPersonalized state, cached content)
- [X] T017 [US3] Verify PersonalizedContent component renders correctly in book-write/src/components/Personalization/PersonalizedContent.tsx

**Checkpoint**: Toggle working - instant switch between original and personalized content

---

## Phase 6: User Story 4 - Error Handling (Priority: P4)

**Goal**: Errors are handled gracefully with user-friendly messages and retry options

**Independent Test**:
- Rate limit exceeded → shows countdown
- API failure → shows retry button
- Incomplete profile → lists missing fields
- Original content remains visible on any error

### Implementation for User Story 4

- [X] T018 [US4] Implement rate limiting in backend/api/personalize.py (5 req/min/user, return 429 with retry_after)
- [X] T019 [US4] Verify error handling in book-write/src/components/Personalization/PersonalizeButton.tsx (handle 401, 404, 400, 429, 500 codes)

**Checkpoint**: Error handling complete - graceful degradation for all error scenarios

---

## Phase 7: Polish & Cross-Cutting Concerns

**Purpose**: Validation, documentation, and final integration

- [X] T020 [P] CORS already configured in backend/main.py for frontend origin (uses ALLOWED_ORIGINS env var)
- [X] T021 Run end-to-end test following quickstart.md validation steps
- [X] T022 Verify all 15 items in bonus eligibility checklist (spec.md Section 9.1)

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup) ─────────────────────────────────────────────┐
    │                                                        │
    ▼                                                        │
Phase 2 (Foundational) ──────────────────────────────────────┤
    │                                                        │
    ├──────────────────┬────────────────┬───────────────┐   │
    ▼                  ▼                ▼               ▼   │
Phase 3 (US1)    Phase 4 (US2)    Phase 5 (US3)   Phase 6 (US4)
(Access Control)  (Personalize)    (Toggle)       (Error Handle)
    │                  │                │               │    │
    └──────────────────┴────────────────┴───────────────┘   │
                           │                                  │
                           ▼                                  │
                     Phase 7 (Polish) ◄───────────────────────┘
```

### User Story Dependencies

| Story | Depends On | Can Parallelize |
|-------|------------|-----------------|
| US1 (Access Control) | Phase 2 | Yes - starts immediately after Phase 2 |
| US2 (Personalization) | Phase 2 | Yes - can run parallel with US1 |
| US3 (Toggle) | US2 | No - needs personalization to test toggle |
| US4 (Error Handling) | US2 | No - needs endpoint to test errors |

### Critical Path

```
T001 → T004 → T010 → T012 → T013 → T014 → T015 → T021 → T022
```

(Setup config → Models → Agent class → Agent method → Endpoint → Main router → Frontend URL → E2E test → Bonus verification)

---

## Parallel Opportunities

### Within Phase 1 (Setup)
```bash
# Can run in parallel:
- T002: Add httpx dependency
- T003: Verify environment variables
```

### Within Phase 2 (Foundational)
```bash
# Can run in parallel:
- T005: Rate limiter utility
- T006: Session validation helper
```

### Within Phase 4 (US2 - Personalization)
```bash
# Can run in parallel:
- T010: PersonalizeContentAgent class
- T011: System prompt definition
```

### Across User Stories (after Phase 2)
```bash
# With multiple developers:
# Developer A: US1 (T008-T009)
# Developer B: US2 (T010-T015)
# Then sequentially: US3 (T016-T017), US4 (T018-T019)
```

---

## Implementation Strategy

### MVP First (US1 + US2)

1. Complete Phase 1: Setup (T001-T003)
2. Complete Phase 2: Foundational (T004-T007)
3. Complete Phase 3: US1 - Access Control (T008-T009)
4. Complete Phase 4: US2 - Personalization (T010-T015)
5. **STOP and VALIDATE**: Test personalization works end-to-end
6. ✅ MVP ready for demo

### Full Implementation

1. MVP (Phases 1-4) → Test
2. Add US3: Toggle (T016-T017) → Test
3. Add US4: Error Handling (T018-T019) → Test
4. Polish (T020-T022) → Final validation
5. ✅ Full feature ready for 50-point bonus verification

---

## File Summary

### New Files to Create

| File | Phase | Purpose |
|------|-------|---------|
| backend/api/models.py | P2 | Pydantic request/response models |
| backend/api/rate_limiter.py | P2 | Rate limiting utility |
| backend/api/auth_helper.py | P2 | Session validation helper |
| backend/db/profile_queries.py | P2 | Profile database query |
| backend/agents/personalize_agent.py | P4 (US2) | PersonalizeContentAgent |
| backend/api/personalize.py | P4 (US2) | FastAPI endpoint |

### Files to Modify

| File | Phase | Purpose |
|------|-------|---------|
| backend/config.py | P1 | Add personalization config |
| backend/pyproject.toml | P1 | Add httpx dependency |
| backend/main.py | P4, P7 | Add router, CORS |
| book-write/src/components/Personalization/utils.ts | P4 | Update API URL |

### Files to Verify (Existing)

| File | Phase | Purpose |
|------|-------|---------|
| book-write/src/components/Personalization/PersonalizeButton.tsx | P3, P6 | Button logic |
| book-write/src/components/Personalization/ChapterPersonalizer.tsx | P3 | Auth checks |
| book-write/src/components/Personalization/PersonalizedContent.tsx | P5 | Content render |

---

## Bonus Eligibility Checklist Tasks

Task T022 must verify all 15 items:

| # | Requirement | Verification |
|---|-------------|--------------|
| 1 | User can sign in via Better-Auth | Sign in test |
| 2 | User profile with software/hardware data | DB check |
| 3 | "Personalize Content" button at chapter start | Visual |
| 4 | Button NOT visible when signed out | Sign out test |
| 5 | Button triggers personalization | Click test |
| 6 | Uses experience_level | Compare outputs |
| 7 | Uses programming_languages | Check notes |
| 8 | Uses hardware background | Check warnings |
| 9 | Personalized content displayed | UI shows change |
| 10 | "Show Original" toggle works | Toggle test |
| 11 | Code blocks unchanged | Diff test |
| 12 | Technical facts preserved | Review |
| 13 | FastAPI backend | Check endpoint |
| 14 | OpenAI Agents SDK pattern | Check agent code |
| 15 | Gemini LLM via OpenAI-compatible | Check client config |

---

## Notes

- All tasks follow format: `- [ ] [TaskID] [P?] [Story?] Description with file path`
- [P] tasks = different files, no dependencies
- [Story] label maps task to user story for traceability
- No test tasks included (not explicitly requested in spec)
- Existing frontend components need verification, not rewrite
- Critical path: ~10 tasks for MVP, ~22 tasks for full feature
- Commit after each task or logical group

---

**END OF TASKS**
