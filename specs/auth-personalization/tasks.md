# Tasks: Authentication & User Personalization

**Feature**: auth-personalization
**Branch**: `better-auth`
**Date**: 2025-12-16
**Spec**: [spec.md](./spec.md) | **Plan**: [plan.md](./plan.md)

---

## Summary

Total tasks for 50-point hackathon bonus feature implementing Better-Auth authentication with user profile collection.

| Phase | Focus | Task Count |
|-------|-------|------------|
| Phase 1 | Setup | 8 |
| Phase 2 | Foundational | 6 |
| Phase 3 | US1: Signup Flow | 8 |
| Phase 4 | US2: Signin Flow | 4 |
| Phase 5 | US3: Session & Profile API | 6 |
| Phase 6 | US4: Profile Display | 4 |
| Phase 7 | Polish & Integration | 5 |
| **Total** | | **41** |

---

## User Story Mapping

| Story ID | Priority | Description | Acceptance Tests |
|----------|----------|-------------|------------------|
| US1 | P1 | Complete signup with background questions | SIGNUP-001 to SIGNUP-004 |
| US2 | P2 | Signin and session establishment | SIGNIN-001 to SIGNIN-003 |
| US3 | P2 | Session validation and profile API | SESSION-001, SESSION-002 |
| US4 | P3 | Profile display and editing | PROFILE-001, PROFILE-002 |

---

## Phase 1: Setup

**Goal**: Initialize Node.js auth service project structure.

**Exit Criteria**: Project compiles, dependencies installed, environment configured.

### Tasks

- [X] T001 Create auth directory structure per plan in /auth/
- [X] T002 [P] Initialize Node.js project with package.json in /auth/package.json
- [X] T003 [P] Create TypeScript configuration in /auth/tsconfig.json
- [X] T004 [P] Create environment variables template in /auth/.env.example
- [X] T005 Install core dependencies (better-auth, drizzle-orm, hono, @neondatabase/serverless) in /auth/
- [X] T006 Install dev dependencies (typescript, tsx, drizzle-kit, @types/node) in /auth/
- [X] T007 Create Drizzle configuration in /auth/drizzle.config.ts
- [X] T008 Add .gitignore for auth service in /auth/.gitignore

---

## Phase 2: Foundational

**Goal**: Set up database connection and Better-Auth configuration (blocking for all user stories).

**Exit Criteria**: Better-Auth endpoints respond, database connected, health check works.

### Tasks

- [X] T009 Create Neon PostgreSQL client in /auth/src/db/client.ts
- [X] T010 Create Drizzle schema for user_profiles in /auth/src/db/schema.ts
- [X] T011 Configure Better-Auth with Neon adapter in /auth/src/better-auth.ts
- [X] T012 Create Hono server entry point in /auth/src/index.ts
- [X] T013 Implement health check endpoint in /auth/src/routes/health.ts
- [ ] T014 Run database migration to create user_profiles table

---

## Phase 3: US1 - Signup Flow (P1)

**Goal**: User can complete signup with email, password, and mandatory background questions.

**Exit Criteria**: SIGNUP-001 to SIGNUP-004 acceptance tests pass.

**Independent Test**: Complete signup creates Better-Auth user AND profile record.

### Tasks

- [X] T015 [US1] Create API types for profile requests in /auth/src/types/api.ts
- [X] T016 [US1] Implement input validation helpers in /auth/src/utils/validation.ts
- [X] T017 [US1] Implement POST /api/profile endpoint in /auth/src/routes/profile.ts
- [X] T018 [US1] Add profile creation with rollback support in /auth/src/routes/profile.ts
- [X] T019 [US1] Install @better-auth/react in /book-write/
- [X] T020 [US1] Create AuthProvider with Better-Auth client in /book-write/src/components/Auth/AuthProvider.tsx
- [X] T021 [US1] Create SignupForm component with background questions in /book-write/src/components/Auth/SignupForm.tsx
- [X] T022 [US1] Implement signup flow with rollback on profile failure in /book-write/src/components/Auth/SignupForm.tsx

---

## Phase 4: US2 - Signin Flow (P2)

**Goal**: Existing user can signin and establish session.

**Exit Criteria**: SIGNIN-001 to SIGNIN-003 acceptance tests pass.

**Independent Test**: User can signin, session cookie set, profile retrieved.

### Tasks

- [X] T023 [US2] Create SigninForm component in /book-write/src/components/Auth/SigninForm.tsx
- [X] T024 [US2] Add signin form validation and error handling in /book-write/src/components/Auth/SigninForm.tsx
- [X] T025 [US2] Implement post-signin profile fetch in AuthProvider in /book-write/src/components/Auth/AuthProvider.tsx
- [X] T026 [US2] Add signout functionality in AuthProvider in /book-write/src/components/Auth/AuthProvider.tsx

---

## Phase 5: US3 - Session & Profile API (P2)

**Goal**: Protected endpoints validate session and return profile data.

**Exit Criteria**: SESSION-001, SESSION-002 acceptance tests pass.

**Independent Test**: Valid session returns profile; invalid session returns 401.

### Tasks

- [X] T027 [US3] Implement session validation middleware in /auth/src/middleware/auth.ts
- [X] T028 [US3] Implement GET /api/profile endpoint in /auth/src/routes/profile.ts
- [X] T029 [US3] Add session context to profile response in /auth/src/routes/profile.ts
- [X] T030 [US3] Implement error response format per spec in /auth/src/utils/errors.ts
- [X] T031 [US3] Apply auth middleware to protected routes in /auth/src/index.ts
- [X] T032 [US3] Handle 401 responses in frontend AuthProvider in /book-write/src/components/Auth/AuthProvider.tsx

---

## Phase 6: US4 - Profile Display (P3)

**Goal**: Signed-in user can view and edit their profile.

**Exit Criteria**: PROFILE-001, PROFILE-002 acceptance tests pass.

**Independent Test**: User can view profile, update fields, see updated values.

### Tasks

- [X] T033 [US4] Implement PUT /api/profile endpoint in /auth/src/routes/profile.ts
- [X] T034 [US4] Create UserProfile display component in /book-write/src/components/Profile/UserProfile.tsx
- [X] T035 [US4] Add profile edit mode to UserProfile component in /book-write/src/components/Profile/UserProfile.tsx
- [X] T036 [US4] Add profile page/route in Docusaurus in /book-write/src/pages/profile.tsx

---

## Phase 7: Polish & Integration

**Goal**: Complete integration, error handling, and final testing.

**Exit Criteria**: All acceptance tests pass, bonus point criteria met.

### Tasks

- [X] T037 Add CORS configuration for frontend URL in /auth/src/index.ts
- [X] T038 Add loading states to auth components in /book-write/src/components/Auth/
- [X] T039 Add auth state persistence across page navigation in /book-write/src/components/Auth/AuthProvider.tsx
- [ ] T040 Add navigation guards for protected pages in /book-write/
- [ ] T041 Final integration test: complete signup → signin → profile view → profile edit flow

---

## Dependencies

### User Story Dependencies

```
US1 (Signup) ──────────────────────────────────────┐
                                                   │
US2 (Signin) ─────────────► US3 (Session/API) ────►│──► US4 (Profile Display)
                                                   │
Phase 1 (Setup) ──► Phase 2 (Foundational) ────────┘
```

### Critical Path

1. **Phase 1** → **Phase 2** (blocking: database and Better-Auth must be configured)
2. **Phase 2** → **US1** (blocking: endpoints must exist for signup)
3. **US1** + **US3** → **US2** (signin needs session validation)
4. **US2** + **US3** → **US4** (profile display needs signed-in state)

### Parallel Opportunities

| Tasks | Can Run In Parallel | Reason |
|-------|---------------------|--------|
| T002, T003, T004 | ✅ Yes | Independent config files |
| T009, T010, T011 | ✅ Yes | Independent modules |
| T015, T016 | ✅ Yes | Types and validation are independent |
| T019, T020 | ❌ No | AuthProvider depends on package install |
| T023, T027 | ✅ Yes | Frontend and backend can develop in parallel |
| T033, T034 | ✅ Yes | API and UI can develop in parallel |

---

## Implementation Strategy

### MVP Scope (US1 Only)

For fastest path to partial functionality:
1. Complete Phase 1 + Phase 2
2. Complete Phase 3 (US1: Signup)
3. Verify: User can signup with background questions

**Note**: Hackathon requires ALL bonus criteria. MVP is for development milestone only.

### Incremental Delivery

| Milestone | Tasks Complete | Functionality |
|-----------|----------------|---------------|
| M1 | T001-T014 | Auth service running, health check works |
| M2 | T015-T022 | Signup flow complete |
| M3 | T023-T032 | Signin and session validation complete |
| M4 | T033-T036 | Profile view/edit complete |
| M5 | T037-T041 | Polish and integration complete |

---

## Acceptance Test Mapping

| Test ID | Verifies | Tasks Required |
|---------|----------|----------------|
| SIGNUP-001 | Successful signup | T015-T022 |
| SIGNUP-002 | Missing field rejection | T016, T017 |
| SIGNUP-003 | Duplicate email rejection | Better-Auth handles |
| SIGNUP-004 | Rollback on profile failure | T018, T022 |
| SIGNIN-001 | Successful signin | T023-T025 |
| SIGNIN-002 | Wrong password rejection | Better-Auth handles |
| SIGNIN-003 | Profile retrieval after signin | T025, T028 |
| SESSION-001 | Valid session access | T027, T028 |
| SESSION-002 | Invalid session 401 | T027, T030 |
| PROFILE-001 | Profile update | T033, T035 |
| PROFILE-002 | Invalid update rejection | T016, T033 |

---

## File Path Reference

### Auth Backend (/auth/)

| File | Purpose | Created By |
|------|---------|------------|
| /auth/package.json | Node.js project config | T002 |
| /auth/tsconfig.json | TypeScript config | T003 |
| /auth/.env.example | Environment template | T004 |
| /auth/drizzle.config.ts | Drizzle ORM config | T007 |
| /auth/src/index.ts | Hono server entry | T012 |
| /auth/src/better-auth.ts | Better-Auth config | T011 |
| /auth/src/db/client.ts | Neon PostgreSQL client | T009 |
| /auth/src/db/schema.ts | Drizzle schema | T010 |
| /auth/src/types/api.ts | API type definitions | T015 |
| /auth/src/utils/validation.ts | Input validation | T016 |
| /auth/src/utils/errors.ts | Error formatting | T030 |
| /auth/src/middleware/auth.ts | Session middleware | T027 |
| /auth/src/routes/health.ts | Health check endpoint | T013 |
| /auth/src/routes/profile.ts | Profile CRUD endpoints | T017, T028, T033 |

### Frontend (/book-write/)

| File | Purpose | Created By |
|------|---------|------------|
| /book-write/src/components/Auth/AuthProvider.tsx | Auth context | T020 |
| /book-write/src/components/Auth/SignupForm.tsx | Signup with background | T021 |
| /book-write/src/components/Auth/SigninForm.tsx | Signin form | T023 |
| /book-write/src/components/Profile/UserProfile.tsx | Profile display/edit | T034 |
| /book-write/src/pages/profile.tsx | Profile page | T036 |

---

## Bonus Point Checklist

All items must be complete for 50 bonus points:

- [X] Signup implemented using Better-Auth (T011, T021, T022)
- [X] Signin implemented using Better-Auth (T011, T023, T024)
- [X] Background questions mandatory and non-skippable (T016, T017, T021)
- [X] User profile data stored in Neon PostgreSQL (T010, T014, T017)
- [X] Profile data available for future personalization (T028, T034)

---

**END OF TASKS**
