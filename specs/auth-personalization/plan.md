# Implementation Plan: Authentication & User Personalization

**Branch**: `better-auth` | **Date**: 2025-12-16 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `specs/auth-personalization/spec.md`

---

## Summary

Implement a **50-point hackathon bonus feature** for user authentication and personalization using Better-Auth (Node.js). The feature enables signup/signin with mandatory background questions, stores user profiles in Neon PostgreSQL, and makes profile data available for future personalization.

**Key Requirements**:
1. Authentication via Better-Auth exclusively (no alternatives)
2. Mandatory background questions at signup (software + hardware)
3. Profile data stored in Neon PostgreSQL
4. Separate auth service (Node.js) from existing chatbot backend (Python)
5. Frontend integration in Docusaurus

---

## Technical Context

**Language/Version**: TypeScript 5.x / Node.js 18+
**Primary Dependencies**: Better-Auth, Drizzle ORM, Hono, @neondatabase/serverless
**Storage**: Neon Serverless PostgreSQL (shared with chatbot)
**Testing**: Vitest (unit), Playwright (e2e)
**Target Platform**: Node.js server + Docusaurus frontend
**Project Type**: Web application (separate auth backend + existing frontend)
**Performance Goals**: Profile creation <3s, Profile fetch <1s, Session validation <500ms
**Constraints**: Chatbot backend (`/backend/`) MUST NOT be modified
**Scale/Scope**: Single-tenant, <1000 users for hackathon

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| V. Authentication-First Access Control | ✅ PASS | Personalization requires auth |
| VII. Authentication & User Profile Standard | ✅ PASS | Better-Auth, mandatory questions |
| I. Accuracy and Verifiability | ✅ PASS | No fabricated APIs |
| IV. Reproducibility | ✅ PASS | Documented setup steps |
| V. API and Service Compliance | ✅ PASS | Better-Auth ToS compliant |

**Constitution Gate**: ✅ PASSED

---

## Project Structure

### Documentation (this feature)

```text
specs/auth-personalization/
├── spec.md              # Feature specification (v2.1)
├── plan.md              # This file
├── research.md          # Phase 0 output - technology decisions
├── data-model.md        # Phase 1 output - database schema
├── quickstart.md        # Phase 1 output - setup guide
├── contracts/           # Phase 1 output - API contracts
│   └── openapi.yaml     # OpenAPI 3.1 specification
└── tasks.md             # Phase 2 output (created by /sp.tasks)
```

### Source Code (repository root)

```text
auth/                              # NEW - Authentication service (Node.js)
├── src/
│   ├── index.ts                   # Entry point (Hono server)
│   ├── better-auth.ts             # Better-Auth configuration
│   ├── routes/
│   │   ├── profile.ts             # Profile CRUD endpoints
│   │   └── health.ts              # Health check
│   └── db/
│       ├── client.ts              # Neon PostgreSQL client
│       └── schema.ts              # Drizzle schema (user_profiles only)
├── tests/
│   ├── unit/
│   │   └── validation.test.ts     # Input validation tests
│   └── integration/
│       └── profile.test.ts        # Profile API tests
├── package.json
├── tsconfig.json
├── drizzle.config.ts
└── .env.example

backend/                           # EXISTING - DO NOT MODIFY
└── (Python RAG chatbot)

book-write/                        # Docusaurus frontend
└── src/
    └── components/
        ├── Auth/
        │   ├── AuthProvider.tsx   # NEW - Auth context
        │   ├── SignupForm.tsx     # NEW - Signup with background
        │   └── SigninForm.tsx     # NEW - Signin form
        └── Profile/
            └── UserProfile.tsx    # NEW - Profile display/edit
```

**Structure Decision**: Web application with separate auth backend. Auth service is a new Node.js project in `/auth/`. Frontend components added to existing Docusaurus in `/book-write/`.

---

## Phase 0: Research Summary

**Status**: ✅ Complete (see [research.md](./research.md))

All technology decisions resolved:

| Decision Area | Choice | Confidence |
|---------------|--------|------------|
| Auth Provider | Better-Auth (Node.js) | High |
| Database Adapter | Drizzle ORM | High |
| Session Strategy | Cookie-based, 7 days | High |
| Schema Ownership | App owns only user_profiles | High |
| Signup Flow | Two-step with rollback | High |
| Frontend Integration | React in Docusaurus | High |
| CORS | Credentials-enabled | High |
| HTTP Framework | Hono | High |

**No NEEDS CLARIFICATION items remain.**

---

## Phase 1: Design Summary

**Status**: ✅ Complete

### Data Model

See [data-model.md](./data-model.md) for full schema.

**Application-Owned Table**: `user_profiles`

| Column | Type | Notes |
|--------|------|-------|
| id | UUID | PK, auto-generated |
| auth_user_id | VARCHAR(255) | UK, references Better-Auth |
| programming_languages | TEXT[] | Required, min 1 |
| frameworks_platforms | TEXT[] | Required, min 1 |
| experience_level | VARCHAR(20) | Enum: beginner/intermediate/advanced/expert |
| device_type | VARCHAR(20) | Enum: desktop/laptop/tablet/mobile/embedded |
| operating_system | VARCHAR(20) | Enum: windows/macos/linux/other |
| system_capability | VARCHAR(10) | Enum: low/medium/high |
| created_at | TIMESTAMPTZ | Auto-set |
| updated_at | TIMESTAMPTZ | Auto-updated |

### API Contracts

See [contracts/openapi.yaml](./contracts/openapi.yaml) for full OpenAPI spec.

**Application Endpoints**:

| Method | Endpoint | Auth | Purpose |
|--------|----------|------|---------|
| POST | /api/profile | Yes | Create profile after signup |
| GET | /api/profile | Yes | Fetch current user's profile |
| PUT | /api/profile | Yes | Update current user's profile |
| GET | /api/health | No | Health check |

**Better-Auth Endpoints** (auto-generated):

| Method | Endpoint | Purpose |
|--------|----------|---------|
| POST | /api/auth/sign-up/email | Create new user |
| POST | /api/auth/sign-in/email | Authenticate user |
| POST | /api/auth/sign-out | End session |
| GET | /api/auth/session | Get current session |

### Quickstart

See [quickstart.md](./quickstart.md) for setup instructions.

---

## Phase 2: Implementation Phases

### Phase 2.1: Auth Backend Setup

**Scope**: Initialize Node.js project, configure Better-Auth, create database schema.

**Tasks**:
1. Initialize `/auth/` Node.js project with TypeScript
2. Install dependencies (better-auth, drizzle-orm, hono, @neondatabase/serverless)
3. Configure Better-Auth with Neon PostgreSQL
4. Create Drizzle schema for user_profiles
5. Run database migration
6. Implement health check endpoint

**Exit Criteria**: Auth service starts, Better-Auth endpoints respond, database migrated.

### Phase 2.2: Profile API

**Scope**: Implement profile CRUD endpoints with validation.

**Tasks**:
1. Implement session validation middleware (calls Better-Auth API)
2. Implement POST /api/profile (create)
3. Implement GET /api/profile (read)
4. Implement PUT /api/profile (update)
5. Add input validation for all endpoints
6. Add error handling with consistent error format

**Exit Criteria**: All profile endpoints pass acceptance tests.

### Phase 2.3: Frontend Auth Components

**Scope**: Create React components for auth flow in Docusaurus.

**Tasks**:
1. Install @better-auth/react in book-write
2. Create AuthProvider with Better-Auth client
3. Create SignupForm with background questions (multi-step)
4. Create SigninForm
5. Implement signup flow with rollback on profile failure
6. Add auth state to Docusaurus layout

**Exit Criteria**: User can complete signup with background questions.

### Phase 2.4: Profile Display

**Scope**: Create profile display and edit UI.

**Tasks**:
1. Create UserProfile component
2. Display all background fields
3. Implement profile edit functionality
4. Add navigation to profile page

**Exit Criteria**: Signed-in user can view and edit profile.

### Phase 2.5: Integration Testing

**Scope**: End-to-end testing of complete auth flow.

**Tasks**:
1. Test complete signup flow (auth + profile)
2. Test signin and profile retrieval
3. Test profile update
4. Test session expiration handling
5. Test rollback on profile creation failure
6. Test error states (invalid input, duplicate email)

**Exit Criteria**: All acceptance tests from spec pass.

---

## Complexity Tracking

> No Constitution Check violations requiring justification.

| Aspect | Complexity Level | Justification |
|--------|------------------|---------------|
| Separate auth service | Medium | Required by constitution (chatbot MUST NOT be modified) |
| Two-step signup | Low | Standard compensating transaction pattern |
| Frontend components | Low | 4 simple React components |

---

## Risk Mitigation

| Risk | Mitigation |
|------|------------|
| Better-Auth API changes | Pin to specific version, test early |
| Neon connection limits | Use connection pooling, monitor |
| CORS issues | Test cross-origin early, document config |
| Rollback failures | Log failed rollbacks, manual cleanup process |

---

## Success Metrics

| Metric | Target | Measurement |
|--------|--------|-------------|
| Signup completion | 100% with valid data | Acceptance test SIGNUP-001 |
| Signup rejection | 100% with invalid data | Acceptance test SIGNUP-002 |
| Profile creation latency | <3 seconds | Performance test |
| Session validation latency | <500ms | Performance test |
| Bonus points | 50/50 | Hackathon evaluation |

---

## Next Steps

1. Run `/sp.tasks` to generate detailed task breakdown
2. Begin Phase 2.1: Auth Backend Setup
3. Commit plan artifacts to repository

---

**END OF IMPLEMENTATION PLAN**
