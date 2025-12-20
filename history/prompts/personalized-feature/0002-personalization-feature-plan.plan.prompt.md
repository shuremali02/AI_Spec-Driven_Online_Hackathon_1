---
id: 0002
title: Chapter Content Personalization Implementation Plan
stage: plan
date: 2025-12-18
surface: agent
model: claude-sonnet-4-5-20250929
feature: personalized-feature
branch: translation-feature
user: shuremali
command: /sp.plan
labels: ["personalization", "bonus-feature", "implementation-plan", "architecture"]
links:
  spec: specs/personalized-feature/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/personalized-feature/plan.md
tests:
  - null
---

## Prompt

/sp.plan carefully read spec.md to create planning

## Response snapshot

Created comprehensive implementation plan for Chapter Content Personalization feature (`specs/personalized-feature/plan.md`) including:

**Key Architectural Decisions:**
1. **Backend Placement**: Add `/api/personalize` endpoint to existing `/auth/` service to reuse session validation and profile access
2. **AI Provider**: Use Google Gemini API (gemini-2.0-flash) for content personalization
3. **State Management**: Component-level React state with caching (no database persistence)
4. **Profile Validation**: Require complete profile (all 6 fields) for personalization

**Implementation Phases:**
- Phase 1: Backend API (8 tasks) - route, rate limiting, profile fetch, Gemini integration
- Phase 2: Frontend Components (5 tasks) - PersonalizeButton, PersonalizedContent, CSS, types
- Phase 3: Integration (4 tasks) - chapter integration, state management, caching
- Phase 4: Polish & Testing (5 tasks) - responsive, profile testing, code block verification

**Technical Approach:**
- Frontend mirrors TranslateButton pattern for consistency
- Backend fetches chapter content server-side (avoids large client payloads)
- Gemini API receives structured prompt with all 6 profile fields
- System prompt enforces MUST/MUST NOT rules from spec

**Existing Infrastructure Reused:**
- `requireAuth` middleware from auth/src/middleware/auth.ts
- `useAuth()` hook with profile from AuthProvider
- Rate limiting pattern from translate.ts
- Error response helpers from utils/errors.ts

## Outcome

- Impact: Complete implementation plan with 4 ADRs, component design, data flow, and API contract
- Tests: Success criteria mapped to implementation approach
- Files: specs/personalized-feature/plan.md created
- Next prompts: /sp.tasks to generate detailed task breakdown
- Reflection: Leveraged translation-feature as reference pattern for consistency

## Evaluation notes (flywheel)

- Failure modes observed: None - clear spec with existing patterns to follow
- Graders run and results (PASS/FAIL): N/A - planning document
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Generate tasks and begin implementation
