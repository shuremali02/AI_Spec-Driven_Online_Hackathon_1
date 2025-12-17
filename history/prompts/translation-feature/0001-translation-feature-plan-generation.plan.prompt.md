---
id: "0001"
title: "Translation Feature Plan Generation"
stage: plan
date: 2025-12-17
surface: agent
model: claude-sonnet-4-5-20250929
feature: translation-feature
branch: translation-feature
user: shuremali
command: /sp.plan
labels: ["translation", "urdu", "bonus-feature", "planning"]
links:
  spec: specs/translation-feature/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/translation-feature/research.md
  - specs/translation-feature/data-model.md
  - specs/translation-feature/contracts/translate-api-contract.md
  - specs/translation-feature/quickstart.md
  - specs/translation-feature/plan.md
tests:
  - null
---

## Prompt

You are a senior system architect and product specification writer.

Your task is to write FINAL, production-ready specifications strictly based on the requirement below.
This feature is part of a hackathon evaluation system, so clarity, completeness, and correctness are critical.

IMPORTANT CONTEXT:
- Authentication (Signup/Signin) is already implemented.
- Only logged-in users can access this feature.
- A translation skill already exists in the system named: **"Translate to Urdu"**.
- This feature must explicitly use the existing "Translate to Urdu" skill.
- Do NOT redesign authentication.
- Do NOT introduce new translation mechanisms.

========================
REQUIREMENT
========================

"Participants can receive up to 50 extra bonus points if the logged user can translate the content in Urdu in the chapters by pressing a button at the start of each chapter."

========================
FEATURE: Chapter Content Translation (Bonus)
========================

[Full specification requirements followed...]

Then /sp.plan was executed to generate implementation planning artifacts.

## Response snapshot

Generated complete implementation plan for Chapter Content Translation (Bonus Feature):

**Phase 0 - Research:**
- Created research.md with decisions on:
  - Auth integration via existing useAuth hook
  - Backend API for skill invocation (not frontend-only)
  - Content extraction via MDX props
  - Noto Nastaliq Urdu font from Google Fonts
  - CSS-only RTL implementation
  - Component-level state management

**Phase 1 - Design:**
- Created data-model.md with:
  - TranslationState interface
  - API request/response models
  - Content processing categories
  - Error types and messages
  - Component props specifications

- Created contracts/translate-api-contract.md with:
  - POST /api/translate endpoint
  - Request/response schemas
  - Error codes (AUTH_REQUIRED, INVALID_REQUEST, etc.)
  - Rate limiting (10 req/min/user)
  - Security considerations

- Created quickstart.md with:
  - Step-by-step implementation guide
  - Code snippets for all components
  - Verification checklist

- Created plan.md with:
  - Technical context
  - Constitution check (all gates PASS)
  - Project structure
  - Architecture decisions
  - Component design
  - Data flow diagrams
  - Risk mitigation

**Key Decisions:**
1. Add translation endpoint to existing /auth/ backend
2. Use component-level React state (no Redux)
3. CSS-only RTL with scoped class
4. Preserve code blocks via regex extraction

## Outcome

- ✅ Impact: High - Complete planning artifacts for 50-point bonus feature
- 🧪 Tests: N/A - Planning phase, no tests yet
- 📁 Files: 5 new files created in specs/translation-feature/
- 🔁 Next prompts: /sp.tasks to generate task breakdown
- 🧠 Reflection: Thorough planning ensures implementation follows existing patterns (auth, Hono routes)

## Evaluation notes (flywheel)

- Failure modes observed: None - all constitution gates passed
- Graders run and results (PASS/FAIL): Constitution check PASS
- Prompt variant (if applicable): Standard /sp.plan workflow
- Next experiment (smallest change to try): Consider theme swizzling for global TranslateButton injection
