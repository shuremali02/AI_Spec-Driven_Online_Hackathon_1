---
id: 0002
title: Clarify Better-Auth Ownership and Architecture
stage: spec
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: auth-personalization
branch: better-auth
user: shuremali
command: /sp.clarify
labels: ["clarification", "better-auth", "architecture", "node.js", "specification"]
links:
  spec: specs/auth-personalization/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/auth-personalization/spec.md
tests:
  - null
---

## Prompt

/sp.clarify - Senior system architect and specification editor task to READ, CORRECT, and CLARIFY the EXISTING authentication and personalization specification.

Key context provided:
- Chatbot backend exists in `/backend` (Python) - MUST remain unchanged
- Authentication MUST be separated from chatbot backend
- Authentication backend should use Better Auth (Node.js ecosystem)
- Frontend is Docusaurus
- Database: Neon Serverless PostgreSQL
- No AI personalization logic required - just data collection, storage, availability

Correction objectives:
1. Better-Auth has FULL ownership of password hashing, credential verification, session management
2. Application MUST NOT store passwords or validate sessions manually
3. Database ownership must be clarified (Better-Auth vs Application)
4. Session handling via Better-Auth APIs only
5. API schemas as contractual specifications, not code

## Response snapshot

**5 Clarification Questions Asked and Answered:**

1. Q: How should the application reference authenticated users for profile data?
   A: Reference Better-Auth user ID via foreign key in user_profiles (application owns only profile data)

2. Q: How should the application validate sessions for protected endpoints?
   A: Call Better-Auth session validation API; receive user context if valid, 401 if invalid

3. Q: Should the specification define Better-Auth-owned tables (users, sessions)?
   A: Remove users/sessions tables from spec; document Better-Auth owns auth schema

4. Q: What is the expected flow for handling signup atomically with profile creation?
   A: Create user via Better-Auth first, then create profile; rollback auth user if profile fails

5. Q: Should the application API expose session tokens or let Better-Auth handle session delivery?
   A: Better-Auth handles session via cookies; application API returns only user/profile data

**Major Corrections Applied:**

1. **Architecture**: Changed auth backend from Python/FastAPI to Node.js/Better-Auth in separate `/auth/` folder
2. **Database Schema**: Removed `users` and `sessions` tables (Better-Auth owned); kept only `user_profiles` (app-owned)
3. **Session Handling**: Removed manual token validation; now uses Better-Auth API calls
4. **Personalization Scope**: Clarified as data collection/storage only, NOT AI-driven content adaptation
5. **System Separation**: Explicitly stated chatbot backend MUST NOT be modified

**Specification Updated to Version 2.0**

## Outcome

- ✅ Impact: Corrected critical architecture violations, clarified Better-Auth ownership boundaries
- 🧪 Tests: 12 acceptance test scenarios preserved with corrected flow expectations
- 📁 Files: 1 file updated (specs/auth-personalization/spec.md v1.0 → v2.0)
- 🔁 Next prompts: /sp.plan to generate implementation plan
- 🧠 Reflection: Original spec incorrectly assumed Python backend for auth and manual session validation. New context from user clarified Node.js ecosystem requirement and separation from chatbot.

## Evaluation notes (flywheel)

- Failure modes observed: Original spec v1.0 violated Better-Auth ownership by defining users/sessions tables and manual token validation
- Graders run and results (PASS/FAIL): Manual review - all 5 correction objectives addressed
- Prompt variant (if applicable): Combined two /sp.clarify runs into single PHR
- Next experiment (smallest change to try): Validate Better-Auth Node.js SDK compatibility with specified architecture
