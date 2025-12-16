---
id: 0001
title: Final Auth & Personalization Specification
stage: spec
date: 2025-12-15
surface: agent
model: claude-sonnet-4-5-20250929
feature: auth-personalization
branch: better-auth
user: shuremali
command: /sp.spacify
labels: ["authentication", "better-auth", "personalization", "bonus-feature", "specification"]
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

/sp.spacify # SYSTEM PROMPT: FINAL SPECIFICATION WRITER (AUTH + PERSONALIZATION BONUS)

You are a **senior system architect and product specification writer**.

Your task is to write **FINAL, production-ready specifications** strictly based on the requirement below.

This feature is part of a **hackathon evaluation system**, so:
- Clarity is mandatory
- No assumptions are allowed
- No implementation gaps are allowed
- Partial or vague specs are unacceptable

You must write specs that are **directly usable** for:
- Implementation planning
- Task breakdown
- Development execution
- Hackathon evaluation

---

## STRICT RULES (DO NOT VIOLATE)

- ❌ DO NOT rewrite or modify the requirement
- ❌ DO NOT add extra features
- ❌ DO NOT suggest alternatives
- ❌ DO NOT include code
- ❌ DO NOT include UI mockups
- ❌ DO NOT include marketing language
- ❌ DO NOT leave anything implied or unclear

If something is required, it must be **explicitly specified**.

---

## REQUIREMENT (DO NOT CHANGE)

"Participants can receive up to 50 extra bonus points if they also implement Signup and Signin using https://www.better-auth.com/.
At signup you will ask questions from the user about their software and hardware background.
Knowing the background of the user we will be able to personalize the content."

---

## FEATURE NAME

**Authentication & User Personalization (Bonus Feature)**

---

## SPECIFICATION OBJECTIVE

Write a **single, clean, FINAL specification document** that enforces:

### 1. AUTHENTICATION (MANDATORY FIRST STEP)
- Signup and Signin MUST exist
- Authentication MUST use **Better-Auth only**
- No custom or alternative auth system is allowed
- Personalization MUST NOT be accessible without authentication

---

### 2. SIGNUP REQUIREMENTS (NON-SKIPPABLE)
At signup, the system MUST collect **mandatory** background information:

#### Software Background
- Programming languages
- Frameworks or platforms
- Experience level

#### Hardware Background
- Device type
- Operating system
- System capability level

Rules:
- Questions MUST be required
- Signup MUST fail if data is missing
- Data MUST be linked to the authenticated user

---

### 3. SIGNIN REQUIREMENTS
- Existing users MUST sign in using Better-Auth
- On successful signin:
  - User identity MUST be validated
  - Background profile MUST be retrieved

---

### 4. DATABASE REQUIREMENTS
- The system MUST use **Neon Serverless PostgreSQL**
- User and background data MUST be stored securely
- Background data MUST be persistently linked to the authenticated user

---

### 5. BACKEND ARCHITECTURE REQUIREMENTS
- A **new backend folder named `auth/` MUST be created**
- All authentication and authorization logic MUST live inside this folder
- Auth guards MUST protect personalized content

---

### 6. PERSONALIZATION REQUIREMENTS
- Personalization MUST use stored background data
- Personalization applies ONLY to authenticated users
- Personalization may include:
  - Content depth adjustment
  - Relevant examples
  - Hardware-aware recommendations

---

### 7. BONUS POINT RULES (STRICT)
- Up to **50 bonus points**
- Bonus points are awarded ONLY IF:
  - Signup uses Better-Auth
  - Signin uses Better-Auth
  - Background questions are mandatory
  - Data is stored in Neon DB
  - Data is actively used for personalization
- Partial implementation = **NO bonus**

---

### 8. NON-FUNCTIONAL REQUIREMENTS
- Secure authentication
- Data privacy
- Concurrent user handling
- Clear user-friendly error messages

---

## OUTPUT REQUIREMENTS

Produce:
- One **FINAL**, **structured**, **production-ready** specification document
- Suitable for hackathon evaluation
- With clearly labeled sections
- With zero ambiguity

## Response snapshot

Created a comprehensive, production-ready specification document for the Authentication & User Personalization bonus feature. The specification includes:

**17 Sections Covering:**
1. Overview and Bonus Point Eligibility (50 points, all-or-nothing)
2. Architecture Requirements - new `backend/auth/` folder structure
3. Authentication Requirements - Better-Auth signup/signin flows with schemas
4. Database Requirements - Neon PostgreSQL with users, user_profiles, sessions tables
5. Access Control Requirements - Auth guards for protected endpoints
6. Background Data Collection - Software (languages, frameworks, experience) and Hardware (device, OS, capability)
7. Personalization Requirements - Content depth, relevant examples, hardware-aware recommendations
8. Functional Requirements - 30+ explicit requirements across auth, signup, signin, data, personalization, access control
9. Non-Functional Requirements - Security, privacy, performance, reliability, UX
10. API Contract - Complete request/response schemas for all endpoints
11. Success Criteria - Measurable outcomes for bonus qualification
12. Acceptance Tests - Given-When-Then test scenarios
13. Dependencies - External (Better-Auth, Neon, FastAPI) and internal
14. Out of Scope - Explicit exclusions (OAuth, password reset, MFA, etc.)
15. Risks and Mitigations
16. Implementation Order - 9-step implementation sequence
17. Document Control

**Key Compliance Points:**
- Better-Auth ONLY for authentication (no alternatives)
- ALL background fields are REQUIRED and non-skippable
- Signup FAILS if any field is missing
- Personalization ONLY available to authenticated users
- Data actively used for personalization (passive storage = no bonus)

## Outcome

- ✅ Impact: Created complete specification for 50-point bonus feature
- 🧪 Tests: 10 acceptance test scenarios defined (SIGNUP-001 to SIGNUP-004, SIGNIN-001 to SIGNIN-003, ACCESS-001 to ACCESS-002, PERS-001 to PERS-003)
- 📁 Files: 1 file created (specs/auth-personalization/spec.md)
- 🔁 Next prompts: /sp.plan to generate implementation plan, /sp.tasks to generate task breakdown
- 🧠 Reflection: Specification follows hackathon requirements exactly - no extras, no assumptions, clear mandatory fields

## Evaluation notes (flywheel)

- Failure modes observed: None - specification generated successfully on first attempt
- Graders run and results (PASS/FAIL): Manual review required - spec follows all STRICT RULES
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Validate Better-Auth API compatibility with specified schema
