---
id: 0003
title: Add Better-Auth Configuration Section
stage: spec
date: 2025-12-16
surface: agent
model: claude-sonnet-4-5-20250929
feature: auth-personalization
branch: better-auth
user: shuremali
command: /sp.clarify
labels: ["configuration", "better-auth", "session", "cookies", "security", "specification"]
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

/sp.clarify - Task to complete and clarify authentication feature specification by adding missing configuration sections.

User requested addition of Section 8.4 Better-Auth Configuration Details including:
- Database adapter setup for Neon PostgreSQL
- Session strategy (cookie-based)
- Cookie settings (httpOnly, secure, sameSite, domain)
- Session expiration duration (recommend 7 days)
- CSRF protection settings
- Any other Better-Auth specific configuration needed

Context:
- Hackathon bonus feature worth 50 points
- Existing chatbot backend (Python/FastAPI) MUST NOT be modified
- Auth backend: Node.js using Better-Auth (separate service)
- Frontend: Docusaurus (React-based)
- Database: Neon Serverless PostgreSQL

## Response snapshot

Added new **Section 8.4 Better-Auth Configuration** with 7 subsections:

**8.4.1 Database Adapter**
- PostgreSQL adapter (Drizzle or Prisma)
- Neon Serverless PostgreSQL connection
- Schema prefix `better_auth_` for table isolation

**8.4.2 Session Strategy**
- Cookie-based strategy (Better-Auth default)
- Database-persisted sessions
- 7-day session duration with sliding expiration

**8.4.3 Cookie Settings**
- `httpOnly: true` (XSS protection)
- `secure: true` (HTTPS in production)
- `sameSite: lax` (CSRF protection)
- `maxAge: 604800` (7 days)

**8.4.4 Security Settings**
- CSRF protection enabled
- Argon2id password hashing
- Rate limiting: 5 attempts per 15 minutes
- Account lockout: 30 minutes after 5 failed attempts

**8.4.5 CORS Configuration**
- Allowed origins: Docusaurus frontend URL
- Credentials: true for cookie support

**8.4.6 Environment Variables**
- DATABASE_URL, BETTER_AUTH_SECRET, BETTER_AUTH_URL, FRONTEND_URL, NODE_ENV

**8.4.7 Better-Auth Endpoints Mapping**
- Mapped Better-Auth internal paths to exposed API endpoints

**Specification updated to Version 2.1**

## Outcome

- ✅ Impact: Added comprehensive Better-Auth configuration section for implementation clarity
- 🧪 Tests: No new tests added (configuration spec only)
- 📁 Files: 1 file updated (specs/auth-personalization/spec.md v2.0 → v2.1)
- 🔁 Next prompts: /sp.plan to generate implementation plan
- 🧠 Reflection: Configuration details were missing from spec, now complete for hackathon evaluation

## Evaluation notes (flywheel)

- Failure modes observed: None - direct enhancement request fulfilled
- Graders run and results (PASS/FAIL): Manual review - all requested config items addressed
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Validate Better-Auth configuration values against official documentation
