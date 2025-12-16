# Research: Authentication & User Personalization

**Feature**: auth-personalization
**Date**: 2025-12-16
**Status**: Complete

---

## 1. Better-Auth Integration Research

### 1.1 Decision: Better-Auth for Node.js Authentication

**Decision**: Use Better-Auth as the sole authentication provider in a separate Node.js service.

**Rationale**:
- Better-Auth is specifically designed for Node.js/TypeScript ecosystem
- Provides built-in PostgreSQL adapter compatible with Neon
- Handles all authentication complexity (password hashing, session management, CSRF)
- Hackathon requirement explicitly mandates Better-Auth usage
- Separation from Python chatbot backend maintains system independence

**Alternatives Considered**:
| Alternative | Rejected Because |
|-------------|------------------|
| Auth0/Clerk | Not compliant with hackathon requirement for Better-Auth |
| Custom JWT auth | Reinvents wheel, security risk, violates Better-Auth mandate |
| Python-based auth in `/backend/` | Constitution mandates chatbot backend MUST NOT be modified |
| Firebase Auth | Not compliant with hackathon requirement |

### 1.2 Decision: PostgreSQL Adapter (Drizzle)

**Decision**: Use Better-Auth's Drizzle PostgreSQL adapter for Neon database integration.

**Rationale**:
- Drizzle is lightweight and TypeScript-native
- Better suited for serverless (Neon) than Prisma's connection pooling complexity
- Better-Auth has first-class Drizzle support
- Simpler schema management for hackathon timeline

**Alternatives Considered**:
| Alternative | Rejected Because |
|-------------|------------------|
| Prisma adapter | Higher overhead, connection pooling complexity with Neon serverless |
| Raw SQL | Loses Better-Auth's built-in schema management |

---

## 2. Session Management Research

### 2.1 Decision: Cookie-Based Sessions

**Decision**: Use Better-Auth's default cookie-based session strategy.

**Rationale**:
- HTTP-only cookies prevent XSS token theft
- Better-Auth handles all cookie management automatically
- Works seamlessly with Docusaurus React frontend
- No need to manage tokens in localStorage (security risk)

**Alternatives Considered**:
| Alternative | Rejected Because |
|-------------|------------------|
| JWT in localStorage | XSS vulnerability, requires manual token management |
| JWT in memory | Lost on page refresh, poor UX |
| Bearer tokens | Requires manual header management, more frontend complexity |

### 2.2 Decision: 7-Day Session Duration

**Decision**: Sessions expire after 7 days of inactivity with sliding expiration.

**Rationale**:
- Balances security (not too long) with UX (not too short)
- Sliding expiration extends session on activity
- Standard for educational platforms
- Better-Auth supports this configuration natively

---

## 3. Database Schema Research

### 3.1 Decision: Application Owns Only user_profiles Table

**Decision**: Application defines and manages only the `user_profiles` table. Better-Auth owns all authentication tables.

**Rationale**:
- Clear separation of concerns
- Better-Auth manages its own schema migrations
- Reduces coupling between auth and application data
- Foreign key via `auth_user_id` provides necessary link

**Schema Ownership**:
| Table | Owner | Notes |
|-------|-------|-------|
| `better_auth_user` | Better-Auth | Email, password hash, timestamps |
| `better_auth_session` | Better-Auth | Session tokens, expiration |
| `better_auth_account` | Better-Auth | OAuth providers (unused) |
| `user_profiles` | Application | Background data, linked via auth_user_id |

### 3.2 Decision: VARCHAR for auth_user_id Reference

**Decision**: Use `VARCHAR(255)` for `auth_user_id` foreign key rather than UUID.

**Rationale**:
- Better-Auth user IDs may vary in format
- VARCHAR provides flexibility without schema changes
- UNIQUE constraint ensures referential integrity
- Cascade delete handled at application level (not DB level, since cross-owner)

---

## 4. Signup Flow Research

### 4.1 Decision: Two-Step Signup with Compensating Rollback

**Decision**: Signup is a two-step process: (1) Better-Auth creates user, (2) Application creates profile. Rollback on failure.

**Rationale**:
- Cannot achieve true atomicity across Better-Auth and application DB
- Compensating transaction (delete auth user on profile failure) is industry standard
- Better-Auth provides user deletion API for rollback
- Frontend coordinates the two steps

**Flow**:
```
1. Frontend → Better-Auth: signup(email, password)
2. Better-Auth: creates user, sets cookie
3. Frontend → App API: createProfile(background_data)
4. App API: validates session via Better-Auth
5. App API: inserts user_profiles row
6. IF step 5 fails:
   - Frontend → Better-Auth: deleteUser()
   - Return error to user
```

### 4.2 Decision: Frontend Coordinates Signup

**Decision**: Frontend (Docusaurus/React) coordinates the two-step signup process.

**Rationale**:
- Frontend already has user input (email, password, background)
- Can detect step 1 success before step 2
- Can trigger rollback if step 2 fails
- Keeps backend stateless and simple

---

## 5. Frontend Integration Research

### 5.1 Decision: React Components in Docusaurus

**Decision**: Create React components for auth forms within Docusaurus `/book-write/src/components/`.

**Rationale**:
- Docusaurus is React-based, native component support
- Can use React hooks for state management
- Better-Auth provides React client library
- Profile display integrates with existing Docusaurus layout

**Components**:
| Component | Purpose |
|-----------|---------|
| `AuthProvider.tsx` | Context provider for auth state |
| `SignupForm.tsx` | Multi-step signup with background questions |
| `SigninForm.tsx` | Email/password signin |
| `UserProfile.tsx` | Display and edit profile |

### 5.2 Decision: Better-Auth React Client

**Decision**: Use `@better-auth/react` client library for frontend integration.

**Rationale**:
- Official Better-Auth React integration
- Handles session state automatically
- Provides hooks like `useSession`, `useUser`
- Manages cookie-based auth transparently

---

## 6. CORS and Cross-Origin Research

### 6.1 Decision: Credentials-Enabled CORS

**Decision**: Configure CORS with `credentials: true` and explicit origin allowlist.

**Rationale**:
- Required for cookie-based auth across origins
- Docusaurus frontend may be on different domain than auth service
- Explicit origin prevents security issues with wildcard

**Configuration**:
```
Allowed Origins: [FRONTEND_URL]
Credentials: true
Methods: GET, POST, PUT, DELETE, OPTIONS
Headers: Content-Type, Authorization
```

---

## 7. Security Research

### 7.1 Decision: Argon2id Password Hashing

**Decision**: Use Better-Auth's default Argon2id for password hashing.

**Rationale**:
- Argon2id is current industry standard (winner of Password Hashing Competition)
- Resistant to GPU attacks and side-channel attacks
- Better-Auth handles this automatically
- No configuration needed

### 7.2 Decision: Rate Limiting at Auth Level

**Decision**: Configure Better-Auth's built-in rate limiting (5 attempts per 15 minutes).

**Rationale**:
- Prevents brute force attacks
- Built into Better-Auth, no additional implementation
- Account lockout after repeated failures
- Hackathon scope doesn't require custom rate limiting

---

## 8. Deployment Research

### 8.1 Decision: Separate Auth Service Deployment

**Decision**: Deploy auth service (`/auth/`) separately from chatbot backend (`/backend/`).

**Rationale**:
- Constitution mandates chatbot backend MUST NOT be modified
- Auth service is Node.js, chatbot is Python
- Independent scaling and deployment
- Can be co-located or separated based on hosting

**Deployment Options**:
| Platform | Suitability |
|----------|-------------|
| Vercel | Excellent for Node.js, serverless |
| Railway | Good for persistent Node.js service |
| Render | Good for containerized deployment |
| Same server | Viable with process manager (PM2) |

---

## 9. Environment Variables Research

### 9.1 Decision: Minimal Environment Configuration

**Decision**: Use 5 core environment variables for auth service.

**Variables**:
| Variable | Purpose |
|----------|---------|
| `DATABASE_URL` | Neon PostgreSQL connection |
| `BETTER_AUTH_SECRET` | Session signing key |
| `BETTER_AUTH_URL` | Auth service base URL |
| `FRONTEND_URL` | Docusaurus frontend URL |
| `NODE_ENV` | Environment mode |

**Rationale**:
- Minimal configuration reduces deployment complexity
- All sensitive values externalized
- Compatible with all deployment platforms

---

## 10. Research Summary

All technical decisions have been made. No NEEDS CLARIFICATION items remain.

| Area | Decision | Confidence |
|------|----------|------------|
| Auth Provider | Better-Auth (Node.js) | High |
| DB Adapter | Drizzle | High |
| Session Strategy | Cookie-based, 7 days | High |
| Schema Ownership | App owns only user_profiles | High |
| Signup Flow | Two-step with rollback | High |
| Frontend Integration | React in Docusaurus | High |
| CORS | Credentials-enabled | High |
| Security | Better-Auth defaults | High |
| Deployment | Separate service | High |

**Ready for Phase 1: Design & Contracts**
