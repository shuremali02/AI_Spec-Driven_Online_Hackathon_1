# Feature Specification: Authentication & User Personalization (Bonus Feature)

**Feature Branch**: `better-auth`
**Created**: 2025-12-15
**Updated**: 2025-12-16
**Status**: Final
**Priority**: Bonus (50 points)
**Version**: 2.1

---

## 1. Feature Overview

### 1.1 Summary

This specification defines the authentication and user personalization bonus feature for the Physical AI & Humanoid Robotics textbook. The feature enables:

1. User signup and signin using Better-Auth (Node.js)
2. Mandatory collection of user background information at signup
3. Storage of user profile data for future personalization use

### 1.2 Original Requirement

> "Participants can receive up to 50 extra bonus points if they also implement Signup and Signin using https://www.better-auth.com/. At signup you will ask questions from the user about their software and hardware background. Knowing the background of the user we will be able to personalize the content."

### 1.3 Bonus Point Eligibility

**Total Available**: 50 bonus points

**Award Conditions** (ALL must be satisfied):
- Signup implemented using Better-Auth
- Signin implemented using Better-Auth
- Background questions are mandatory and non-skippable at signup
- User profile data stored in Neon Serverless PostgreSQL
- Profile data available for future personalization use

**Partial Implementation**: NO bonus points awarded

---

## 2. Clarifications

### Session 2025-12-16

- Q: How should the application reference authenticated users for profile data? → A: Reference Better-Auth user ID via foreign key in user_profiles (application owns only profile data)
- Q: How should the application validate sessions for protected endpoints? → A: Call Better-Auth session validation API; receive user context if valid, 401 if invalid
- Q: Should the specification define Better-Auth-owned tables (users, sessions)? → A: Remove users/sessions tables from spec; document Better-Auth owns auth schema
- Q: What is the expected flow for handling signup atomically with profile creation? → A: Create user via Better-Auth first, then create profile; rollback auth user if profile fails
- Q: Should the application API expose session tokens or let Better-Auth handle session delivery? → A: Better-Auth handles session via cookies; application API returns only user/profile data
- Q: What Better-Auth configuration details are needed? → A: Added Section 8.4 with database adapter, session strategy, cookie settings, CSRF, CORS, and environment variables

---

## 3. Architecture Boundaries

### 3.1 System Separation

| Component | Technology | Responsibility | Location |
|-----------|------------|----------------|----------|
| Existing Chatbot Backend | Python/FastAPI | RAG chatbot functionality | `/backend/` |
| Authentication Backend | Node.js/Better-Auth | Signup, signin, session management | `/auth/` (NEW) |
| Frontend | Docusaurus + React | UI for auth forms, profile display | `/book-write/` |
| Database | Neon Serverless PostgreSQL | User profiles (app-owned), auth data (Better-Auth-owned) | Cloud |

### 3.2 Critical Constraint

**The existing chatbot backend (`/backend/`) MUST NOT be modified.** Authentication is a separate, independent system.

### 3.3 Folder Structure

```
project-root/
├── auth/                              # NEW - Authentication service (Node.js)
│   ├── src/
│   │   ├── index.ts                   # Entry point
│   │   ├── better-auth.ts             # Better-Auth configuration
│   │   ├── routes/
│   │   │   ├── profile.ts             # Profile CRUD endpoints
│   │   │   └── health.ts              # Health check
│   │   └── db/
│   │       ├── client.ts              # Neon PostgreSQL client
│   │       └── schema.ts              # Application-owned schema only
│   ├── package.json
│   └── tsconfig.json
│
├── backend/                           # EXISTING - DO NOT MODIFY
│   └── (Python RAG chatbot)
│
└── book-write/                        # Docusaurus frontend
    └── src/
        └── components/
            ├── Auth/
            │   ├── SignupForm.tsx
            │   ├── SigninForm.tsx
            │   └── AuthProvider.tsx
            └── Profile/
                └── UserProfile.tsx
```

### 3.4 Data Ownership

| Data Type | Owner | Notes |
|-----------|-------|-------|
| User credentials (email, password hash) | Better-Auth | Application MUST NOT access or store |
| Session tokens and cookies | Better-Auth | Application MUST NOT parse or validate manually |
| User profile (background data) | Application | Linked to Better-Auth user via `auth_user_id` |

---

## 4. Authentication Flow

### 4.1 Better-Auth Ownership

Better-Auth has **FULL ownership** of:
- Password hashing and storage
- Credential verification
- Session creation, validation, and expiration
- Cookie management

The application MUST NOT:
- Store or manage raw passwords
- Perform manual session token validation
- Implement any authentication logic outside Better-Auth

### 4.2 Signup Flow

```
1. User fills signup form (email, password, background questions)
2. Frontend validates all fields are complete
3. Frontend calls Better-Auth signup API with email + password
4. Better-Auth creates user, returns user context, sets session cookie
5. Frontend immediately calls application API to create profile with background data
6. Application stores profile linked to auth_user_id
7. If profile creation fails → call Better-Auth to delete user (rollback)
8. Success: User is signed up and signed in with profile stored
```

### 4.3 Signin Flow

```
1. User fills signin form (email, password)
2. Frontend calls Better-Auth signin API
3. Better-Auth validates credentials, returns user context, sets session cookie
4. Frontend calls application API to fetch profile
5. Application validates session via Better-Auth API
6. Application returns profile data
7. Success: User is signed in with profile displayed
```

### 4.4 Session Validation

For any protected endpoint:
```
1. Request arrives with Better-Auth session cookie
2. Application calls Better-Auth session validation API
3. Better-Auth returns: valid (with user context) OR invalid (401)
4. If valid: proceed with user context
5. If invalid: return 401 Unauthorized
```

---

## 5. Signup Requirements

### 5.1 Required Fields

All fields are **MANDATORY**. Signup MUST fail if any field is missing or invalid.

#### Authentication Fields (Better-Auth Owned)

| Field | Type | Validation |
|-------|------|------------|
| email | string | Valid email format, unique |
| password | string | Minimum 8 characters |

#### Software Background Fields (Application Owned)

| Field | Type | Validation | Options |
|-------|------|------------|---------|
| programming_languages | string[] | Minimum 1 selection | Python, JavaScript/TypeScript, C/C++, Java, Go, Rust, Other |
| frameworks_platforms | string[] | Minimum 1 selection | ROS/ROS 2, TensorFlow, PyTorch, OpenCV, Arduino/Embedded, Web Development, Mobile Development, Other |
| experience_level | enum | Required | beginner, intermediate, advanced, expert |

#### Hardware Background Fields (Application Owned)

| Field | Type | Validation | Options |
|-------|------|------------|---------|
| device_type | enum | Required | desktop, laptop, tablet, mobile, embedded |
| operating_system | enum | Required | windows, macos, linux, other |
| system_capability | enum | Required | low, medium, high |

### 5.2 Signup Request Flow

**Step 1: Better-Auth Signup**
```
POST /api/auth/signup (Better-Auth endpoint)
Body: { email, password }
Response: User context + session cookie set
```

**Step 2: Profile Creation**
```
POST /api/profile (Application endpoint)
Cookie: Better-Auth session cookie
Body: { software_background, hardware_background }
Response: { profile_id, created_at }
```

### 5.3 Validation Rules

| Condition | Behavior |
|-----------|----------|
| Missing email or password | Better-Auth returns 400 |
| Duplicate email | Better-Auth returns 400 |
| Missing any background field | Application returns 400 |
| Invalid enum value | Application returns 400 |
| Empty programming_languages array | Application returns 400 |
| Empty frameworks_platforms array | Application returns 400 |

### 5.4 Atomicity and Rollback

- Profile creation is NOT atomic with Better-Auth user creation
- If profile creation fails after successful Better-Auth signup:
  - Application MUST call Better-Auth user deletion API
  - Return error to user indicating signup failed
  - User must retry signup from beginning

---

## 6. User Profile Handling

### 6.1 Profile Storage

The application owns a single table for user profiles:

**Table: user_profiles** (Application-Owned)
```
| Column | Type | Constraints |
|--------|------|-------------|
| id | UUID | PRIMARY KEY, DEFAULT gen_random_uuid() |
| auth_user_id | VARCHAR(255) | UNIQUE, NOT NULL (references Better-Auth user) |
| programming_languages | TEXT[] | NOT NULL |
| frameworks_platforms | TEXT[] | NOT NULL |
| experience_level | VARCHAR(20) | NOT NULL, CHECK IN ('beginner','intermediate','advanced','expert') |
| device_type | VARCHAR(20) | NOT NULL, CHECK IN ('desktop','laptop','tablet','mobile','embedded') |
| operating_system | VARCHAR(20) | NOT NULL, CHECK IN ('windows','macos','linux','other') |
| system_capability | VARCHAR(10) | NOT NULL, CHECK IN ('low','medium','high') |
| created_at | TIMESTAMPTZ | DEFAULT NOW() |
| updated_at | TIMESTAMPTZ | DEFAULT NOW() |
```

**Note:** Better-Auth manages its own tables (users, sessions, etc.). The application MUST NOT define or interact with those tables directly.

### 6.2 Profile Endpoints

#### GET /api/profile

Fetch authenticated user's profile.

```
Request:
  Cookie: Better-Auth session cookie

Response (200):
  {
    "auth_user_id": "ba_user_123",
    "email": "user@example.com",  // from Better-Auth context
    "software_background": {
      "programming_languages": ["Python", "JavaScript"],
      "frameworks_platforms": ["ROS 2", "PyTorch"],
      "experience_level": "intermediate"
    },
    "hardware_background": {
      "device_type": "laptop",
      "operating_system": "linux",
      "system_capability": "high"
    },
    "created_at": "2025-12-15T10:30:00Z",
    "updated_at": "2025-12-15T10:30:00Z"
  }

Response (401): Unauthorized (no valid session)
Response (404): Profile not found
```

#### PUT /api/profile

Update authenticated user's profile.

```
Request:
  Cookie: Better-Auth session cookie
  Body:
    {
      "software_background": { ... },
      "hardware_background": { ... }
    }

Response (200):
  {
    "message": "Profile updated successfully",
    "updated_at": "2025-12-15T14:45:00Z"
  }

Response (400): Validation error
Response (401): Unauthorized
```

### 6.3 Profile Display

After signin, the frontend MUST display the user's profile on a dedicated profile page or section. This includes:
- Email (from Better-Auth context)
- All background fields
- Option to update profile

---

## 7. Personalization Scope

### 7.1 What Personalization Means (Bonus Feature)

For this bonus feature, **personalization is LIMITED to**:

1. **Data Collection**: Gathering user background at signup
2. **Data Storage**: Persisting profile data in PostgreSQL
3. **Data Availability**: Making profile data accessible via API for future use

### 7.2 What Personalization Does NOT Include

The following are **explicitly OUT OF SCOPE** for bonus points:

- AI-driven content adaptation
- Chatbot response modification based on profile
- Dynamic content rendering based on experience level
- Recommendation algorithms
- Any integration with the existing RAG chatbot

### 7.3 Future Use

The stored profile data is intended for future personalization features (not required for bonus):
- Content depth adjustment based on experience level
- Code examples in user's preferred languages
- Hardware-specific setup instructions

---

## 8. API Contract

### 8.1 Authentication Endpoints (Better-Auth Managed)

These endpoints are provided by Better-Auth. The application configures but does not implement them.

| Endpoint | Method | Purpose |
|----------|--------|---------|
| `/api/auth/signup` | POST | Create new user |
| `/api/auth/signin` | POST | Authenticate existing user |
| `/api/auth/signout` | POST | End session |
| `/api/auth/session` | GET | Validate session, get user context |

### 8.2 Application Endpoints

| Endpoint | Method | Auth Required | Purpose |
|----------|--------|---------------|---------|
| `/api/profile` | POST | Yes | Create profile after signup |
| `/api/profile` | GET | Yes | Fetch current user's profile |
| `/api/profile` | PUT | Yes | Update current user's profile |
| `/api/health` | GET | No | Health check |

### 8.3 Error Response Format

All application endpoints return errors in consistent format:

```json
{
  "error": "error_code",
  "field": "field_name",  // optional, for validation errors
  "message": "Human-readable error description"
}
```

### 8.4 Better-Auth Configuration

This section defines the required Better-Auth configuration settings. These are **contractual specifications**, not implementation code.

#### 8.4.1 Database Adapter

| Setting | Value | Notes |
|---------|-------|-------|
| Adapter | PostgreSQL (Drizzle or Prisma) | Better-Auth's PostgreSQL adapter |
| Connection | Neon Serverless PostgreSQL | Same database instance as chatbot |
| Connection String | Environment variable `DATABASE_URL` | Neon connection string with SSL |
| Schema Prefix | `better_auth_` | Isolate Better-Auth tables from application tables |

#### 8.4.2 Session Strategy

| Setting | Value | Notes |
|---------|-------|-------|
| Strategy | Cookie-based | Better-Auth default, recommended for web apps |
| Session Storage | Database | Sessions persisted in Better-Auth managed table |
| Session Duration | 7 days (604800 seconds) | Default expiration for inactive sessions |
| Sliding Expiration | Enabled | Session refreshed on activity |

#### 8.4.3 Cookie Settings

| Setting | Value | Notes |
|---------|-------|-------|
| `httpOnly` | `true` | Prevent JavaScript access (XSS protection) |
| `secure` | `true` (production) | HTTPS only in production |
| `sameSite` | `lax` | CSRF protection while allowing navigation |
| `domain` | Application domain | Set to production domain |
| `path` | `/` | Cookie available site-wide |
| `maxAge` | 604800 (7 days) | Matches session duration |

#### 8.4.4 Security Settings

| Setting | Value | Notes |
|---------|-------|-------|
| CSRF Protection | Enabled | Better-Auth built-in CSRF tokens |
| Password Hashing | Argon2id | Better-Auth default (secure) |
| Rate Limiting | 5 attempts per 15 minutes | Brute force protection for signin |
| Account Lockout | 30 minutes after 5 failed attempts | Temporary lockout |

#### 8.4.5 CORS Configuration

| Setting | Value | Notes |
|---------|-------|-------|
| Allowed Origins | Docusaurus frontend URL | e.g., `https://your-domain.com` |
| Credentials | `true` | Allow cookies in cross-origin requests |
| Allowed Methods | `GET, POST, PUT, DELETE, OPTIONS` | Standard REST methods |
| Allowed Headers | `Content-Type, Authorization` | Required headers |

#### 8.4.6 Environment Variables

The following environment variables MUST be configured:

| Variable | Description | Example |
|----------|-------------|---------|
| `DATABASE_URL` | Neon PostgreSQL connection string | `postgresql://user:pass@host/db?sslmode=require` |
| `BETTER_AUTH_SECRET` | Secret key for signing sessions | 32+ character random string |
| `BETTER_AUTH_URL` | Base URL for auth service | `https://auth.your-domain.com` |
| `FRONTEND_URL` | Docusaurus frontend URL | `https://your-domain.com` |
| `NODE_ENV` | Environment mode | `production` or `development` |

#### 8.4.7 Better-Auth Endpoints Mapping

Better-Auth auto-generates these endpoints based on configuration:

| Better-Auth Path | Exposed As | Purpose |
|------------------|------------|---------|
| `/auth/sign-up/email` | `/api/auth/signup` | Email/password signup |
| `/auth/sign-in/email` | `/api/auth/signin` | Email/password signin |
| `/auth/sign-out` | `/api/auth/signout` | End session |
| `/auth/session` | `/api/auth/session` | Get current session |
| `/auth/user` | Internal | Delete user (for rollback) |

---

## 9. Non-Functional Requirements

### 9.1 Security

- **SEC-001**: All endpoints MUST use HTTPS in production
- **SEC-002**: Passwords are NEVER handled by application code (Better-Auth only)
- **SEC-003**: Session cookies MUST be HTTP-only and secure
- **SEC-004**: All database queries MUST use parameterized queries
- **SEC-005**: User input MUST be sanitized before storage

### 9.2 Performance

- **PERF-001**: Profile creation MUST complete within 3 seconds
- **PERF-002**: Profile fetch MUST complete within 1 second
- **PERF-003**: Session validation MUST complete within 500ms

### 9.3 Reliability

- **REL-001**: Authentication service MUST be independent of chatbot backend
- **REL-002**: Chatbot MUST continue functioning if auth service is down
- **REL-003**: Profile creation failure MUST trigger auth user rollback

### 9.4 Data Privacy

- **PRIV-001**: Profile data used only for personalization purposes
- **PRIV-002**: Users can view and update their profile data
- **PRIV-003**: No profile data shared with third parties

---

## 10. Acceptance Criteria

### 10.1 Signup Tests

**Test SIGNUP-001**: Successful signup with all required fields
- **Given**: User provides valid email, password, and all background fields
- **When**: User completes signup flow
- **Then**: Better-Auth user created, profile stored, user signed in

**Test SIGNUP-002**: Signup fails with missing background field
- **Given**: User provides email, password, but missing programming_languages
- **When**: User attempts to complete signup
- **Then**: 400 error returned, no user created

**Test SIGNUP-003**: Signup fails with duplicate email
- **Given**: Email already registered in Better-Auth
- **When**: User attempts signup
- **Then**: Better-Auth returns 400, signup fails

**Test SIGNUP-004**: Profile failure triggers rollback
- **Given**: Better-Auth signup succeeds, profile creation fails
- **When**: Error occurs during profile storage
- **Then**: Better-Auth user deleted, error returned to user

### 10.2 Signin Tests

**Test SIGNIN-001**: Successful signin
- **Given**: Registered user with valid credentials
- **When**: User submits signin form
- **Then**: Session cookie set, user context available

**Test SIGNIN-002**: Signin fails with wrong password
- **Given**: Registered user provides incorrect password
- **When**: User submits signin form
- **Then**: 401 error returned, no session created

**Test SIGNIN-003**: Profile retrieved after signin
- **Given**: Signed-in user
- **When**: Frontend fetches profile
- **Then**: Profile data returned with background information

### 10.3 Session Tests

**Test SESSION-001**: Valid session allows profile access
- **Given**: User with valid session cookie
- **When**: User requests profile
- **Then**: Profile data returned

**Test SESSION-002**: Invalid session returns 401
- **Given**: Request without session cookie or expired session
- **When**: User requests profile
- **Then**: 401 Unauthorized returned

### 10.4 Profile Tests

**Test PROFILE-001**: Profile update succeeds
- **Given**: Signed-in user
- **When**: User updates experience_level to "advanced"
- **Then**: Profile updated, new updated_at timestamp

**Test PROFILE-002**: Profile update with invalid enum fails
- **Given**: Signed-in user
- **When**: User submits invalid experience_level value
- **Then**: 400 error with field indication

---

## 11. Out of Scope

The following are **explicitly NOT included** in this specification:

### Authentication Features
- Social authentication (OAuth, Google, GitHub)
- Password reset functionality
- Email verification
- Multi-factor authentication
- User roles and permissions

### Personalization Features
- AI-driven content personalization
- Chatbot response modification
- Dynamic content rendering
- Recommendation systems
- A/B testing

### System Modifications
- Any changes to existing chatbot backend (`/backend/`)
- Chatbot integration with user profiles
- Admin panel for user management

---

## 12. Dependencies

### 12.1 External Dependencies

| Dependency | Purpose |
|------------|---------|
| Better-Auth | Authentication provider (Node.js) |
| Neon PostgreSQL | Database storage |
| Node.js 18+ | Authentication backend runtime |

### 12.2 Internal Dependencies

| Dependency | Location | Notes |
|------------|----------|-------|
| Docusaurus Frontend | `/book-write/` | Add auth UI components |
| PostgreSQL Client | Shared with chatbot | Connection to Neon |

---

## 13. Implementation Order

1. Set up `/auth/` Node.js project structure
2. Configure Better-Auth with Neon PostgreSQL
3. Create application-owned `user_profiles` table
4. Implement profile CRUD endpoints
5. Add session validation middleware
6. Create frontend signup form with background questions
7. Create frontend signin form
8. Create frontend profile display component
9. Implement signup flow with rollback logic
10. Integration testing
11. End-to-end testing

---

## 14. Document Control

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0 | 2025-12-15 | Claude Code | Initial specification |
| 2.0 | 2025-12-16 | Claude Code | Corrected Better-Auth ownership, separated auth backend (Node.js), clarified personalization scope, added architecture boundaries |
| 2.1 | 2025-12-16 | Claude Code | Added Section 8.4 Better-Auth Configuration (database adapter, session strategy, cookie settings, security, CORS, environment variables) |

---

**END OF SPECIFICATION**
