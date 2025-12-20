# Data Model: Chapter Content Personalization

**Feature**: personalized-content
**Date**: 2025-12-19

---

## Entities

### 1. UserProfile (Existing)

**Table**: `user_profiles`
**Status**: Already implemented
**Location**: Neon PostgreSQL

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| id | UUID | Yes | Primary key |
| auth_user_id | VARCHAR(255) | Yes | Foreign key to Better-Auth user |
| experience_level | VARCHAR(20) | Yes | beginner, intermediate, advanced, expert |
| programming_languages | TEXT[] | Yes | Array of languages |
| frameworks_platforms | TEXT[] | Yes | Array of frameworks |
| device_type | VARCHAR(20) | Yes | desktop, laptop, tablet, mobile, embedded |
| operating_system | VARCHAR(20) | Yes | windows, macos, linux, other |
| system_capability | VARCHAR(10) | Yes | low, medium, high |
| created_at | TIMESTAMPTZ | Yes | Creation timestamp |
| updated_at | TIMESTAMPTZ | Yes | Last update timestamp |

**Validation Rules**:
- `auth_user_id` must be unique
- `experience_level` must be one of: beginner, intermediate, advanced, expert
- `programming_languages` must have at least one element
- `frameworks_platforms` must have at least one element
- `device_type` must be one of: desktop, laptop, tablet, mobile, embedded
- `operating_system` must be one of: windows, macos, linux, other
- `system_capability` must be one of: low, medium, high

---

### 2. PersonalizationRequest (Runtime)

**Type**: Request body (Pydantic model)
**Status**: NEW
**Persisted**: No

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| chapter_id | str | Yes | Unique identifier for chapter |
| chapter_content | str | Yes | Full markdown content to personalize |

**Validation Rules**:
- `chapter_id` must be non-empty string
- `chapter_content` must be non-empty string
- `chapter_content` should be < 50,000 characters

---

### 3. PersonalizationResponse (Runtime)

**Type**: Response body (Pydantic model)
**Status**: NEW
**Persisted**: No

| Field | Type | Required | Description |
|-------|------|----------|-------------|
| success | bool | Yes | Whether personalization succeeded |
| personalized_content | str | Conditional | Personalized markdown (on success) |
| chapter_id | str | Yes | Echo of input chapter_id |
| personalization_summary | object | Conditional | Summary of adjustments (on success) |
| timestamp | str | Yes | ISO8601 timestamp |
| error | str | Conditional | Error code (on failure) |
| message | str | Conditional | Error message (on failure) |
| missing_fields | list[str] | Conditional | Missing profile fields (on PROFILE_INCOMPLETE) |
| retry_after | int | Conditional | Seconds to wait (on RATE_LIMITED/PERSONALIZATION_FAILED) |

**PersonalizationSummary Structure**:
```
{
  "experience_level": str,
  "programming_context": list[str],
  "hardware_context": {
    "system_capability": str,
    "operating_system": str
  },
  "adjustments_made": list[str]
}
```

---

### 4. RateLimitEntry (Runtime)

**Type**: In-memory data structure
**Status**: NEW
**Persisted**: No

| Field | Type | Description |
|-------|------|-------------|
| count | int | Number of requests in current window |
| reset_time | float | Unix timestamp when window resets |

---

## State Transitions

### Personalization Button State Machine

```
                ┌───────────────────┐
                │      IDLE         │
                │ (Show button)     │
                └─────────┬─────────┘
                          │
                  [Click button]
                          │
                          ▼
                ┌───────────────────┐
                │     LOADING       │
                │ (Personalizing...)│
                └─────────┬─────────┘
                          │
           ┌──────────────┼──────────────┐
           │              │              │
     [Success]      [Error]        [Rate Limited]
           │              │              │
           ▼              ▼              ▼
    ┌──────────┐   ┌──────────┐   ┌──────────┐
    │ SUCCESS  │   │  ERROR   │   │  RATE    │
    │(Show Orig│   │ (Retry)  │   │ LIMITED  │
    └────┬─────┘   └────┬─────┘   └────┬─────┘
         │              │              │
   [Toggle]       [Retry]        [Wait]
         │              │              │
         ▼              │              │
    ┌──────────┐        │              │
    │ ORIGINAL │        │              │
    │(Show Pers│◄───────┴──────────────┘
    └────┬─────┘
         │
   [Toggle back]
         │
         ▼
    ┌──────────┐
    │ SUCCESS  │ (cached)
    └──────────┘
```

---

## Relationships

```
Better-Auth User (1) ──── (0..1) UserProfile
                                    │
                                    │ (used for)
                                    ▼
                            PersonalizationRequest
                                    │
                                    │ (produces)
                                    ▼
                            PersonalizationResponse
```

---

## No New Database Tables

This feature does NOT require new database tables. All personalization is:
- Runtime-only (not persisted)
- Based on existing `user_profiles` table
- Cached in frontend component state

---

**END OF DATA MODEL**
