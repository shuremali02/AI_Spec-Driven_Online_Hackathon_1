# API Contract: Personalization Endpoint

**Feature**: personalized-content
**Date**: 2025-12-19
**Version**: 1.0

---

## Endpoint

```
POST /api/personalize
```

---

## Authentication

**Required**: Yes
**Method**: Cookie-based (Better-Auth session)
**Cookie Name**: `better-auth.session_token`

---

## Request

### Headers

| Header | Value | Required |
|--------|-------|----------|
| Content-Type | application/json | Yes |
| Cookie | better-auth.session_token=<token> | Yes |

### Body

```json
{
  "chapter_id": "module-1/chapter-01-intro-physical-ai",
  "chapter_content": "# Chapter 1: Introduction to Physical AI\n\n..."
}
```

### Fields

| Field | Type | Required | Constraints |
|-------|------|----------|-------------|
| chapter_id | string | Yes | Non-empty, valid chapter path |
| chapter_content | string | Yes | Non-empty, < 50,000 characters |

---

## Responses

### 200 OK (Success)

```json
{
  "success": true,
  "personalized_content": "# Chapter 1: Introduction to Physical AI\n\n[personalized content...]",
  "chapter_id": "module-1/chapter-01-intro-physical-ai",
  "personalization_summary": {
    "experience_level": "intermediate",
    "programming_context": ["Python", "JavaScript"],
    "hardware_context": {
      "system_capability": "medium",
      "operating_system": "linux"
    },
    "adjustments_made": [
      "Adapted explanations for intermediate level",
      "Added Python-specific context",
      "Included Linux-specific notes"
    ]
  },
  "timestamp": "2025-12-19T10:30:00Z"
}
```

### 401 Unauthorized (AUTH_REQUIRED)

**Condition**: No session cookie OR invalid/expired session

```json
{
  "success": false,
  "error": "AUTH_REQUIRED",
  "message": "Authentication required to personalize content"
}
```

### 404 Not Found (PROFILE_NOT_FOUND)

**Condition**: User has no profile in `user_profiles` table

```json
{
  "success": false,
  "error": "PROFILE_NOT_FOUND",
  "message": "Please complete your profile to personalize content"
}
```

### 400 Bad Request (PROFILE_INCOMPLETE)

**Condition**: User profile is missing required fields

```json
{
  "success": false,
  "error": "PROFILE_INCOMPLETE",
  "message": "Your profile is missing required fields",
  "missing_fields": ["experience_level", "device_type"]
}
```

### 400 Bad Request (INVALID_REQUEST)

**Condition**: Missing or invalid request fields

```json
{
  "success": false,
  "error": "INVALID_REQUEST",
  "message": "Chapter ID is required",
  "field": "chapter_id"
}
```

### 429 Too Many Requests (RATE_LIMITED)

**Condition**: User exceeded 5 requests per minute

```json
{
  "success": false,
  "error": "RATE_LIMITED",
  "message": "Too many requests. Please try again later.",
  "retry_after": 45
}
```

**Headers**:
```
Retry-After: 45
```

### 500 Internal Server Error (PERSONALIZATION_FAILED)

**Condition**: LLM call failed or timed out

```json
{
  "success": false,
  "error": "PERSONALIZATION_FAILED",
  "message": "Personalization service encountered an error",
  "retry_after": 5
}
```

---

## Error Codes

| Code | HTTP Status | Meaning |
|------|-------------|---------|
| AUTH_REQUIRED | 401 | Session missing or invalid |
| PROFILE_NOT_FOUND | 404 | No user profile exists |
| PROFILE_INCOMPLETE | 400 | Profile missing required fields |
| INVALID_REQUEST | 400 | Request body validation failed |
| RATE_LIMITED | 429 | Rate limit exceeded |
| PERSONALIZATION_FAILED | 500 | Server-side error |

---

## Rate Limiting

| Parameter | Value |
|-----------|-------|
| Limit | 5 requests |
| Window | 60 seconds |
| Scope | Per authenticated user |
| Algorithm | Sliding window |

---

## Timeouts

| Operation | Timeout |
|-----------|---------|
| Session validation | 5 seconds |
| Profile query | 5 seconds |
| LLM personalization | 60 seconds |
| Total request | 70 seconds |

---

## CORS

| Origin | Allowed |
|--------|---------|
| Frontend domain | Yes |
| localhost:3000 | Yes (development) |
| Other | No |

---

## Example cURL

```bash
curl -X POST 'https://api.example.com/api/personalize' \
  -H 'Content-Type: application/json' \
  -b 'better-auth.session_token=<session_token>' \
  -d '{
    "chapter_id": "module-1/chapter-01-intro-physical-ai",
    "chapter_content": "# Chapter 1: Introduction to Physical AI\n\nThis chapter covers..."
  }'
```

---

## Frontend Integration

### TypeScript Types

```typescript
interface PersonalizeRequest {
  chapter_id: string;
  chapter_content: string;
}

interface PersonalizationSummary {
  experience_level: string;
  programming_context: string[];
  hardware_context: {
    system_capability: string;
    operating_system: string;
  };
  adjustments_made: string[];
}

interface PersonalizeSuccessResponse {
  success: true;
  personalized_content: string;
  chapter_id: string;
  personalization_summary: PersonalizationSummary;
  timestamp: string;
}

interface PersonalizeErrorResponse {
  success: false;
  error: string;
  message: string;
  missing_fields?: string[];
  retry_after?: number;
  field?: string;
}

type PersonalizeResponse = PersonalizeSuccessResponse | PersonalizeErrorResponse;
```

### Fetch Example

```typescript
async function personalizeContent(
  chapterId: string,
  chapterContent: string
): Promise<PersonalizeResponse> {
  const response = await fetch('/api/personalize', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    credentials: 'include', // Include cookies
    body: JSON.stringify({
      chapter_id: chapterId,
      chapter_content: chapterContent,
    }),
  });

  return response.json();
}
```

---

**END OF API CONTRACT**
