# Quickstart: Chapter Content Personalization

**Feature**: personalized-content
**Date**: 2025-12-19

---

## Prerequisites

1. Python 3.11+ installed
2. Node.js 18+ installed
3. Access to:
   - Gemini API key (OPENAI_API_KEY)
   - Neon PostgreSQL database (DATABASE_URL)
   - Better-Auth backend running

---

## Step 1: Backend Setup

### 1.1 Navigate to backend directory

```bash
cd backend
```

### 1.2 Install dependencies (if not already)

```bash
pip install -e .
# or
uv pip install -e .
```

### 1.3 Set environment variables

```bash
export OPENAI_API_KEY=<your-gemini-api-key>
export OPENAI_BASE_URL=https://generativelanguage.googleapis.com/v1beta/openai
export GEMINI_MODEL=gemini-2.5-flash
export DATABASE_URL=<your-neon-postgres-url>
export AUTH_BACKEND_URL=<your-auth-backend-url>
export PERSONALIZE_MAX_TOKENS=8192
export PERSONALIZE_RATE_LIMIT=5
export PERSONALIZE_TIMEOUT=60
```

### 1.4 Start the backend

```bash
uvicorn main:app --reload --port 8000
```

---

## Step 2: Frontend Setup

### 2.1 Navigate to frontend directory

```bash
cd book-write
```

### 2.2 Install dependencies (if not already)

```bash
npm install
```

### 2.3 Set environment variables

Create `.env.local`:
```
NEXT_PUBLIC_API_URL=http://localhost:8000
NEXT_PUBLIC_AUTH_URL=http://localhost:3001
```

### 2.4 Start the frontend

```bash
npm run start
```

---

## Step 3: Test the Flow

### 3.1 Create a test user

1. Navigate to signup page
2. Fill in email and password
3. Complete software background:
   - Experience level: intermediate
   - Programming languages: Python, JavaScript
   - Frameworks: ROS/ROS 2
4. Complete hardware background:
   - Device type: laptop
   - Operating system: linux
   - System capability: medium

### 3.2 Test personalization

1. Sign in with test user
2. Navigate to any chapter (e.g., Module 1 / Chapter 1)
3. Verify "Personalize Content" button is visible
4. Click the button
5. Wait for personalization (up to 60 seconds)
6. Verify content has changed
7. Click "Show Original" to toggle back
8. Verify original content is displayed

### 3.3 Test access control

1. Sign out
2. Navigate to same chapter
3. Verify "Personalize Content" button is NOT visible

---

## Quick API Test

### Test with cURL

```bash
# 1. Sign in and get session cookie (via browser or auth API)

# 2. Test personalization endpoint
curl -X POST 'http://localhost:8000/api/personalize' \
  -H 'Content-Type: application/json' \
  -b 'better-auth.session_token=<your-session-token>' \
  -d '{
    "chapter_id": "test-chapter",
    "chapter_content": "# Test Chapter\n\nThis is a test paragraph about ROS 2."
  }'
```

### Expected Success Response

```json
{
  "success": true,
  "personalized_content": "# Test Chapter\n\n[personalized content]...",
  "chapter_id": "test-chapter",
  "personalization_summary": {
    "experience_level": "intermediate",
    "programming_context": ["Python", "JavaScript"],
    "hardware_context": {
      "system_capability": "medium",
      "operating_system": "linux"
    },
    "adjustments_made": [
      "Adapted explanations for intermediate level",
      "Added Python-specific context"
    ]
  },
  "timestamp": "2025-12-19T12:00:00Z"
}
```

---

## Troubleshooting

### Button not visible

1. Check if user is signed in (auth state)
2. Check if user has profile (database)
3. Check browser console for errors

### 401 AUTH_REQUIRED

1. Verify session cookie is set
2. Check if session has expired
3. Try signing out and back in

### 429 RATE_LIMITED

1. Wait for `retry_after` seconds
2. Rate limit is 5 requests per minute per user

### 500 PERSONALIZATION_FAILED

1. Check backend logs for LLM errors
2. Verify OPENAI_API_KEY is set correctly
3. Check Gemini API quota

---

## File Locations

| Component | Location |
|-----------|----------|
| PersonalizeContentAgent | `backend/agents/personalize_agent.py` |
| FastAPI endpoint | `backend/api/personalize.py` |
| Frontend button | `book-write/src/components/Personalization/PersonalizeButton.tsx` |
| Frontend content | `book-write/src/components/Personalization/PersonalizedContent.tsx` |

---

**END OF QUICKSTART**
