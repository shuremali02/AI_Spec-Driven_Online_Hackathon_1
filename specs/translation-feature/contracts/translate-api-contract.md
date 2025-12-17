# API Contract: Translation Endpoint

**Feature**: translation-feature
**Date**: 2025-12-17
**Status**: Final

---

## 1. Endpoint Overview

| Property | Value |
|----------|-------|
| **Path** | `/api/translate` |
| **Method** | `POST` |
| **Authentication** | Required (Better-Auth session cookie) |
| **Content-Type** | `application/json` |
| **Rate Limit** | 10 requests/minute/user |

---

## 2. Request

### 2.1 Headers

| Header | Required | Value |
|--------|----------|-------|
| `Content-Type` | Yes | `application/json` |
| `Cookie` | Yes | Better-Auth session cookie (automatic) |

### 2.2 Request Body

```json
{
  "chapter_id": "string (required)",
  "content": "string (required)"
}
```

### 2.3 Field Specifications

| Field | Type | Required | Constraints | Description |
|-------|------|----------|-------------|-------------|
| `chapter_id` | string | Yes | Non-empty, max 200 chars | Unique chapter identifier (slug) |
| `content` | string | Yes | Non-empty, max 100,000 chars | Full markdown content to translate |

### 2.4 Example Request

```http
POST /api/translate HTTP/1.1
Host: localhost:3001
Content-Type: application/json
Cookie: better-auth.session_token=xxx

{
  "chapter_id": "module-1/chapter-01-intro-physical-ai",
  "content": "# Introduction to Physical AI\n\nPhysical AI combines artificial intelligence with robotics...\n\n```python\nimport rclpy\n```"
}
```

---

## 3. Responses

### 3.1 Success Response (200 OK)

```json
{
  "success": true,
  "translated_content": "string",
  "chapter_id": "string",
  "word_count": "number",
  "timestamp": "string (ISO 8601)"
}
```

**Example:**
```json
{
  "success": true,
  "translated_content": "# فزیکل AI کا تعارف\n\nفزیکل AI مصنوعی ذہانت کو روبوٹکس کے ساتھ ملاتی ہے...\n\n```python\nimport rclpy\n```",
  "chapter_id": "module-1/chapter-01-intro-physical-ai",
  "word_count": 4200,
  "timestamp": "2025-12-17T10:30:00.000Z"
}
```

### 3.2 Error: Unauthorized (401)

Returned when user is not authenticated or session expired.

```json
{
  "success": false,
  "error": "AUTH_REQUIRED",
  "message": "Authentication required to translate content"
}
```

### 3.3 Error: Bad Request (400)

Returned for validation errors.

**Missing/Invalid chapter_id:**
```json
{
  "success": false,
  "error": "INVALID_REQUEST",
  "message": "Chapter ID is required",
  "field": "chapter_id"
}
```

**Missing/Invalid content:**
```json
{
  "success": false,
  "error": "INVALID_REQUEST",
  "message": "Content is required",
  "field": "content"
}
```

**Content too long:**
```json
{
  "success": false,
  "error": "CONTENT_TOO_LONG",
  "message": "Content exceeds maximum length of 100,000 characters"
}
```

### 3.4 Error: Rate Limited (429)

Returned when user exceeds rate limit.

```json
{
  "success": false,
  "error": "RATE_LIMITED",
  "message": "Too many translation requests. Please try again later.",
  "retry_after": 60
}
```

### 3.5 Error: Server Error (500)

Returned when translation processing fails.

```json
{
  "success": false,
  "error": "TRANSLATION_FAILED",
  "message": "Translation service encountered an error. Please try again.",
  "retry_after": 5
}
```

---

## 4. Response Codes Summary

| Status | Error Code | Description |
|--------|------------|-------------|
| 200 | - | Translation successful |
| 400 | `INVALID_REQUEST` | Validation error |
| 400 | `CONTENT_TOO_LONG` | Content exceeds limit |
| 401 | `AUTH_REQUIRED` | Not authenticated |
| 429 | `RATE_LIMITED` | Too many requests |
| 500 | `TRANSLATION_FAILED` | Processing error |

---

## 5. Translation Processing Rules

### 5.1 Content Preservation (NOT Translated)

| Content Type | Pattern | Example |
|--------------|---------|---------|
| Code blocks | ` ```...``` ` | Python, bash code |
| Inline code | `` `...` `` | `rclpy.init()` |
| Frontmatter | `---\n...\n---` | YAML metadata |
| URLs | `http://`, `https://` | Links |
| Technical terms | See list | ROS 2, URDF, DDS |
| File paths | `/path/to/file` | System paths |

### 5.2 Technical Terms (Preserved in English)

```
ROS 2, ROS2, rclpy, rclcpp, colcon, ament, DDS, QoS, URDF, Xacro, TF2,
Python, C++, Node, Topic, Service, Action, Publisher, Subscriber,
Gazebo, RViz, Isaac Sim, Docker, Ubuntu, Linux, Windows, macOS
```

### 5.3 Content Translated

| Content Type | Treatment |
|--------------|-----------|
| Headings (H1-H6) | Translate text, preserve `#` syntax |
| Paragraphs | Fully translate |
| Lists | Translate text, preserve `-`, `*`, numbers |
| Bold/Italic | Translate text, preserve `**`, `*` |
| Tables | Translate cell content, preserve structure |
| Admonitions | Translate content, preserve `:::note`, `:::tip` syntax |

---

## 6. Security Considerations

### 6.1 Authentication

- Session validated via Better-Auth middleware
- Invalid session returns 401 immediately
- No authentication bypass allowed

### 6.2 Input Sanitization

- Content length validated before processing
- Markdown preserved, no HTML injection
- XSS-safe: no raw HTML in response

### 6.3 Rate Limiting

- 10 requests per minute per authenticated user
- Tracked by user ID from session
- 429 returned with `retry_after` seconds

---

## 7. Integration Example

### 7.1 Frontend (TypeScript)

```typescript
async function translateChapter(chapterId: string, content: string): Promise<string> {
  const response = await fetch('/api/translate', {
    method: 'POST',
    credentials: 'include', // Include session cookie
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({ chapter_id: chapterId, content }),
  });

  if (!response.ok) {
    const error = await response.json();
    throw new Error(error.message || 'Translation failed');
  }

  const data = await response.json();
  return data.translated_content;
}
```

### 7.2 Backend Route (Hono/TypeScript)

```typescript
import { Hono } from 'hono';
import { requireAuth, getAuthContext } from '../middleware/auth.js';

const routes = new Hono();

routes.post('/translate', requireAuth, async (c) => {
  const auth = getAuthContext(c);
  const body = await c.req.json();

  // Validate request
  if (!body.chapter_id) {
    return c.json({
      success: false,
      error: 'INVALID_REQUEST',
      message: 'Chapter ID is required',
      field: 'chapter_id',
    }, 400);
  }

  if (!body.content) {
    return c.json({
      success: false,
      error: 'INVALID_REQUEST',
      message: 'Content is required',
      field: 'content',
    }, 400);
  }

  if (body.content.length > 100000) {
    return c.json({
      success: false,
      error: 'CONTENT_TOO_LONG',
      message: 'Content exceeds maximum length of 100,000 characters',
    }, 400);
  }

  try {
    const translatedContent = await translateToUrdu(body.content);

    return c.json({
      success: true,
      translated_content: translatedContent,
      chapter_id: body.chapter_id,
      word_count: body.content.split(/\s+/).length,
      timestamp: new Date().toISOString(),
    });
  } catch (error) {
    return c.json({
      success: false,
      error: 'TRANSLATION_FAILED',
      message: 'Translation service encountered an error',
      retry_after: 5,
    }, 500);
  }
});

export default routes;
```

---

## 8. Testing Scenarios

### 8.1 Happy Path

| Test | Input | Expected |
|------|-------|----------|
| Valid translation | Valid chapter_id, content | 200, translated_content |
| Code preservation | Content with ` ```python...``` ` | Code unchanged in response |
| Technical terms | Content with "ROS 2" | "ROS 2" unchanged |

### 8.2 Error Cases

| Test | Input | Expected |
|------|-------|----------|
| No auth | No session cookie | 401, AUTH_REQUIRED |
| Missing chapter_id | `{}` | 400, INVALID_REQUEST |
| Missing content | `{chapter_id: "x"}` | 400, INVALID_REQUEST |
| Content too long | 101,000 chars | 400, CONTENT_TOO_LONG |
| Rate limit | 11+ requests/min | 429, RATE_LIMITED |

---

**END OF CONTRACT**
