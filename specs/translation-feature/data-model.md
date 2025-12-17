# Data Model: Chapter Content Translation Feature

**Feature**: translation-feature
**Date**: 2025-12-17
**Status**: Complete

---

## 1. Overview

This feature does NOT require persistent database storage. All state is managed in-memory:
- Frontend: React component state
- Backend: Stateless request/response

---

## 2. Frontend State Model

### 2.1 TranslationState (Component-Level)

```typescript
/**
 * State managed by TranslateButton component
 * Scope: Per chapter, per session (not persisted)
 */
interface TranslationState {
  /** Current translation operation status */
  status: 'idle' | 'loading' | 'success' | 'error';

  /** Translated Urdu markdown content (cached after successful translation) */
  translatedContent: string | null;

  /** Whether to display translated content (true) or original (false) */
  showTranslated: boolean;

  /** Error message if translation failed */
  errorMessage: string | null;
}
```

### 2.2 State Transitions

| Current State | Event | Next State | Action |
|---------------|-------|------------|--------|
| `idle` | Click translate | `loading` | Call API |
| `loading` | API success | `success` | Store content, show translated |
| `loading` | API error | `error` | Store error message |
| `success` | Click toggle | `idle` (showTranslated=false) | Show original |
| `idle` (cached) | Click translate | `success` | Show cached content |
| `error` | Click retry | `loading` | Call API again |

### 2.3 Initial State

```typescript
const initialState: TranslationState = {
  status: 'idle',
  translatedContent: null,
  showTranslated: false,
  errorMessage: null,
};
```

---

## 3. API Request/Response Models

### 3.1 Translation Request

```typescript
/**
 * POST /api/translate
 * Request body
 */
interface TranslateRequest {
  /** Unique identifier for the chapter (e.g., "module-1/chapter-01-intro-physical-ai") */
  chapter_id: string;

  /** Full markdown content of the chapter to translate */
  content: string;
}
```

**Validation Rules**:
| Field | Rule | Error Code |
|-------|------|------------|
| chapter_id | Required, non-empty string | `INVALID_CHAPTER_ID` |
| content | Required, non-empty string | `INVALID_CONTENT` |
| content | Max 100,000 characters | `CONTENT_TOO_LONG` |

### 3.2 Translation Response (Success)

```typescript
/**
 * 200 OK response
 */
interface TranslateSuccessResponse {
  success: true;

  /** Translated Urdu markdown content */
  translated_content: string;

  /** Echo back the chapter ID */
  chapter_id: string;

  /** Word count of original content */
  word_count: number;

  /** Timestamp of translation */
  timestamp: string; // ISO 8601 format
}
```

### 3.3 Translation Response (Error)

```typescript
/**
 * 4xx/5xx error response
 */
interface TranslateErrorResponse {
  success: false;

  /** Machine-readable error code */
  error: 'AUTH_REQUIRED' | 'INVALID_REQUEST' | 'TRANSLATION_FAILED' | 'RATE_LIMITED';

  /** Human-readable error message */
  message: string;

  /** Seconds to wait before retry (optional) */
  retry_after?: number;
}
```

---

## 4. Content Processing Model

### 4.1 Markdown Content Categories

```typescript
/**
 * Categories of content during translation processing
 */
enum ContentCategory {
  /** Translatable: headings, paragraphs, lists */
  TRANSLATABLE = 'translatable',

  /** Preserved: code blocks, inline code, URLs */
  PRESERVED = 'preserved',

  /** Metadata: frontmatter (YAML between ---) */
  METADATA = 'metadata',
}
```

### 4.2 Content Tokens (Parsing Model)

```typescript
/**
 * Token produced during markdown parsing
 */
interface ContentToken {
  type: 'text' | 'code_block' | 'inline_code' | 'frontmatter' | 'url' | 'technical_term';
  content: string;
  translate: boolean;
  originalIndex: number; // Position in original document
}
```

### 4.3 Technical Terms (Preserved)

```typescript
/**
 * Terms that should NOT be translated
 */
const TECHNICAL_TERMS: string[] = [
  // ROS 2 specific
  'ROS 2', 'ROS2', 'rclpy', 'rclcpp', 'colcon', 'ament',
  'DDS', 'QoS', 'URDF', 'Xacro', 'TF2', 'tf2',

  // Programming
  'Python', 'C++', 'Node', 'Topic', 'Service', 'Action',
  'Publisher', 'Subscriber', 'Client', 'Server',

  // Tools
  'Gazebo', 'RViz', 'Isaac Sim', 'Docker',

  // File types
  '.py', '.cpp', '.urdf', '.xacro', '.yaml', '.launch',
];
```

---

## 5. Props Model (React Components)

### 5.1 TranslateButton Props

```typescript
interface TranslateButtonProps {
  /** Unique chapter identifier for API call */
  chapterId: string;

  /** Original markdown content to translate */
  chapterContent: string;

  /** Whether user is authenticated (from useAuth) */
  isAuthenticated: boolean;

  /** Optional: Callback when translation completes */
  onTranslationComplete?: (translatedContent: string) => void;

  /** Optional: Custom CSS class */
  className?: string;
}
```

### 5.2 UrduContent Props

```typescript
interface UrduContentProps {
  /** Translated Urdu markdown content */
  content: string;

  /** Optional: Custom CSS class */
  className?: string;
}
```

### 5.3 TranslationWrapper Props (Container)

```typescript
interface TranslationWrapperProps {
  /** Chapter ID from frontmatter or slug */
  chapterId: string;

  /** Original English content */
  originalContent: React.ReactNode;

  /** Raw markdown source for translation */
  rawMarkdown: string;

  /** React children (original rendered content) */
  children: React.ReactNode;
}
```

---

## 6. Error Model

### 6.1 Error Types

```typescript
/**
 * Standardized error types for translation feature
 */
enum TranslationErrorType {
  /** User not authenticated */
  AUTH_REQUIRED = 'AUTH_REQUIRED',

  /** Invalid request body */
  INVALID_REQUEST = 'INVALID_REQUEST',

  /** Invalid chapter ID */
  INVALID_CHAPTER_ID = 'INVALID_CHAPTER_ID',

  /** Content too long */
  CONTENT_TOO_LONG = 'CONTENT_TOO_LONG',

  /** Translation process failed */
  TRANSLATION_FAILED = 'TRANSLATION_FAILED',

  /** Rate limit exceeded */
  RATE_LIMITED = 'RATE_LIMITED',

  /** Network/connection error (frontend only) */
  NETWORK_ERROR = 'NETWORK_ERROR',

  /** Request timeout */
  TIMEOUT = 'TIMEOUT',
}
```

### 6.2 Error Messages (User-Facing)

```typescript
const ERROR_MESSAGES: Record<TranslationErrorType, string> = {
  AUTH_REQUIRED: 'Please sign in to translate content.',
  INVALID_REQUEST: 'Translation request was invalid. Please try again.',
  INVALID_CHAPTER_ID: 'Chapter not found.',
  CONTENT_TOO_LONG: 'Chapter is too long to translate at once.',
  TRANSLATION_FAILED: 'Translation failed. Please try again.',
  RATE_LIMITED: 'Too many translation requests. Please wait and try again.',
  NETWORK_ERROR: 'Connection error. Check your internet connection.',
  TIMEOUT: 'Translation timed out. Please try again.',
};
```

---

## 7. Configuration Constants

### 7.1 API Configuration

```typescript
const TRANSLATION_CONFIG = {
  /** API endpoint for translation */
  ENDPOINT: '/api/translate',

  /** Request timeout in milliseconds */
  TIMEOUT_MS: 30000,

  /** Max content length (characters) */
  MAX_CONTENT_LENGTH: 100000,

  /** Rate limit: requests per minute per user */
  RATE_LIMIT_PER_MINUTE: 10,
};
```

### 7.2 UI Configuration

```typescript
const UI_CONFIG = {
  /** Button labels */
  BUTTON_LABELS: {
    TRANSLATE: 'Translate to Urdu',
    TRANSLATE_URDU: 'اردو میں ترجمہ کریں',
    LOADING: 'Translating...',
    SHOW_ORIGINAL: 'Show Original',
    SHOW_ORIGINAL_URDU: 'اصل مواد دکھائیں',
    RETRY: 'Translation Failed - Retry',
  },

  /** Badge label for translated content */
  TRANSLATED_BADGE: 'اردو ترجمہ',
};
```

---

## 8. Database Schema

**NOT APPLICABLE**

This feature does not persist any data to the database. All state is:
- Frontend: React component state (cleared on navigation)
- Backend: Stateless (no storage)

Future enhancement (out of scope):
- Translation cache table
- Translation analytics table

---

## 9. Relationships

### 9.1 Component Relationships

```
AuthProvider (Context)
     │
     └── provides: user, isAuthenticated
            │
            ▼
TranslationWrapper (HOC/Wrapper)
     │
     ├── reads: isAuthenticated from useAuth()
     │
     └── renders conditionally:
            │
            ├── TranslateButton (if authenticated)
            │      │
            │      └── manages: TranslationState
            │
            └── UrduContent (if translated)
                   │
                   └── receives: translatedContent from parent
```

### 9.2 Data Flow

```
User Click
    │
    ▼
TranslateButton
    │
    ├── State: status = 'loading'
    │
    ▼
POST /api/translate
    │
    ├── Request: { chapter_id, content }
    │
    ▼
Backend Processing
    │
    ├── Auth validation
    ├── Content parsing
    ├── Translation (skill logic)
    │
    ▼
Response
    │
    ├── Success: { translated_content }
    │
    ▼
TranslateButton
    │
    ├── State: status = 'success', translatedContent = response
    │
    ▼
UrduContent
    │
    └── Renders: RTL Urdu markdown
```

---

**END OF DATA MODEL**
