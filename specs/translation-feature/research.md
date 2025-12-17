# Research: Chapter Content Translation Feature

**Feature**: translation-feature
**Date**: 2025-12-17
**Status**: Complete

---

## 1. Technical Context Clarifications

### 1.1 Authentication Integration

**Decision**: Reuse existing Better-Auth integration via `useAuth` hook

**Rationale**:
- Authentication system already implemented in `/auth/` service
- AuthProvider context exists at `/book-write/src/components/Auth/AuthProvider.tsx`
- `useAuth()` hook provides `user`, `profile`, `isLoading` state
- Session validation middleware exists at `/auth/src/middleware/auth.ts`

**Alternatives Considered**:
- Direct session cookie parsing: Rejected - violates Better-Auth ownership principle
- Separate auth check: Rejected - duplicates existing infrastructure

**Implementation Pattern**:
```typescript
import { useAuth } from '@site/src/components/Auth/AuthProvider';

const { user, isLoading } = useAuth();
const isAuthenticated = !!user && !isLoading;
```

---

### 1.2 Translate-to-Urdu Skill Integration

**Decision**: Invoke skill via backend API endpoint, not directly in frontend

**Rationale**:
- Claude Code skills are designed for agent-side invocation
- Skills require Claude Code runtime context
- Security: Translation should be server-validated (auth check)
- The skill at `.claude/skills/translate-to-urdu/SKILL.md` defines behavior

**Skill Behavior** (from SKILL.md):
- Preserves markdown headings, lists, bold/italic
- Preserves code blocks untouched
- Preserves technical terms
- Returns only translated text in clean Urdu

**Implementation Approach**:
The backend endpoint (`/api/translate`) will:
1. Receive chapter content from frontend
2. Validate user session
3. Process content using skill logic (markdown parsing + translation rules)
4. Return translated content

**Alternatives Considered**:
- Frontend-only translation: Rejected - skills are agent-side, need auth validation
- External translation API: Rejected - violates spec constraint (must use existing skill)

---

### 1.3 Chapter Content Access

**Decision**: Extract chapter content from Docusaurus MDX at runtime

**Rationale**:
- Chapters are stored as `.md` files in `/book-write/docs/module-1/`
- Docusaurus renders markdown to HTML on build
- Need access to original markdown for translation

**Pattern for Content Extraction**:
- Option A: Read raw markdown from `import` using Docusaurus raw-loader
- Option B: Use frontmatter metadata to identify chapter, fetch content via API
- Option C: Pass content from page component to translation component

**Decision**: Option C - Pass content from MDXLayout

**Implementation**:
```typescript
// Custom MDX component wrapper that extracts content
// TranslateButton receives chapterContent as prop
```

**Alternatives Considered**:
- API to read files: Rejected - requires backend file access, complexity
- Frontmatter API: Rejected - Docusaurus doesn't expose raw markdown by default

---

### 1.4 Urdu Font Rendering

**Decision**: Use Noto Nastaliq Urdu from Google Fonts with system fallbacks

**Rationale**:
- Noto Nastaliq Urdu is free, widely supported, open-source
- Google Fonts CDN provides fast loading
- Nastaliq script is the standard for Urdu typography

**Font Stack**:
```css
font-family: "Noto Nastaliq Urdu", "Jameel Noori Nastaleeq", serif;
```

**Implementation**:
- Add Google Font link to Docusaurus head config
- Apply font via CSS class `.urdu-content`

**Alternatives Considered**:
- Self-hosted fonts: Rejected - adds build complexity, larger bundle
- System fonts only: Rejected - inconsistent Urdu rendering across platforms

---

### 1.5 RTL Layout Implementation

**Decision**: CSS-only RTL with scoped class `.urdu-content`

**Rationale**:
- CSS `direction: rtl` handles text flow
- `text-align: right` aligns Urdu text
- Code blocks need `direction: ltr` override
- Docusaurus supports custom CSS

**Implementation Pattern**:
```css
.urdu-content {
  direction: rtl;
  text-align: right;
}

.urdu-content pre,
.urdu-content code {
  direction: ltr;
  text-align: left;
}
```

**Alternatives Considered**:
- HTML `dir="rtl"` attribute: Rejected - harder to scope, affects child elements
- Third-party RTL library: Rejected - overkill for this use case

---

### 1.6 State Management for Translation

**Decision**: React useState within component (no global state)

**Rationale**:
- Translation state is per-chapter, per-session
- No need for global state (Redux/Context for translation)
- Simple toggle: original ↔ translated
- Cache translated content in component state

**State Shape**:
```typescript
interface TranslationState {
  status: 'idle' | 'loading' | 'success' | 'error';
  translatedContent: string | null;
  showTranslated: boolean;
  error: string | null;
}
```

**Alternatives Considered**:
- Global Redux store: Rejected - overkill for chapter-scoped state
- localStorage cache: Rejected - out of scope per spec (Section 14)
- React Query: Rejected - adds dependency, useState sufficient

---

## 2. Best Practices Research

### 2.1 Docusaurus Custom Components

**Best Practices**:
- Place custom components in `/book-write/src/components/`
- Use TypeScript for type safety
- Import in MDX files or wrap in theme layout
- Use `@site` alias for imports

**Pattern for Chapter Enhancement**:
```typescript
// src/theme/DocItem/Layout.tsx - swizzle Docusaurus theme
// Wrap children with TranslateButton
```

**Decision**: Create new components, integrate via MDX or theme swizzling

---

### 2.2 Hono Backend Routes

**Best Practices** (from existing codebase):
- Group routes in `/auth/src/routes/` directory
- Use `requireAuth` middleware for protected endpoints
- Return consistent JSON response format
- Handle errors with utils/errors.ts helpers

**Pattern** (from profile.ts):
```typescript
import { Hono } from 'hono';
import { requireAuth, getAuthContext } from '../middleware/auth.js';

const routes = new Hono();
routes.use('*', requireAuth);
routes.post('/translate', async (c) => { ... });

export default routes;
```

---

### 2.3 Markdown Processing for Translation

**Best Practices**:
- Parse markdown to AST for selective translation
- Preserve code blocks (triple backticks) unchanged
- Preserve inline code (backticks) unchanged
- Preserve frontmatter (YAML between ---)
- Handle Docusaurus admonitions (:::note, :::tip, etc.)

**Libraries**:
- `remark` + `unified`: Full AST parsing
- `marked`: Lightweight parsing
- `gray-matter`: Frontmatter extraction

**Decision**: Use simple regex-based approach for MVP
- Extract and preserve code blocks
- Translate remaining text
- Reconstruct markdown

---

## 3. Integration Patterns

### 3.1 Frontend → Backend Flow

```
┌─────────────────────────────────────────────────────────────────┐
│                           FRONTEND                              │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ ChapterPage                                             │   │
│  │  ├── AuthProvider (useAuth hook)                        │   │
│  │  │    └── user, isLoading, isAuthenticated              │   │
│  │  │                                                      │   │
│  │  └── TranslateButton                                    │   │
│  │       ├── onClick → POST /api/translate                 │   │
│  │       └── Response → UrduContent                        │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
                              │
                              │ HTTP POST (with session cookie)
                              ▼
┌─────────────────────────────────────────────────────────────────┐
│                           BACKEND                               │
│  ┌─────────────────────────────────────────────────────────┐   │
│  │ /api/translate                                          │   │
│  │  ├── requireAuth middleware                             │   │
│  │  ├── Validate request body                              │   │
│  │  ├── Parse markdown content                             │   │
│  │  ├── Apply translation rules (from skill)               │   │
│  │  └── Return translated content                          │   │
│  └─────────────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────────────┘
```

### 3.2 Component Integration

```
Docusaurus Page Rendering
        │
        ▼
DocItem/Layout (theme component)
        │
        ├── Check: isAuthenticated?
        │         │
        │    YES  │  NO
        │    ▼    │   ▼
        │  Show   │  Hide
        │  Button │  Button
        │
        ▼
TranslateButton
        │
        ├── On Click → API Call
        │
        ▼
UrduContent (conditional render)
        │
        └── RTL styled markdown
```

---

## 4. Risk Assessment

### 4.1 Technical Risks

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Translation skill unavailable | Low | High | Graceful error message, retry |
| Large chapters timeout | Medium | Medium | Set 30s timeout, show progress |
| RTL rendering issues | Medium | Low | Test on multiple browsers |
| Font loading slow | Low | Low | Use font-display: swap |

### 4.2 Integration Risks

| Risk | Likelihood | Impact | Mitigation |
|------|------------|--------|------------|
| Auth state race condition | Low | Medium | Check isLoading before render |
| CORS issues | Low | Medium | Auth backend already configured |
| Content extraction fails | Low | High | Pass content as prop, validate |

---

## 5. Dependencies Confirmed

### 5.1 Existing (No Changes Needed)

| Dependency | Location | Status |
|------------|----------|--------|
| Better-Auth | `/auth/` | Implemented |
| AuthProvider | `/book-write/src/components/Auth/` | Implemented |
| useAuth hook | `/book-write/src/components/Auth/` | Implemented |
| requireAuth middleware | `/auth/src/middleware/auth.ts` | Implemented |
| Hono server | `/auth/src/index.ts` | Running |

### 5.2 New (To Be Created)

| Component | Location | Purpose |
|-----------|----------|---------|
| TranslateButton | `/book-write/src/components/Translation/` | UI button |
| UrduContent | `/book-write/src/components/Translation/` | RTL display |
| translate route | `/auth/src/routes/translate.ts` | Backend API |
| urdu.css | `/book-write/src/css/` | Urdu styling |

---

## 6. Decisions Summary

| Area | Decision | Confidence |
|------|----------|------------|
| Auth integration | Reuse useAuth hook | High |
| Skill invocation | Backend API endpoint | High |
| Content access | Pass as prop from MDX | Medium |
| Font | Google Fonts Noto Nastaliq | High |
| RTL | CSS-only scoped class | High |
| State | Component-level useState | High |
| Caching | In-memory component state | High |

---

**END OF RESEARCH**
