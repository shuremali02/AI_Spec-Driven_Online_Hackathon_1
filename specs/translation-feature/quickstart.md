# Quickstart: Chapter Content Translation Feature

**Feature**: translation-feature
**Date**: 2025-12-17
**Time to Complete**: ~2-3 hours

---

## Prerequisites

Before starting, ensure:

- [ ] Authentication system is running (`/auth/` service)
- [ ] Docusaurus frontend is running (`/book-write/`)
- [ ] You can sign in/sign up successfully
- [ ] Node.js 18+ installed
- [ ] Basic knowledge of React/TypeScript

---

## Step 1: Create Translation Components Directory

```bash
mkdir -p book-write/src/components/Translation
```

---

## Step 2: Create TranslateButton Component

**File**: `book-write/src/components/Translation/TranslateButton.tsx`

```typescript
import React, { useState } from 'react';
import { useAuth } from '@site/src/components/Auth/AuthProvider';
import styles from './Translation.module.css';

interface TranslateButtonProps {
  chapterId: string;
  chapterContent: string;
  onTranslate: (translatedContent: string) => void;
}

type Status = 'idle' | 'loading' | 'success' | 'error';

export default function TranslateButton({
  chapterId,
  chapterContent,
  onTranslate,
}: TranslateButtonProps) {
  const { user } = useAuth();
  const [status, setStatus] = useState<Status>('idle');
  const [showTranslated, setShowTranslated] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // Don't render if not authenticated
  if (!user) {
    return null;
  }

  const handleTranslate = async () => {
    if (status === 'success' && !showTranslated) {
      setShowTranslated(true);
      return;
    }

    setStatus('loading');
    setError(null);

    try {
      const response = await fetch('/api/translate', {
        method: 'POST',
        credentials: 'include',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({
          chapter_id: chapterId,
          content: chapterContent,
        }),
      });

      if (!response.ok) {
        const errorData = await response.json();
        throw new Error(errorData.message || 'Translation failed');
      }

      const data = await response.json();
      setStatus('success');
      setShowTranslated(true);
      onTranslate(data.translated_content);
    } catch (err) {
      setStatus('error');
      setError(err instanceof Error ? err.message : 'Translation failed');
    }
  };

  const handleToggle = () => {
    setShowTranslated(!showTranslated);
  };

  const getButtonLabel = () => {
    if (status === 'loading') return 'Translating...';
    if (status === 'error') return 'Retry Translation';
    if (showTranslated) return 'Show Original';
    return 'Translate to Urdu';
  };

  return (
    <div className={styles.translateContainer}>
      <button
        onClick={status === 'success' ? handleToggle : handleTranslate}
        disabled={status === 'loading'}
        className={`${styles.translateButton} ${status === 'error' ? styles.error : ''}`}
      >
        {getButtonLabel()}
      </button>
      {error && <p className={styles.errorMessage}>{error}</p>}
    </div>
  );
}
```

---

## Step 3: Create UrduContent Component

**File**: `book-write/src/components/Translation/UrduContent.tsx`

```typescript
import React from 'react';
import styles from './Translation.module.css';

interface UrduContentProps {
  content: string;
}

export default function UrduContent({ content }: UrduContentProps) {
  return (
    <div className={styles.urduContent}>
      <div className={styles.urduBadge}>اردو ترجمہ</div>
      <div
        className={styles.urduText}
        dangerouslySetInnerHTML={{ __html: content }}
      />
    </div>
  );
}
```

---

## Step 4: Create CSS Styles

**File**: `book-write/src/components/Translation/Translation.module.css`

```css
.translateContainer {
  margin: 1rem 0;
  padding: 1rem;
  border-radius: 8px;
  background: var(--ifm-background-surface-color);
}

.translateButton {
  padding: 12px 24px;
  font-size: 1rem;
  font-weight: 600;
  color: white;
  background: var(--ifm-color-primary);
  border: none;
  border-radius: 8px;
  cursor: pointer;
  transition: background 0.2s;
}

.translateButton:hover:not(:disabled) {
  background: var(--ifm-color-primary-dark);
}

.translateButton:disabled {
  opacity: 0.7;
  cursor: not-allowed;
}

.translateButton.error {
  background: var(--ifm-color-danger);
}

.errorMessage {
  color: var(--ifm-color-danger);
  margin-top: 0.5rem;
  font-size: 0.875rem;
}

.urduContent {
  margin-top: 2rem;
  padding: 1.5rem;
  background: #faf8f5;
  border-radius: 8px;
  border-left: 4px solid var(--ifm-color-primary);
}

.urduBadge {
  display: inline-block;
  padding: 4px 12px;
  background: var(--ifm-color-primary);
  color: white;
  border-radius: 4px;
  font-size: 0.875rem;
  margin-bottom: 1rem;
}

.urduText {
  direction: rtl;
  text-align: right;
  font-family: "Noto Nastaliq Urdu", "Jameel Noori Nastaleeq", serif;
  font-size: 1.125rem;
  line-height: 2;
}

.urduText pre,
.urduText code {
  direction: ltr;
  text-align: left;
  font-family: monospace;
}

@media (prefers-color-scheme: dark) {
  .urduContent {
    background: #2a2a2a;
  }
}
```

---

## Step 5: Add Translation API Route

**File**: `auth/src/routes/translate.ts`

```typescript
import { Hono } from 'hono';
import { requireAuth, getAuthContext } from '../middleware/auth.js';

const routes = new Hono();

// Require authentication for all translation routes
routes.use('*', requireAuth);

// POST /api/translate
routes.post('/translate', async (c) => {
  const body = await c.req.json();

  // Validate request
  if (!body.chapter_id || typeof body.chapter_id !== 'string') {
    return c.json({
      success: false,
      error: 'INVALID_REQUEST',
      message: 'Chapter ID is required',
    }, 400);
  }

  if (!body.content || typeof body.content !== 'string') {
    return c.json({
      success: false,
      error: 'INVALID_REQUEST',
      message: 'Content is required',
    }, 400);
  }

  if (body.content.length > 100000) {
    return c.json({
      success: false,
      error: 'CONTENT_TOO_LONG',
      message: 'Content exceeds maximum length',
    }, 400);
  }

  try {
    // Apply translation logic (simplified - implement full logic later)
    const translatedContent = translateToUrdu(body.content);

    return c.json({
      success: true,
      translated_content: translatedContent,
      chapter_id: body.chapter_id,
      word_count: body.content.split(/\s+/).length,
      timestamp: new Date().toISOString(),
    });
  } catch (error) {
    console.error('Translation error:', error);
    return c.json({
      success: false,
      error: 'TRANSLATION_FAILED',
      message: 'Translation failed. Please try again.',
    }, 500);
  }
});

// Simple translation function (placeholder - implement full logic)
function translateToUrdu(content: string): string {
  // Preserve code blocks
  const codeBlocks: string[] = [];
  let processed = content.replace(/```[\s\S]*?```/g, (match) => {
    codeBlocks.push(match);
    return `__CODE_BLOCK_${codeBlocks.length - 1}__`;
  });

  // Preserve inline code
  const inlineCode: string[] = [];
  processed = processed.replace(/`[^`]+`/g, (match) => {
    inlineCode.push(match);
    return `__INLINE_CODE_${inlineCode.length - 1}__`;
  });

  // TODO: Implement actual translation using translate-to-urdu skill
  // For now, return placeholder
  const translated = `[اردو ترجمہ]\n\n${processed}`;

  // Restore code blocks
  codeBlocks.forEach((block, i) => {
    translated.replace(`__CODE_BLOCK_${i}__`, block);
  });

  // Restore inline code
  inlineCode.forEach((code, i) => {
    translated.replace(`__INLINE_CODE_${i}__`, code);
  });

  return translated;
}

export default routes;
```

---

## Step 6: Register Translation Route

**File**: `auth/src/index.ts` (add import and route)

```typescript
// Add to imports
import translateRoutes from './routes/translate.js';

// Add after existing routes
app.route('/api', translateRoutes);
```

---

## Step 7: Add Google Font for Urdu

**File**: `book-write/docusaurus.config.ts` (add to themeConfig)

```typescript
// Add to config object
headTags: [
  {
    tagName: 'link',
    attributes: {
      rel: 'preconnect',
      href: 'https://fonts.googleapis.com',
    },
  },
  {
    tagName: 'link',
    attributes: {
      rel: 'stylesheet',
      href: 'https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu&display=swap',
    },
  },
],
```

---

## Step 8: Test the Feature

1. **Start the auth service**:
   ```bash
   cd auth && npm run dev
   ```

2. **Start Docusaurus**:
   ```bash
   cd book-write && npm start
   ```

3. **Test flow**:
   - Sign in to the application
   - Navigate to any chapter
   - Verify "Translate to Urdu" button appears
   - Click the button and verify translation
   - Click "Show Original" to toggle back

---

## Verification Checklist

- [ ] Button only visible when logged in
- [ ] Button hidden for anonymous users
- [ ] Loading state shows during translation
- [ ] Error message displays on failure
- [ ] Toggle between original/translated works
- [ ] Code blocks preserved in translation
- [ ] RTL styling applied to Urdu content

---

## Next Steps

After quickstart is working:

1. Implement full translation logic using skill rules
2. Add rate limiting
3. Add comprehensive error handling
4. Test on all chapters
5. Responsive design testing

---

**END OF QUICKSTART**
