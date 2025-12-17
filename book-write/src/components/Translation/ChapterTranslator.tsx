import React, { useState, useCallback, useEffect, useRef } from 'react';
import { useAuth } from '@site/src/components/Auth/AuthProvider';
import { AUTH_BASE_URL } from '@site/src/components/Auth/authClient';
import { UrduContent } from './UrduContent';
import styles from './Translation.module.css';
import type {
  TranslationState,
  TranslateResponse,
  TranslationErrorType,
} from './types';
import {
  BUTTON_LABELS,
  TRANSLATION_CONFIG,
  ERROR_MESSAGES,
} from './types';

interface ChapterTranslatorProps {
  chapterId: string;
}

// Store original TOC text for restoration
interface TocItem {
  element: HTMLElement;
  originalText: string;
}

/**
 * ChapterTranslator component
 *
 * Fetches and translates the entire chapter content.
 * Uses DOM to get the actual rendered chapter content.
 */
export function ChapterTranslator({ chapterId }: ChapterTranslatorProps): React.ReactElement | null {
  const { user, isLoading: authLoading } = useAuth();
  const [chapterContent, setChapterContent] = useState<string>('');
  const originalTocItems = useRef<TocItem[]>([]);
  const [tocTranslated, setTocTranslated] = useState(false);

  const [state, setState] = useState<TranslationState>({
    status: 'idle',
    translatedContent: null,
    showTranslated: false,
    errorMessage: null,
  });

  // Extract chapter content from the DOM on mount
  useEffect(() => {
    const extractContent = () => {
      // Find the main article content
      const article = document.querySelector('article');
      if (article) {
        // Get text content, preserving structure
        const content = extractTextContent(article);
        setChapterContent(content);
      }
    };

    // Wait for DOM to be ready
    const timer = setTimeout(extractContent, 500);
    return () => clearTimeout(timer);
  }, []);

  /**
   * Extract text content from an element while preserving markdown-like structure
   */
  const extractTextContent = (element: Element): string => {
    let content = '';

    const walk = (node: Node) => {
      if (node.nodeType === Node.TEXT_NODE) {
        content += node.textContent;
      } else if (node.nodeType === Node.ELEMENT_NODE) {
        const el = node as Element;
        const tagName = el.tagName.toLowerCase();

        // Skip the translate button itself
        if (el.classList.contains('translateButton') || el.closest('[class*="translateButton"]')) {
          return;
        }

        // Add markdown-like formatting
        if (tagName === 'h1') content += '\n# ';
        else if (tagName === 'h2') content += '\n## ';
        else if (tagName === 'h3') content += '\n### ';
        else if (tagName === 'h4') content += '\n#### ';
        else if (tagName === 'p') content += '\n\n';
        else if (tagName === 'li') content += '\n* ';
        else if (tagName === 'pre' || tagName === 'code') {
          // Preserve code blocks
          content += '\n```\n' + el.textContent + '\n```\n';
          return; // Don't recurse into code blocks
        }
        else if (tagName === 'br') content += '\n';

        // Recurse into children
        el.childNodes.forEach(walk);

        // Add newlines after block elements
        if (['h1', 'h2', 'h3', 'h4', 'p', 'div', 'ul', 'ol'].includes(tagName)) {
          content += '\n';
        }
      }
    };

    walk(element);
    return content.trim();
  };

  /**
   * Translate the right sidebar TOC (Table of Contents)
   */
  const translateToc = useCallback(async () => {
    const tocContainer = document.querySelector('.table-of-contents');
    if (!tocContainer) return;

    const tocLinks = tocContainer.querySelectorAll('a');
    if (tocLinks.length === 0) return;

    // Store original text if not already stored
    if (originalTocItems.current.length === 0) {
      tocLinks.forEach((link) => {
        originalTocItems.current.push({
          element: link as HTMLElement,
          originalText: link.textContent || '',
        });
      });
    }

    // Collect TOC text for translation
    const tocTexts = originalTocItems.current.map(item => item.originalText).join('\n');

    try {
      const response = await fetch(`${AUTH_BASE_URL}${TRANSLATION_CONFIG.ENDPOINT}`, {
        method: 'POST',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_id: `${chapterId}-toc`,
          content: tocTexts,
        }),
      });

      const data: TranslateResponse = await response.json();

      if (data.success && data.translated_content) {
        const translatedLines = data.translated_content.split('\n');

        // Apply translated text to TOC items
        originalTocItems.current.forEach((item, index) => {
          if (translatedLines[index]) {
            item.element.textContent = translatedLines[index];
            item.element.style.fontFamily = "'Noto Nastaliq Urdu', 'Jameel Noori Nastaleeq', serif";
            item.element.style.direction = 'rtl';
          }
        });

        // Apply RTL to TOC container
        (tocContainer as HTMLElement).style.direction = 'rtl';
        (tocContainer as HTMLElement).style.textAlign = 'right';

        setTocTranslated(true);
      }
    } catch (error) {
      console.error('[TOC Translation] Error:', error);
    }
  }, [chapterId]);

  /**
   * Restore original TOC text
   */
  const restoreToc = useCallback(() => {
    const tocContainer = document.querySelector('.table-of-contents');

    originalTocItems.current.forEach((item) => {
      item.element.textContent = item.originalText;
      item.element.style.fontFamily = '';
      item.element.style.direction = '';
    });

    if (tocContainer) {
      (tocContainer as HTMLElement).style.direction = '';
      (tocContainer as HTMLElement).style.textAlign = '';
    }

    setTocTranslated(false);
  }, []);

  const handleTranslate = useCallback(async () => {
    // If we have cached content, just toggle the view
    if (state.translatedContent) {
      const newShowTranslated = !state.showTranslated;
      setState((prev) => ({
        ...prev,
        showTranslated: newShowTranslated,
      }));

      // Toggle TOC translation
      if (newShowTranslated && !tocTranslated) {
        translateToc();
      } else if (!newShowTranslated && tocTranslated) {
        restoreToc();
      }
      return;
    }

    if (!chapterContent) {
      setState((prev) => ({
        ...prev,
        status: 'error',
        errorMessage: 'Chapter content not found. Please refresh the page.',
      }));
      return;
    }

    // Start loading
    setState((prev) => ({
      ...prev,
      status: 'loading',
      errorMessage: null,
    }));

    try {
      const controller = new AbortController();
      // 2 minute timeout for large chapters
      const timeoutId = setTimeout(() => controller.abort(), 120000);

      const response = await fetch(`${AUTH_BASE_URL}${TRANSLATION_CONFIG.ENDPOINT}`, {
        method: 'POST',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          chapter_id: chapterId,
          content: chapterContent,
        }),
        signal: controller.signal,
      });

      clearTimeout(timeoutId);

      const data: TranslateResponse = await response.json();

      if (data.success) {
        setState({
          status: 'success',
          translatedContent: data.translated_content,
          showTranslated: true,
          errorMessage: null,
        });

        // Also translate TOC
        translateToc();
      } else {
        const errorType = data.error as TranslationErrorType;
        const errorMessage = ERROR_MESSAGES[errorType] || data.message || 'Translation failed';

        setState((prev) => ({
          ...prev,
          status: 'error',
          errorMessage,
        }));
      }
    } catch (error) {
      let errorMessage = ERROR_MESSAGES.TRANSLATION_FAILED;

      if (error instanceof Error) {
        if (error.name === 'AbortError') {
          errorMessage = ERROR_MESSAGES.TIMEOUT;
        } else if (error.message.includes('fetch') || error.message.includes('network')) {
          errorMessage = ERROR_MESSAGES.NETWORK_ERROR;
        }
      }

      setState((prev) => ({
        ...prev,
        status: 'error',
        errorMessage,
      }));
    }
  }, [chapterId, chapterContent, state.translatedContent, state.showTranslated, tocTranslated, translateToc, restoreToc]);

  const handleRetry = useCallback(() => {
    setState({
      status: 'idle',
      translatedContent: null,
      showTranslated: false,
      errorMessage: null,
    });
    // Reset TOC
    restoreToc();
  }, [restoreToc]);

  // Retry translation after state reset
  useEffect(() => {
    if (state.status === 'idle' && state.translatedContent === null && chapterContent) {
      // Only auto-retry if this was triggered by handleRetry
    }
  }, [state.status, state.translatedContent, chapterContent]);

  // Don't render if auth is loading
  if (authLoading) {
    return null;
  }

  // Don't render if user is not authenticated
  if (!user) {
    return null;
  }

  const { status, showTranslated, errorMessage } = state;

  const getButtonLabel = (): string => {
    if (status === 'loading') {
      return BUTTON_LABELS.LOADING;
    }
    if (status === 'error') {
      return BUTTON_LABELS.RETRY;
    }
    if (showTranslated) {
      return BUTTON_LABELS.SHOW_ORIGINAL;
    }
    return BUTTON_LABELS.TRANSLATE;
  };

  const getButtonStateClass = (): string => {
    switch (status) {
      case 'loading':
        return styles.buttonLoading;
      case 'success':
        return showTranslated ? styles.buttonSuccess : styles.buttonIdle;
      case 'error':
        return styles.buttonError;
      default:
        return styles.buttonIdle;
    }
  };

  const isDisabled = status === 'loading';

  return (
    <div>
      <button
        type="button"
        className={`${styles.translateButton} ${getButtonStateClass()}`}
        onClick={handleTranslate}
        disabled={isDisabled}
        aria-label={showTranslated ? 'Show original English content' : 'Translate content to Urdu'}
        aria-busy={status === 'loading'}
      >
        {status === 'loading' ? (
          <span className={styles.loadingSpinner} aria-hidden="true" />
        ) : (
          <svg
            className={styles.buttonIcon}
            viewBox="0 0 24 24"
            fill="none"
            stroke="currentColor"
            strokeWidth="2"
            strokeLinecap="round"
            strokeLinejoin="round"
            aria-hidden="true"
          >
            <path d="m5 8 6 6" />
            <path d="m4 14 6-6 2-3" />
            <path d="M2 5h12" />
            <path d="M7 2h1" />
            <path d="m22 22-5-10-5 10" />
            <path d="M14 18h6" />
          </svg>
        )}
        <span>{getButtonLabel()}</span>
      </button>

      {status === 'error' && errorMessage && (
        <div className={styles.errorMessage} role="alert">
          <span>{errorMessage}</span>
          <button
            type="button"
            className={styles.retryLink}
            onClick={handleRetry}
            aria-label="Retry translation"
          >
            Try again
          </button>
        </div>
      )}

      {/* Display translated Urdu content when available */}
      {showTranslated && state.translatedContent && (
        <UrduContent content={state.translatedContent} />
      )}
    </div>
  );
}

export default ChapterTranslator;
