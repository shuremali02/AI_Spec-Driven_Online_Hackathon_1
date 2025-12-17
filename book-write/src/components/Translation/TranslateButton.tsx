import React, { useState, useCallback } from 'react';
import { useAuth } from '@site/src/components/Auth/AuthProvider';
import { AUTH_BASE_URL } from '@site/src/components/Auth/authClient';
import { UrduContent } from './UrduContent';
import styles from './Translation.module.css';
import type {
  TranslateButtonProps,
  TranslationState,
  TranslateResponse,
  TranslationErrorType,
} from './types';
import {
  BUTTON_LABELS,
  TRANSLATION_CONFIG,
  ERROR_MESSAGES,
} from './types';

/**
 * TranslateButton component
 *
 * Renders a translation button that only appears for authenticated users.
 * Handles the translation API call, caching, and toggle between original/translated content.
 */
export function TranslateButton({
  chapterId,
  chapterContent,
  onTranslationComplete,
  onToggle,
  className,
}: TranslateButtonProps): React.ReactElement | null {
  const { user, isLoading: authLoading } = useAuth();

  const [state, setState] = useState<TranslationState>({
    status: 'idle',
    translatedContent: null,
    showTranslated: false,
    errorMessage: null,
  });

  /**
   * Handle translation API call
   */
  const handleTranslate = useCallback(async () => {
    // If we have cached content, just toggle the view
    if (state.translatedContent) {
      const newShowTranslated = !state.showTranslated;
      setState((prev) => ({
        ...prev,
        showTranslated: newShowTranslated,
      }));
      onToggle?.(newShowTranslated);
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
      const timeoutId = setTimeout(() => controller.abort(), TRANSLATION_CONFIG.TIMEOUT_MS);

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
        onTranslationComplete?.(data.translated_content);
        onToggle?.(true);
      } else {
        // Handle error response
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
  }, [chapterId, chapterContent, state.translatedContent, state.showTranslated, onTranslationComplete, onToggle]);

  /**
   * Handle toggle between original and translated content
   */
  const handleToggle = useCallback(() => {
    if (state.translatedContent) {
      const newShowTranslated = !state.showTranslated;
      setState((prev) => ({
        ...prev,
        showTranslated: newShowTranslated,
      }));
      onToggle?.(newShowTranslated);
    }
  }, [state.translatedContent, state.showTranslated, onToggle]);

  /**
   * Handle retry after error
   */
  const handleRetry = useCallback(() => {
    setState({
      status: 'idle',
      translatedContent: null,
      showTranslated: false,
      errorMessage: null,
    });
    // Immediately trigger translation
    handleTranslate();
  }, [handleTranslate]);

  // Don't render if auth is loading
  if (authLoading) {
    return null;
  }

  // Don't render if user is not authenticated
  if (!user) {
    return null;
  }

  // Determine button state and label
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
    <div className={className}>
      <button
        type="button"
        className={`${styles.translateButton} ${getButtonStateClass()}`}
        onClick={showTranslated && status === 'success' ? handleToggle : handleTranslate}
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

export default TranslateButton;
