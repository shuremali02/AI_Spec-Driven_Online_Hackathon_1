import React, { useState, useCallback } from 'react';
import { createPortal } from 'react-dom';
import { useAuth } from '@site/src/components/Auth/AuthProvider';
import { PersonalizedContent } from './PersonalizedContent';
import { hasCompleteProfile } from './utils';
import styles from './Personalization.module.css';
import type {
  PersonalizationState,
  PersonalizeResponse,
  PersonalizationErrorType,
} from './types';
import {
  BUTTON_LABELS,
  PERSONALIZATION_CONFIG,
  ERROR_MESSAGES,
} from './types';

interface ChapterPersonalizerProps {
  chapterId: string;
}

/**
 * ChapterPersonalizer component
 *
 * Renders a personalization button that only appears for authenticated users
 * with a complete profile. The backend fetches chapter content by chapterId,
 * so no DOM extraction is needed.
 */
export function ChapterPersonalizer({ chapterId }: ChapterPersonalizerProps): React.ReactElement | null {
  const { user, profile, isLoading: authLoading } = useAuth();

  const [state, setState] = useState<PersonalizationState>({
    status: 'idle',
    personalizedContent: null,
    showPersonalized: false,
    errorMessage: null,
    personalizationSummary: null,
  });

  /**
   * Handle personalization API call
   */
  const handlePersonalize = useCallback(async () => {
    // If we have cached content, just toggle the view
    if (state.personalizedContent) {
      const newShowPersonalized = !state.showPersonalized;
      setState((prev) => ({
        ...prev,
        showPersonalized: newShowPersonalized,
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
      // First, fetch the chapter content from the current page
      console.log('[Personalization] Starting personalization for chapter:', chapterId);
      let chapterContent = '';

      // Try to get content from the current page
      const contentElement = document.querySelector('article');
      if (contentElement) {
        // Extract the main content, excluding the translator/personalizer buttons
        const clone = contentElement.cloneNode(true) as HTMLElement;
        // Remove the personalization and translation buttons
        const buttonContainer = clone.querySelector('div:has(.personalizeButton), div:has(.translateButton)');
        if (buttonContainer) {
          buttonContainer.remove();
        }
        chapterContent = clone.innerText || clone.textContent || '';
      } else {
        // Fallback: try to get content from the main content area
        const mainContent = document.querySelector('main') || document.querySelector('#__docusaurus');
        if (mainContent) {
          const clone = mainContent.cloneNode(true) as HTMLElement;
          // Remove buttons to avoid including them in content
          const buttons = clone.querySelectorAll('button');
          buttons.forEach(btn => {
            if (btn.classList.contains('personalizeButton') ||
                btn.classList.contains('translateButton')) {
              btn.remove();
            }
          });
          chapterContent = clone.innerText || clone.textContent || '';
        }
      }

      const controller = new AbortController();
      const timeoutId = setTimeout(() => controller.abort(), PERSONALIZATION_CONFIG.TIMEOUT_MS);

      // Use FastAPI backend for personalization
      const apiUrl = `${PERSONALIZATION_CONFIG.BACKEND_URL}${PERSONALIZATION_CONFIG.ENDPOINT}`;
      console.log('[Personalization] Calling API:', apiUrl);
      console.log('[Personalization] Content length:', chapterContent.length);

      const response = await fetch(apiUrl, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        credentials: 'include',  // Send cookies for authentication
        body: JSON.stringify({
          chapter_id: chapterId,
          chapter_content: chapterContent,
          user_id: user?.id,  // Send logged-in user's ID
        }),
        signal: controller.signal,
      });

      console.log('[Personalization] Response status:', response.status);

      clearTimeout(timeoutId);

      const data: PersonalizeResponse = await response.json();

      if (data.success) {
        setState({
          status: 'success',
          personalizedContent: data.personalized_content,
          showPersonalized: true,
          errorMessage: null,
          personalizationSummary: data.personalization_summary,
        });
      } else {
        // Handle error response
        const errorType = data.error as PersonalizationErrorType;
        const errorMessage = ERROR_MESSAGES[errorType] || data.message || 'Personalization failed';

        setState((prev) => ({
          ...prev,
          status: 'error',
          errorMessage,
        }));
      }
    } catch (error) {
      console.error('[Personalization] Error:', error);
      let errorMessage = ERROR_MESSAGES.PERSONALIZATION_FAILED;

      if (error instanceof Error) {
        console.error('[Personalization] Error name:', error.name, 'Message:', error.message);
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
  }, [chapterId, state.personalizedContent, state.showPersonalized]);

  /**
   * Handle retry after error
   */
  const handleRetry = useCallback(() => {
    setState({
      status: 'idle',
      personalizedContent: null,
      showPersonalized: false,
      errorMessage: null,
      personalizationSummary: null,
    });
    // Immediately trigger personalization
    handlePersonalize();
  }, [handlePersonalize]);

  // Don't render if auth is loading
  if (authLoading) {
    return null;
  }

  // Don't render if user is not authenticated
  if (!user) {
    return null;
  }

  // Don't render if user doesn't have a complete profile
  if (!hasCompleteProfile(profile)) {
    return null;
  }

  // Determine button state and label
  const { status, showPersonalized, errorMessage } = state;

  const getButtonLabel = (): string => {
    if (status === 'loading') {
      return BUTTON_LABELS.LOADING;
    }
    if (status === 'error') {
      return BUTTON_LABELS.RETRY;
    }
    if (showPersonalized) {
      return BUTTON_LABELS.SHOW_ORIGINAL;
    }
    if (state.personalizedContent && !showPersonalized) {
      return BUTTON_LABELS.PERSONALIZE_AGAIN;
    }
    return BUTTON_LABELS.PERSONALIZE;
  };

  const getButtonStateClass = (): string => {
    switch (status) {
      case 'loading':
        return styles.buttonLoading;
      case 'success':
        return showPersonalized ? styles.buttonSuccess : styles.buttonIdle;
      case 'error':
        return styles.buttonError;
      default:
        return styles.buttonIdle;
    }
  };

  const isDisabled = status === 'loading';

  return (
    <>
      <button
        type="button"
        className={`${styles.personalizeButton} ${getButtonStateClass()}`}
        onClick={handlePersonalize}
        disabled={isDisabled}
        aria-label={showPersonalized ? 'Show original content' : 'Personalize content based on your profile'}
        aria-busy={status === 'loading'}
        title="Adapt content to your experience level and background"
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
            <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
            <circle cx="12" cy="7" r="4" />
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
            aria-label="Retry personalization"
          >
            Try again
          </button>
        </div>
      )}

      {/* Render PersonalizedContent via portal to avoid flex container issues */}
      {showPersonalized && state.personalizedContent && typeof document !== 'undefined' && (() => {
        const article = document.querySelector('article');
        const buttonContainer = article?.querySelector('div[style*="flex"]');
        if (buttonContainer) {
          let contentContainer = document.getElementById('personalized-content-portal');
          if (!contentContainer) {
            contentContainer = document.createElement('div');
            contentContainer.id = 'personalized-content-portal';
            contentContainer.style.cssText = 'width: 100%; max-width: 100%;';
            buttonContainer.insertAdjacentElement('afterend', contentContainer);
          }
          return createPortal(
            <PersonalizedContent
              content={state.personalizedContent}
              personalizationSummary={state.personalizationSummary || undefined}
            />,
            contentContainer
          );
        }
        return null;
      })()}
    </>
  );
}

export default ChapterPersonalizer;
