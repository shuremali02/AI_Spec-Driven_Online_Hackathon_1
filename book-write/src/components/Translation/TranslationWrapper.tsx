import React, { useState, useCallback } from 'react';
import { TranslateButton } from './TranslateButton';
import { UrduContent } from './UrduContent';
import styles from './Translation.module.css';
import type { TranslationWrapperProps } from './types';

/**
 * TranslationWrapper component
 *
 * Wraps chapter content and provides translation functionality.
 * Shows TranslateButton at top and conditionally renders
 * either original content or translated UrduContent.
 */
export function TranslationWrapper({
  chapterId,
  rawMarkdown,
  children,
}: TranslationWrapperProps): React.ReactElement {
  const [translatedContent, setTranslatedContent] = useState<string | null>(null);
  const [showTranslated, setShowTranslated] = useState(false);

  /**
   * Handle successful translation
   */
  const handleTranslationComplete = useCallback((content: string) => {
    setTranslatedContent(content);
    setShowTranslated(true);
  }, []);

  /**
   * Handle toggle between original and translated
   */
  const handleToggle = useCallback((show: boolean) => {
    setShowTranslated(show);
  }, []);

  return (
    <div className={styles.translationWrapper}>
      {/* Translation button - only visible to authenticated users */}
      <div className={styles.buttonContainer}>
        <TranslateButton
          chapterId={chapterId}
          chapterContent={rawMarkdown}
          onTranslationComplete={handleTranslationComplete}
          onToggle={handleToggle}
        />
      </div>

      {/* Content display - either original or translated */}
      {showTranslated && translatedContent ? (
        <UrduContent content={translatedContent} />
      ) : (
        children
      )}
    </div>
  );
}

export default TranslationWrapper;
