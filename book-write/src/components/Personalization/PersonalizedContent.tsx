import React, { useState } from 'react';
import styles from './Personalization.module.css';
import type { PersonalizedContentProps, PersonalizationSummary } from './types';

/**
 * PersonalizedContent component
 *
 * Renders personalized markdown content with a visual indicator badge
 * and optional summary of personalization adjustments.
 */
export function PersonalizedContent({
  content,
  personalizationSummary,
  className,
}: PersonalizedContentProps): React.ReactElement {
  const [showSummary, setShowSummary] = useState(false);

  /**
   * Process markdown content for display
   * - Preserve code blocks
   * - Render headers, lists, and paragraphs
   */
  const processContent = (text: string): React.ReactNode[] => {
    const elements: React.ReactNode[] = [];
    let key = 0;

    // Split content by code blocks
    const parts = text.split(/(```[\s\S]*?```)/g);

    parts.forEach((part) => {
      if (part.startsWith('```') && part.endsWith('```')) {
        // Code block - render as pre/code
        const codeContent = part.slice(3, -3);
        const firstNewline = codeContent.indexOf('\n');
        const language = firstNewline > 0 ? codeContent.slice(0, firstNewline).trim() : '';
        const code = firstNewline > 0 ? codeContent.slice(firstNewline + 1) : codeContent;

        elements.push(
          <pre key={key++}>
            <code className={language ? `language-${language}` : undefined}>
              {code}
            </code>
          </pre>
        );
      } else if (part.trim()) {
        // Regular text - process for inline elements
        const lines = part.split('\n');
        lines.forEach((line) => {
          if (line.trim()) {
            // Check for headers
            const headerMatch = line.match(/^(#{1,6})\s+(.*)$/);
            if (headerMatch) {
              const level = headerMatch[1].length;
              const headerText = processInlineCode(headerMatch[2]);
              const HeaderTag = `h${level}` as keyof JSX.IntrinsicElements;
              elements.push(
                <HeaderTag key={key++}>{headerText}</HeaderTag>
              );
            }
            // Check for unordered list items
            else if (line.match(/^[-*]\s+/)) {
              const listText = processInlineCode(line.replace(/^[-*]\s+/, ''));
              elements.push(
                <li key={key++}>{listText}</li>
              );
            }
            // Check for numbered list
            else if (line.match(/^\d+\.\s+/)) {
              const listText = processInlineCode(line.replace(/^\d+\.\s+/, ''));
              elements.push(
                <li key={key++}>{listText}</li>
              );
            }
            // Check for blockquote
            else if (line.match(/^>\s+/)) {
              const quoteText = processInlineCode(line.replace(/^>\s+/, ''));
              elements.push(
                <blockquote key={key++}>{quoteText}</blockquote>
              );
            }
            // Regular paragraph
            else {
              elements.push(
                <p key={key++}>{processInlineCode(line)}</p>
              );
            }
          }
        });
      }
    });

    return elements;
  };

  /**
   * Process inline code within text
   * Returns mixed content with code spans
   */
  const processInlineCode = (text: string): React.ReactNode => {
    // Handle bold text
    let processed = text.replace(/\*\*([^*]+)\*\*/g, '<strong>$1</strong>');
    // Handle italic text
    processed = processed.replace(/\*([^*]+)\*/g, '<em>$1</em>');

    const parts = processed.split(/(`[^`]+`)/g);

    if (parts.length === 1 && !processed.includes('<')) {
      return text;
    }

    return parts.map((part, index) => {
      if (part.startsWith('`') && part.endsWith('`')) {
        return (
          <code key={index}>
            {part.slice(1, -1)}
          </code>
        );
      }
      // Handle HTML-like tags from our processing
      if (part.includes('<strong>') || part.includes('<em>')) {
        return (
          <span
            key={index}
            dangerouslySetInnerHTML={{ __html: part }}
          />
        );
      }
      return part;
    });
  };

  /**
   * Render personalization summary section
   */
  const renderSummary = (summary: PersonalizationSummary): React.ReactNode => {
    return (
      <div className={styles.personalizationSummary}>
        <div className={styles.summaryTitle}>
          Personalization Applied:
        </div>
        <ul className={styles.summaryList}>
          {summary.adjustments_made.map((adjustment, index) => (
            <li key={index}>{adjustment}</li>
          ))}
        </ul>
      </div>
    );
  };

  return (
    <div className={`${styles.personalizedContent} ${className || ''}`}>
      {/* Personalization badge */}
      <div className={styles.personalizationBadge}>
        <svg
          className={styles.badgeIcon}
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
        <span>Personalized for you</span>
        {personalizationSummary && (
          <button
            type="button"
            onClick={() => setShowSummary(!showSummary)}
            style={{
              background: 'none',
              border: 'none',
              cursor: 'pointer',
              marginLeft: '0.5rem',
              fontSize: '0.75rem',
              color: 'inherit',
              textDecoration: 'underline',
            }}
            aria-expanded={showSummary}
          >
            {showSummary ? 'Hide details' : 'Show details'}
          </button>
        )}
      </div>

      {/* Optional summary */}
      {showSummary && personalizationSummary && renderSummary(personalizationSummary)}

      {/* Personalized content */}
      <div className={styles.contentText}>
        {processContent(content)}
      </div>
    </div>
  );
}

export default PersonalizedContent;
