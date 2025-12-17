import React from 'react';
import styles from './Translation.module.css';
import type { UrduContentProps } from './types';

/**
 * UrduContent component
 *
 * Renders translated Urdu content with proper RTL styling,
 * Noto Nastaliq Urdu font, and preserves code blocks in LTR.
 */
export function UrduContent({ content, className }: UrduContentProps): React.ReactElement {
  /**
   * Process markdown content for display
   * - Preserve code blocks with LTR direction
   * - Apply RTL to regular text
   */
  const processContent = (text: string): React.ReactNode[] => {
    const elements: React.ReactNode[] = [];
    let key = 0;

    // Split content by code blocks
    const parts = text.split(/(```[\s\S]*?```)/g);

    parts.forEach((part) => {
      if (part.startsWith('```') && part.endsWith('```')) {
        // Code block - render as pre/code with LTR
        const codeContent = part.slice(3, -3);
        const firstNewline = codeContent.indexOf('\n');
        const language = firstNewline > 0 ? codeContent.slice(0, firstNewline).trim() : '';
        const code = firstNewline > 0 ? codeContent.slice(firstNewline + 1) : codeContent;

        elements.push(
          <pre key={key++} style={{ direction: 'ltr', textAlign: 'left' }}>
            <code className={language ? `language-${language}` : undefined}>
              {code}
            </code>
          </pre>
        );
      } else if (part.trim()) {
        // Regular text - process for inline elements
        const lines = part.split('\n');
        lines.forEach((line, lineIndex) => {
          if (line.trim()) {
            // Check for headers
            const headerMatch = line.match(/^(#{1,6})\s+(.*)$/);
            if (headerMatch) {
              const level = headerMatch[1].length;
              const text = processInlineCode(headerMatch[2]);
              const HeaderTag = `h${level}` as keyof JSX.IntrinsicElements;
              elements.push(
                <HeaderTag key={key++}>{text}</HeaderTag>
              );
            }
            // Check for list items
            else if (line.match(/^[-*]\s+/)) {
              const text = processInlineCode(line.replace(/^[-*]\s+/, ''));
              elements.push(
                <li key={key++}>{text}</li>
              );
            }
            // Check for numbered list
            else if (line.match(/^\d+\.\s+/)) {
              const text = processInlineCode(line.replace(/^\d+\.\s+/, ''));
              elements.push(
                <li key={key++}>{text}</li>
              );
            }
            // Regular paragraph
            else {
              elements.push(
                <p key={key++}>{processInlineCode(line)}</p>
              );
            }
          }
          // Add line break between non-empty lines within a part
          else if (lineIndex < lines.length - 1 && lines[lineIndex + 1]?.trim()) {
            // Skip empty lines
          }
        });
      }
    });

    return elements;
  };

  /**
   * Process inline code within text
   * Returns mixed content with code spans in LTR
   */
  const processInlineCode = (text: string): React.ReactNode => {
    const parts = text.split(/(`[^`]+`)/g);

    if (parts.length === 1) {
      return text;
    }

    return parts.map((part, index) => {
      if (part.startsWith('`') && part.endsWith('`')) {
        return (
          <code
            key={index}
            style={{ direction: 'ltr', unicodeBidi: 'isolate' }}
          >
            {part.slice(1, -1)}
          </code>
        );
      }
      return part;
    });
  };

  return (
    <div className={`${styles.urduContent} ${className || ''}`}>
      {/* Urdu translation badge */}
      <div className={styles.urduBadge}>
        <svg
          width="16"
          height="16"
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
        <span>اردو ترجمہ</span>
      </div>

      {/* Translated content */}
      <div className={styles.urduText}>
        {processContent(content)}
      </div>
    </div>
  );
}

export default UrduContent;
