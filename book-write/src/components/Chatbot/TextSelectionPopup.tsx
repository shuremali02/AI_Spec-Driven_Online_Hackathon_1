import React, { useState, useEffect, useCallback, useRef } from 'react';
import './TextSelectionPopup.css';

interface TextSelectionPopupProps {
  onAskAboutSelection: (selectedText: string) => void;
}

const TextSelectionPopup: React.FC<TextSelectionPopupProps> = ({ onAskAboutSelection }) => {
  const [isVisible, setIsVisible] = useState(false);
  const [position, setPosition] = useState({ x: 0, y: 0 });
  const [selectedText, setSelectedText] = useState('');
  const popupRef = useRef<HTMLDivElement>(null);
  const justShownRef = useRef(false);

  const handleMouseUp = useCallback((e: MouseEvent) => {
    // Ignore if clicking on the popup itself
    if (popupRef.current?.contains(e.target as Node)) {
      return;
    }

    // Small delay to let selection complete
    setTimeout(() => {
      const selection = window.getSelection();
      const text = selection?.toString().trim();

      if (text && text.length > 3 && text.length < 1000) {
        try {
          const range = selection?.getRangeAt(0);
          const rect = range?.getBoundingClientRect();

          if (rect && rect.width > 0) {
            // Position popup above the selection (using fixed positioning)
            const x = rect.left + rect.width / 2;
            const y = rect.top - 10; // No scrollY needed for fixed position

            setPosition({ x, y });
            setSelectedText(text);
            setIsVisible(true);
            justShownRef.current = true;

            // Reset justShown after a brief moment
            setTimeout(() => {
              justShownRef.current = false;
            }, 200);
          }
        } catch (err) {
          // Selection might be invalid
          console.log('Selection error:', err);
        }
      } else {
        if (!justShownRef.current) {
          setIsVisible(false);
          setSelectedText('');
        }
      }
    }, 10);
  }, []);

  const handleClickOutside = useCallback((e: MouseEvent) => {
    // Don't hide if we just showed the popup
    if (justShownRef.current) return;

    // Hide popup when clicking outside
    if (popupRef.current && !popupRef.current.contains(e.target as Node)) {
      setIsVisible(false);
    }
  }, []);

  const handleScroll = useCallback(() => {
    setIsVisible(false);
  }, []);

  const handleKeyDown = useCallback((e: KeyboardEvent) => {
    if (e.key === 'Escape') {
      setIsVisible(false);
    }
  }, []);

  useEffect(() => {
    document.addEventListener('mouseup', handleMouseUp);
    document.addEventListener('mousedown', handleClickOutside);
    document.addEventListener('scroll', handleScroll, true);
    document.addEventListener('keydown', handleKeyDown);

    return () => {
      document.removeEventListener('mouseup', handleMouseUp);
      document.removeEventListener('mousedown', handleClickOutside);
      document.removeEventListener('scroll', handleScroll, true);
      document.removeEventListener('keydown', handleKeyDown);
    };
  }, [handleMouseUp, handleClickOutside, handleScroll, handleKeyDown]);

  const handleAskClick = (e: React.MouseEvent) => {
    e.preventDefault();
    e.stopPropagation();

    if (selectedText) {
      onAskAboutSelection(selectedText);
      setIsVisible(false);
      setSelectedText('');
      // Clear the selection
      window.getSelection()?.removeAllRanges();
    }
  };

  if (!isVisible) return null;

  return (
    <div
      ref={popupRef}
      className="text-selection-popup"
      style={{
        left: `${position.x}px`,
        top: `${position.y}px`,
      }}
      onMouseDown={(e) => e.stopPropagation()}
    >
      <button className="ask-button" onClick={handleAskClick} onMouseDown={(e) => e.stopPropagation()}>
        <svg width="16" height="16" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
          <path d="M8.5 19H8C4 19 2 18 2 13V8C2 4 4 2 8 2H16C20 2 22 4 22 8V13C22 17 20 19 16 19H15.5C15.19 19 14.89 19.15 14.7 19.4L13.2 21.4C12.54 22.28 11.46 22.28 10.8 21.4L9.3 19.4C9.14 19.18 8.77 19 8.5 19Z" stroke="currentColor" strokeWidth="1.5" strokeMiterlimit="10" strokeLinecap="round" strokeLinejoin="round"/>
          <path d="M12 11V11.01" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          <path d="M12 8C10.34 8 9 6.66 9 5" stroke="currentColor" strokeWidth="1.5" strokeLinecap="round" strokeLinejoin="round"/>
        </svg>
        Ask about this
      </button>
      <div className="popup-arrow"></div>
    </div>
  );
};

export default TextSelectionPopup;
