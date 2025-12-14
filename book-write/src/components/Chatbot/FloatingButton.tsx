import React, { useState } from 'react';
import './FloatingButton.css';

interface FloatingButtonProps {
  onClick: () => void;
  isOpen?: boolean;
}

const FloatingButton: React.FC<FloatingButtonProps> = ({ onClick, isOpen = false }) => {
  const [isHovered, setIsHovered] = useState(false);

  return (
    <button
      className={`chatbot-floating-button ${isOpen ? 'open' : ''} ${isHovered ? 'hovered' : ''}`}
      onClick={onClick}
      onMouseEnter={() => setIsHovered(true)}
      onMouseLeave={() => setIsHovered(false)}
      aria-label={isOpen ? "Close chat" : "Open chat"}
      title={isOpen ? "Close chat" : "Open chatbot"}
    >
      <div className="chatbot-icon">
        {isOpen ? (
          // Close icon (X)
          <svg width="24" height="24" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
            <path d="M18 6L6 18M6 6L18 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
          </svg>
        ) : (
          // White robot icon for purple gradient background
          <svg width="36" height="36" viewBox="0 0 200 200" xmlns="http://www.w3.org/2000/svg">
            {/* Robot Head Background */}
            <rect x="40" y="30" width="120" height="100" rx="20" fill="white"/>
            {/* Robot Face Plate */}
            <rect x="50" y="45" width="100" height="70" rx="12" fill="rgba(102,126,234,0.15)"/>
            {/* Robot Eyes */}
            <circle cx="75" cy="75" r="15" fill="white"/>
            <circle cx="125" cy="75" r="15" fill="white"/>
            {/* Eye Pupils */}
            <circle cx="75" cy="75" r="8" fill="rgba(102,126,234,0.6)"/>
            <circle cx="125" cy="75" r="8" fill="rgba(102,126,234,0.6)"/>
            {/* Eye Highlights */}
            <circle cx="72" cy="72" r="3" fill="white"/>
            <circle cx="122" cy="72" r="3" fill="white"/>
            {/* Robot Mouth/Speaker */}
            <rect x="70" y="95" width="60" height="10" rx="5" fill="white" opacity="0.8"/>
            {/* Antenna */}
            <rect x="95" y="10" width="10" height="25" rx="5" fill="white"/>
            <circle cx="100" cy="10" r="8" fill="white" opacity="0.9"/>
            {/* Robot Body */}
            <rect x="55" y="135" width="90" height="50" rx="10" fill="white"/>
            {/* Body Details */}
            <rect x="70" y="145" width="60" height="8" rx="4" fill="rgba(102,126,234,0.3)"/>
            <rect x="70" y="160" width="40" height="8" rx="4" fill="rgba(102,126,234,0.3)"/>
            {/* Arms */}
            <rect x="30" y="140" width="20" height="35" rx="8" fill="white" opacity="0.9"/>
            <rect x="150" y="140" width="20" height="35" rx="8" fill="white" opacity="0.9"/>
            {/* Hands */}
            <circle cx="40" cy="180" r="10" fill="white"/>
            <circle cx="160" cy="180" r="10" fill="white"/>
          </svg>
        )}
      </div>
      {!isOpen && isHovered && (
        <span className="chatbot-tooltip">Chat with textbook</span>
      )}
    </button>
  );
};

export default FloatingButton;