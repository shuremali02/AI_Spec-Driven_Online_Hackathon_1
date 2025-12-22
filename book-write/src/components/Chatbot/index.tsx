import React, { useState, useEffect } from 'react';
import FloatingButton from './FloatingButton';
import ChatWindow from './ChatWindow';
import TextSelectionPopup from './TextSelectionPopup';
import './Chatbot.css';

const Chatbot: React.FC = () => {
  const [isChatOpen, setIsChatOpen] = useState(false);
  const [selectedTextQuery, setSelectedTextQuery] = useState<string | null>(null);

  // Listen for custom event to open chatbot from anywhere
  useEffect(() => {
    const handleOpenChatbot = () => {
      setIsChatOpen(true);
    };

    window.addEventListener('open-chatbot', handleOpenChatbot);
    return () => {
      window.removeEventListener('open-chatbot', handleOpenChatbot);
    };
  }, []);

  const toggleChat = () => {
    setIsChatOpen(!isChatOpen);
  };

  const closeChat = () => {
    setIsChatOpen(false);
  };

  const handleAskAboutSelection = (selectedText: string) => {
    // Open chat and pass the selected text as initial query
    setSelectedTextQuery(`Explain this: "${selectedText}"`);
    setIsChatOpen(true);
  };

  const clearSelectedTextQuery = () => {
    setSelectedTextQuery(null);
  };

  return (
    <div className={`chatbot-container ${isChatOpen ? 'chat-open' : ''}`}>
      <TextSelectionPopup onAskAboutSelection={handleAskAboutSelection} />
      <FloatingButton onClick={toggleChat} isOpen={isChatOpen} />
      <ChatWindow
        isOpen={isChatOpen}
        onClose={closeChat}
        initialQuery={selectedTextQuery}
        onQueryProcessed={clearSelectedTextQuery}
      />
    </div>
  );
};

export default Chatbot;
