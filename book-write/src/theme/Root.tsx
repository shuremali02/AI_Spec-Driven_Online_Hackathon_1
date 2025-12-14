import React, { useEffect, useState } from 'react';

// Lazy load Chatbot component only on client-side
function ChatbotWrapper() {
  const [ChatbotComponent, setChatbotComponent] = useState<React.ComponentType | null>(null);

  useEffect(() => {
    // Dynamic import only runs in browser
    import('../components/Chatbot').then((module) => {
      setChatbotComponent(() => module.default);
    });
  }, []);

  if (!ChatbotComponent) return null;
  return <ChatbotComponent />;
}

// This component wraps the entire app and adds the chatbot globally
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      <ChatbotWrapper />
    </>
  );
}
