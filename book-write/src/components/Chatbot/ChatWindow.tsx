import React, { useState, useEffect, useRef } from 'react';
import { Conversation, Message, Citation, chatbotApi } from './api';
import { useAuth } from '@site/src/components/Auth/AuthProvider';
import './ChatWindow.css';

/**
 * Generates a personalized greeting based on time and user name
 */
const getPersonalizedGreeting = (userName?: string | null): string => {
  const hour = new Date().getHours();
  let prefix: string;

  if (hour >= 5 && hour < 12) {
    prefix = 'Good morning';
  } else if (hour >= 12 && hour < 18) {
    prefix = 'Good afternoon';
  } else {
    prefix = 'Good evening';
  }

  if (userName) {
    // Extract first name if full name provided
    const firstName = userName.split(' ')[0];
    return `${prefix}, ${firstName}! How can I help you today?`;
  }
  return `${prefix}! How can I help you today?`;
};

// Generate or get session ID for anonymous users
const getSessionId = (): string => {
  let sessionId = localStorage.getItem('rag_chat_session');
  if (!sessionId) {
    sessionId = crypto.randomUUID();
    localStorage.setItem('rag_chat_session', sessionId);
  }
  return sessionId;
};

interface ChatWindowProps {
  isOpen: boolean;
  onClose: () => void;
  initialConversation?: Conversation;
  initialQuery?: string | null;
  onQueryProcessed?: () => void;
}

interface ChatMessage {
  id: string;
  content: string;
  sender: 'user' | 'system';
  timestamp: Date;
  citations?: Citation[];
}

const ChatWindow: React.FC<ChatWindowProps> = ({
  isOpen,
  onClose,
  initialConversation,
  initialQuery,
  onQueryProcessed
}) => {
  const { user } = useAuth();
  const [messages, setMessages] = useState<ChatMessage[]>([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [currentConversation, setCurrentConversation] = useState<Conversation | null>(
    initialConversation || null
  );
  const messagesEndRef = useRef<null | HTMLDivElement>(null);
  const hasProcessedQuery = useRef(false);

  // Auto-scroll to bottom when messages change
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  // Handle initial query from text selection
  useEffect(() => {
    if (initialQuery && isOpen && !hasProcessedQuery.current && !isLoading) {
      hasProcessedQuery.current = true;
      setInputValue(initialQuery);
      // Auto-send the query after a short delay
      setTimeout(() => {
        sendMessageWithContent(initialQuery);
        if (onQueryProcessed) {
          onQueryProcessed();
        }
      }, 100);
    }

    // Reset when chat closes
    if (!isOpen) {
      hasProcessedQuery.current = false;
    }
  }, [initialQuery, isOpen]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  const sendMessageWithContent = async (content: string) => {
    if (!content.trim() || isLoading) return;

    // Add user message to UI immediately
    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      content: content,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setIsLoading(true);

    try {
      let conversationId = currentConversation?.conversation_id;

      // If no conversation exists, create one
      if (!conversationId) {
        const newConversation = await chatbotApi.createConversation({
          session_id: getSessionId(),
          initial_query: content,
        });
        setCurrentConversation(newConversation);
        conversationId = newConversation.conversation_id;

        // Fetch the conversation to get the response
        const conversationData = await chatbotApi.getConversation(conversationId);

        // Find the system response message
        if (conversationData.messages && conversationData.messages.length > 1) {
          const systemResponse = conversationData.messages.find(
            (msg: any) => msg.sender_type === 'system'
          );
          if (systemResponse) {
            const systemMessage: ChatMessage = {
              id: systemResponse.message_id,
              content: systemResponse.content,
              sender: 'system',
              timestamp: new Date(systemResponse.created_at),
              citations: systemResponse.citations || [],
            };
            setMessages(prev => [...prev, systemMessage]);
          }
        }
      } else {
        // Send message to existing conversation
        const response = await chatbotApi.sendMessage(conversationId, {
          content: content,
          message_type: 'text-selection',
        });

        const systemMessage: ChatMessage = {
          id: response.message_id,
          content: response.content,
          sender: 'system',
          timestamp: new Date(response.created_at),
          citations: response.citations || [],
        };
        setMessages(prev => [...prev, systemMessage]);
      }
    } catch (error) {
      console.error('Error sending message:', error);

      const errorMessage: ChatMessage = {
        id: (Date.now() + 1).toString(),
        content: 'Sorry, there was an error processing your message. Please try again.',
        sender: 'system',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleSendMessage = async () => {
    if (!inputValue.trim() || isLoading) return;

    // Add user message to UI immediately
    const userMessage: ChatMessage = {
      id: Date.now().toString(),
      content: inputValue,
      sender: 'user',
      timestamp: new Date(),
    };

    setMessages(prev => [...prev, userMessage]);
    const messageToSend = inputValue;
    setInputValue('');
    setIsLoading(true);

    try {
      let conversationId = currentConversation?.conversation_id;

      // If no conversation exists, create one
      if (!conversationId) {
        const newConversation = await chatbotApi.createConversation({
          session_id: getSessionId(),
          initial_query: messageToSend,
        });
        setCurrentConversation(newConversation);
        conversationId = newConversation.conversation_id;

        // If initial_query was sent, the response is already generated
        // We need to fetch the conversation to get the response
        const conversationData = await chatbotApi.getConversation(conversationId);

        // Find the system response message
        if (conversationData.messages && conversationData.messages.length > 1) {
          const systemResponse = conversationData.messages.find(
            (msg: any) => msg.sender_type === 'system'
          );
          if (systemResponse) {
            const systemMessage: ChatMessage = {
              id: systemResponse.message_id,
              content: systemResponse.content,
              sender: 'system',
              timestamp: new Date(systemResponse.created_at),
              citations: systemResponse.citations || [],
            };
            setMessages(prev => [...prev, systemMessage]);
          }
        }
      } else {
        // Send message to existing conversation
        const response = await chatbotApi.sendMessage(conversationId, {
          content: messageToSend,
          message_type: 'query',
        });

        const systemMessage: ChatMessage = {
          id: response.message_id,
          content: response.content,
          sender: 'system',
          timestamp: new Date(response.created_at),
          citations: response.citations || [],
        };
        setMessages(prev => [...prev, systemMessage]);
      }
    } catch (error) {
      console.error('Error sending message:', error);

      // Add error message
      const errorMessage: ChatMessage = {
        id: (Date.now() + 1).toString(),
        content: 'Sorry, there was an error processing your message. Please try again.',
        sender: 'system',
        timestamp: new Date(),
      };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e: React.KeyboardEvent) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      handleSendMessage();
    }
  };

  if (!isOpen) return null;

  return (
    <div className="chat-window-overlay">
      <div className="chat-window">
        <div className="chat-header">
          <h3>Textbook Assistant</h3>
          <button className="chat-close-button" onClick={onClose}>
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M18 6L6 18M6 6L18 18" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          </button>
        </div>

        <div className="chat-messages">
          {messages.length === 0 ? (
            <div className="chat-welcome-message">
              <p className="greeting">{getPersonalizedGreeting(user?.name)}</p>
              <p>I'm your textbook assistant. Ask me anything about the Physical AI & Humanoid Robotics content, and I'll provide answers based on the textbook with citations.</p>
            </div>
          ) : (
            messages.map((message) => (
              <div
                key={message.id}
                className={`chat-message ${message.sender === 'user' ? 'user-message' : 'system-message'}`}
              >
                <div className="message-content">
                  {message.content}
                </div>
                {message.citations && message.citations.length > 0 && (
                  <div className="message-citations">
                    <strong>Citations:</strong>
                    <ul>
                      {message.citations.map((citation, index) => (
                        <li key={index}>
                          <a
                            href={citation.url_path}
                            target="_blank"
                            rel="noopener noreferrer"
                            onClick={(e) => e.stopPropagation()}
                          >
                            {citation.chapter_title} - {citation.section_title}
                          </a>
                        </li>
                      ))}
                    </ul>
                  </div>
                )}
                <div className="message-timestamp">
                  {message.timestamp.toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })}
                </div>
              </div>
            ))
          )}
          {isLoading && (
            <div className="chat-message system-message">
              <div className="message-content">
                <div className="typing-indicator">
                  <span></span>
                  <span></span>
                  <span></span>
                </div>
              </div>
            </div>
          )}
          <div ref={messagesEndRef} />
        </div>

        <div className="chat-input-area">
          <textarea
            value={inputValue}
            onChange={(e) => setInputValue(e.target.value)}
            onKeyPress={handleKeyPress}
            placeholder="Ask a question about the textbook..."
            disabled={isLoading}
            rows={2}
          />
          <button
            onClick={handleSendMessage}
            disabled={!inputValue.trim() || isLoading}
            className="send-button"
          >
            <svg width="20" height="20" viewBox="0 0 24 24" fill="none" xmlns="http://www.w3.org/2000/svg">
              <path d="M22 2L11 13M22 2L15 22L11 13M11 13L2 9L22 2" stroke="currentColor" strokeWidth="2" strokeLinecap="round" strokeLinejoin="round"/>
            </svg>
          </button>
        </div>
      </div>
    </div>
  );
};

export default ChatWindow;