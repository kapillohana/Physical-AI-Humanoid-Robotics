import React, { useState, useEffect, useRef } from 'react';
import './Chatbot.css';

const Chatbot = () => {
  const [isOpen, setIsOpen] = useState(false);
  const [messages, setMessages] = useState([]);
  const [inputValue, setInputValue] = useState('');
  const [isLoading, setIsLoading] = useState(false);
  const [selectedText, setSelectedText] = useState('');
  const [explainButtonPos, setExplainButtonPos] = useState({ x: 0, y: 0, visible: false });
  const [isExplainButtonVisible, setIsExplainButtonVisible] = useState(false);

  const messagesEndRef = useRef(null);
  const inputRef = useRef(null);

  // Scroll to bottom of messages
  useEffect(() => {
    scrollToBottom();
  }, [messages]);

  const scrollToBottom = () => {
    messagesEndRef.current?.scrollIntoView({ behavior: 'smooth' });
  };

  // Handle global mouseup for text selection
  useEffect(() => {
    const handleGlobalMouseUp = () => {
      const selectedText = window.getSelection().toString().trim();
      if (selectedText) {
        const selection = window.getSelection();
        if (selection.rangeCount > 0) {
          const range = selection.getRangeAt(0);
          const rect = range.getBoundingClientRect();
          setExplainButtonPos({
            x: rect.right - 50, // Position button near the right of selection
            y: rect.top - 40,   // Position button above the selection
            visible: true
          });
          setIsExplainButtonVisible(true);
        }
      } else {
        setIsExplainButtonVisible(false);
      }
    };

    const handleGlobalClick = (e) => {
      // Hide explain button if clicking outside of it
      if (!e.target.closest('.explain-button')) {
        setIsExplainButtonVisible(false);
      }
    };

    document.addEventListener('mouseup', handleGlobalMouseUp);
    document.addEventListener('click', handleGlobalClick);

    return () => {
      document.removeEventListener('mouseup', handleGlobalMouseUp);
      document.removeEventListener('click', handleGlobalClick);
    };
  }, []);

  const handleExplainClick = () => {
    const selectedText = window.getSelection().toString().trim();
    if (selectedText) {
      setSelectedText(selectedText);
      setIsOpen(true);
      // Clear selection
      window.getSelection().removeAllRanges();
      setIsExplainButtonVisible(false);

      // Automatically send the selected text with the specific prompt
      setTimeout(() => {
        setInputValue(`Please explain this specific excerpt from the book: ${selectedText}`);
        // Trigger the send message after a short delay to allow the chat to open
        setTimeout(() => {
          if (selectedText) {
            sendMessage();
          }
        }, 300);
      }, 100);
    }
  };

  const toggleChat = () => {
    setIsOpen(!isOpen);
    if (!isOpen) {
      setTimeout(() => {
        if (inputRef.current) {
          inputRef.current.focus();
        }
      }, 100);
    }
  };

  const sendMessage = async () => {
    if (!inputValue.trim() && !selectedText.trim()) return;

    const messageToSend = inputValue.trim() || selectedText.trim();
    const userMessage = { text: messageToSend, sender: 'user' };

    setMessages(prev => [...prev, userMessage]);
    setInputValue('');
    setSelectedText('');
    setIsLoading(true);

    try {
      console.log('Sending request to backend:', {
        query: messageToSend,
        selected_text: selectedText || null
      });

      const BACKEND_URL = process.env.REACT_APP_BACKEND_URL || 'https://your-railway-app-name.up.railway.app';
      const response = await fetch(`${BACKEND_URL}/api/chat`, {
        method: 'POST',
        headers: {
          'Content-Type': 'application/json',
        },
        body: JSON.stringify({
          query: messageToSend,
          selected_text: selectedText || null
        }),
      });

      console.log('Response status:', response.status);
      console.log('Response ok:', response.ok);

      if (!response.ok) {
        const errorText = await response.text();
        console.error('API Error:', response.status, errorText);
        throw new Error(`HTTP error! status: ${response.status}, message: ${errorText}`);
      }

      const data = await response.json();
      console.log('API Response:', data);

      const botMessage = { text: data.response, sender: 'bot', context: data.context_used };
      setMessages(prev => [...prev, botMessage]);
    } catch (error) {
      console.error('Error sending message:', error);
      console.error('Error details:', {
        message: error.message,
        stack: error.stack,
      });

      let errorMessageText = 'Sorry, I encountered an error. Please try again.';

      // Provide more specific error messages
      if (error.message.includes('fetch')) {
        errorMessageText = 'Unable to connect to the backend server. Please check if the backend is properly deployed and the URL is configured correctly.';
      } else if (error.message.includes('404')) {
        errorMessageText = 'API endpoint not found. Please check if the backend server is properly configured.';
      } else if (error.message.includes('500')) {
        errorMessageText = 'Backend server error. Please check the server logs.';
      }

      const errorMessage = { text: errorMessageText, sender: 'bot' };
      setMessages(prev => [...prev, errorMessage]);
    } finally {
      setIsLoading(false);
    }
  };

  const handleKeyPress = (e) => {
    if (e.key === 'Enter' && !e.shiftKey) {
      e.preventDefault();
      sendMessage();
    }
  };

  return (
    <>
      {/* Explain button that appears when text is selected */}
      {isExplainButtonVisible && (
        <button
          className="explain-button"
          style={{
            position: 'fixed',
            left: `${explainButtonPos.x}px`,
            top: `${explainButtonPos.y}px`,
            zIndex: 10000,
          }}
          onClick={handleExplainClick}
        >
          Explain
        </button>
      )}

      {/* Floating Action Button (FAB) */}
      <div className="chatbot-fab-container">
        <button
          className={`chatbot-fab ${isOpen ? 'open' : ''}`}
          onClick={toggleChat}
          aria-label={isOpen ? 'Close chat' : 'Open chat'}
        >
          <div className="fab-content">
            {isOpen ? (
              <span className="close-icon">✕</span>
            ) : (
              <span className="chat-icon">💬</span>
            )}
          </div>
        </button>

        {/* Chat Window - only visible when open */}
        {isOpen && (
          <div className="chatbot-window">
            <div className="chatbot-header">
              <h3>AI Assistant</h3>
              <button
                className="close-btn"
                onClick={toggleChat}
                aria-label="Close chat"
              >
                ✕
              </button>
            </div>

            <div className="chatbot-messages">
              {messages.length === 0 ? (
                <div className="welcome-message">
                  <p>Hello! I'm your AI assistant for the Physical AI & Humanoid Robotics textbook.</p>
                  <p>You can ask me questions or select text on the page and click "Explain" to get more information.</p>
                </div>
              ) : (
                messages.map((message, index) => (
                  <div
                    key={index}
                    className={`message ${message.sender === 'user' ? 'user-message' : 'bot-message'}`}
                  >
                    <div className="message-content">
                      {message.text}
                    </div>
                  </div>
                ))
              )}
              {isLoading && (
                <div className="message bot-message">
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

            <div className="chatbot-input">
              <textarea
                ref={inputRef}
                value={inputValue}
                onChange={(e) => setInputValue(e.target.value)}
                onKeyPress={handleKeyPress}
                placeholder="Ask me anything about the textbook..."
                rows="1"
                disabled={isLoading}
              />
              <button
                onClick={sendMessage}
                disabled={!inputValue.trim() || isLoading}
                className="send-btn"
              >
                {isLoading ? 'Sending...' : 'Send'}
              </button>
            </div>
          </div>
        )}
      </div>
    </>
  );
};

export default Chatbot;