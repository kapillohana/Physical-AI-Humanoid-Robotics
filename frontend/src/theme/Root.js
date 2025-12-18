import React from 'react';
import Chatbot from '../components/Chatbot';

// Default theme Root component wrapper
function Root({ children }) {
  return (
    <>
      {children}
      <Chatbot />
    </>
  );
}

export default Root;