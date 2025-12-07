<!-- Sync Impact Report
Version change: 1.0.0 → 1.1.0
List of modified principles:
- Technical Stack: Added
- Architecture: Added
- Code Quality & Standards: Added
- Testing: Added
- Security: Added
- Content Generation: Added
- Feature Governance: Added
Added sections:
- AI Agent Principles
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending
- .specify/templates/spec-template.md: ⚠ pending
- .specify/templates/tasks-template.md: ⚠ pending
- .specify/templates/commands/*.md: ⚠ pending
Follow-up TODOs: None
-->
# Book Writing Hackathon Constitution

## Core Principles

### I. Technical Stack
Mandatory use of Docusaurus for the book frontend, FastAPI (Python) for the backend/RAG API, Node.js for the frontend build/scripting, Neon Serverless Postgres for user/session data (for bonus features), and Qdrant Cloud Free Tier for vector storage.

### II. Architecture
The solution must follow a Decoupled Monolith or Service-Oriented Architecture (SOA) pattern, separating the Docusaurus frontend (client), the FastAPI RAG/Bonus feature backend (API), and the OpenAI Agent logic (Agent Service). Code reuse must be prioritized; the agent must not duplicate existing functionality (e.g., in auth or RAG utility functions).

### III. Code Quality & Standards
All generated code must be clean, secure, and modern.

### IV. Testing
Test-Driven Development (TDD) is mandatory for all core backend logic (FastAPI), including RAG and Authentication endpoints. Use pytest for Python unit tests.

### V. Security
Authentication/Authorization must exclusively use Better Auth standards. All API endpoints must enforce proper session validation and adhere to OWASP Top 10 guidelines for injection prevention.

### VI. Content Generation
The textbook content must be technically accurate, pedagogically sound, and strictly follow the course outline (Modules 1-4, Weeks 1-13). Content should be written at a collegiate/graduate level.

### VII. Feature Governance (Non-Negotiable)
- RAG Chatbot: Must be embedded within the Docusaurus site and only answer questions based on the book's content. It must support the selected text context-aware questioning.
- Personalization/Translation (Bonus): If implemented, these features must be triggered by a user action (button press) at the chapter start and must be protected behind the Better Auth login.

## AI Agent Principles

- **Technology Frameworks**: Docusaurus, FastAPI, OpenAI Agents/ChatKit SDKs.
- **Data Stores**: Mandatory Services Neon Serverless Postgres, Qdrant Cloud Free Tier.
- **Architecture Structure**: Decoupled services (Frontend, RAG/Auth Backend, Agent Core).
- **Quality Testing**: TDD.

## Governance

All PRs/reviews must verify compliance with this constitution. Complexity must be justified. Amendments require documentation, approval, and a migration plan.

**Version**: 1.1.0 | **Ratified**: 2025-12-07 | **Last Amended**: 2025-12-07
