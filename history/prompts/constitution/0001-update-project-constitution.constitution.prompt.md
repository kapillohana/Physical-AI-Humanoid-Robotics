---
id: 0001
title: Update Project Constitution
stage: constitution
date: 2025-12-07
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: PMLS
command: /sp.constitution
labels: ["constitution", "project-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - .specify/memory/constitution.md
tests:

---

## Prompt

Create project governing principles focusing on:Technical Stack: Mandatory use of Docusaurus for the book frontend, FastAPI (Python) for the backend/RAG API, Node.js for the frontend build/scripting, Neon Serverless Postgres for user/session data (for bonus features), and Qdrant Cloud Free Tier for vector storage.Architecture: The solution must follow a Decoupled Monolith or Service-Oriented Architecture (SOA) pattern, separating the Docusaurus frontend (client), the FastAPI RAG/Bonus feature backend (API), and the OpenAI Agent logic (Agent Service). Code reuse must be prioritized; the agent must not duplicate existing functionality (e.g., in auth or RAG utility functions).Code Quality & Standards: All generated code must be clean, secure, and modern.Testing: Test-Driven Development (TDD) is mandatory for all core backend logic (FastAPI), including RAG and Authentication endpoints. Use pytest for Python unit tests.Security: Authentication/Authorization must exclusively use Better Auth standards. All API endpoints must enforce proper session validation and adhere to OWASP Top 10 guidelines for injection prevention.Content Generation: The textbook content must be technically accurate, pedagogically sound, and strictly follow the course outline (Modules 1-4, Weeks 1-13). Content should be written at a collegiate/graduate level.Feature Governance (Non-Negotiable):RAG Chatbot: Must be embedded within the Docusaurus site and only answer questions based on the book's content. It must support the selected text context-aware questioning.Personalization/Translation (Bonus): If implemented, these features must be triggered by a user action (button press) at the chapter start and must be protected behind the Better Auth login.Key Principles for the AI AgentCategoryPrincipleMandateTechnologyFrameworksDocusaurus, FastAPI, OpenAI Agents/ChatKit SDKs.Data StoresMandatory ServicesNeon Serverless Postgres, Qdrant Cloud Free Tier.ArchitectureStructureDecoupled services (Frontend, RAG/Auth Backend, Agent Core).QualityTestingTDD usi

## Response snapshot

Updated the project constitution with new principles covering technical stack, architecture, quality, testing, security, content generation, and feature governance. Version bumped to 1.1.0. Flagged related templates for manual follow-up.

## Outcome

- ✅ Impact: Constitution updated.
- 🧪 Tests: N/A
- 📁 Files: 1 file modified: .specify/memory/constitution.md
- 🔁 Next prompts: Manual updates for dependent templates: plan-template.md, spec-template.md, tasks-template.md, commands/*.md
- 🧠 Reflection: PHR creation successful after path correction.

## Evaluation notes (flywheel)

- Failure modes observed: Unix-style path returned by Bash script on Windows.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Ensure Bash scripts return platform-agnostic paths or handle path conversion internally.
