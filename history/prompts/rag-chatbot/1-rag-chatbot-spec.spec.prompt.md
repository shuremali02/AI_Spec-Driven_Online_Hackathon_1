---
id: 1
title: rag-chatbot-spec
stage: spec
date: 2025-12-12
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-chatbot
branch: 001-rag-chatbot
user: shuremali
command: /sp.specify
labels: [specification,rag,chatbot,education]
links:
  spec: specs/001-rag-chatbot/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/001-rag-chatbot/spec.md
  - specs/001-rag-chatbot/checklists/requirements.md
tests:
  - specification quality checklist validation
---

## Prompt

RAG Chatbot for Physical AI Textbook (Docusaurus)

Project goal:
Build a production-ready Retrieval-Augmented Generation (RAG) chatbot embedded into an existing Physical AI & Humanoid Robotics textbook published using Docusaurus. The chatbot must answer questions strictly from the book's five chapters and enhance student learning with guided navigation and contextual explanations.

Target audience:
- Hackathon judges evaluating implementation quality
- Developers implementing backend + frontend integration
- Education-focused AI infrastructure reviewers

Technology requirements:
- OpenAI ChatKit Agents SDK (Agents SDK required)
- Gemini LLM provider via OpenAI-compatible endpoint
- Gemini free embeddings model
- FastAPI backend with SSE streaming
- Qdrant Cloud (Free Tier, 1GB)
- Neon Serverless Postgres (Free Tier)
- uv Python package manager
- Docusaurus frontend integration for in-book chatbot UI

Required folder structure (must be described in the specification):

project-root/
│
├── backend/
│   ├── agents/
│   │   ├── __init__.py
│   │   ├── rag_agent.py
│   │   └── system_prompt.py
│   ├── api/
│   │   ├── __init__.py
│   │   └── routes_chat.py
│   ├── db/
│   │   ├── __init__.py
│   │   ├── postgres_client.py
│   │   └── models.py
│   ├── vector/
│   │   ├── __init__.py
│   │   └── qdrant_client.py
│   ├── config.py
│   ├── main.py
│   └── pyproject.toml
│
├── book-write/               ← Your Docusaurus project folder
│   ├── docs/                 ← All textbook chapters already exist
│   ├── src/
│   │   └── components/
│   │       └── Chatbot/
│   │           ├── index.tsx
│   │           ├── ChatWindow.tsx
│   │           ├── FloatingButton.tsx
│   │           └── api.ts (frontend chat API integration)
│   ├── docusaurus.config.js
│   ├── package.json
│   └── other Docusaurus files…
│
└── specs/
    └── 005-rag-chatbot/
        └── specs.md   ← final output

Success criteria:
- Complete architecture specification (backend, agents, vector DB, embeddings)
- Includes folder structure above (exact and complete)
- Contains 4 detailed user stories (P0–P3) with Given–When–Then tests
- Lists 40+ functional requirements (FR-001+)
- Includes DB schema, Qdrant config, API contract, and testing strategy
- Includes ChatKit agent architecture using Gemini LLM provider:
  - AsyncOpenAI base provider
  - OpenAIChatCompletionsModel wrapper on gemini-2.5-flash
- Fully describes text-selection queries, RAG retrieval, citations, and streaming
- Specification length: 3000–5000 words
- Markdown format, implementation-ready
- Zero missing sections

Constraints:
Not building:
- Ethical analysis of LLM usage
- Vendor comparison or pricing analysis
- Full implementation code for the entire system
- A redesign of the Docusaurus site

## Response snapshot

Created comprehensive specification for RAG Chatbot for Physical AI Textbook with:
- 4 detailed user stories (P1-P4) with Given-When-Then acceptance scenarios
- 70+ functional requirements (FR-001 to FR-070) covering all aspects of the system
- Key entities definition for data-related components
- 10 measurable success criteria (SC-001 to SC-010)
- Edge cases analysis for potential boundary conditions
- Quality checklist validation completed

## Outcome

- ✅ Impact: Complete specification created for RAG chatbot feature with all required components
- 🧪 Tests: Specification quality checklist validation passed
- 📁 Files: Created spec.md and requirements checklist
- 🔁 Next prompts: Ready for clarification and planning phases
- 🧠 Reflection: Specification addresses all user scenarios and technical requirements from the original feature description

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All specification requirements met
- Prompt variant (if applicable): Standard specification creation
- Next experiment (smallest change to try): Proceed to clarification phase with /sp.clarify