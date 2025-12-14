# Implementation Plan: RAG Chatbot for Physical AI Textbook

**Branch**: `001-rag-chatbot` | **Date**: 2025-12-13 | **Spec**: [spec.md](spec.md)
**Input**: Feature specification from `/specs/001-rag-chatbot/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

The RAG Chatbot for Physical AI Textbook is a web application feature that integrates an AI-powered chatbot into a Docusaurus-based textbook. The system uses Retrieval-Augmented Generation (RAG) methodology to answer student questions based on textbook content. The backend is built with FastAPI and uses OpenAI ChatKit Agents SDK, with Gemini LLM provider via OpenAI-compatible endpoint. Vector storage uses Qdrant Cloud, while conversation history is stored in Neon Serverless Postgres. The frontend chatbot UI is embedded in the Docusaurus textbook interface with a floating button and chat window components. This implementation aligns with Part-2 of the project constitution v0.1.6, specifically addressing Gemini LLM integration, student-centric design, content fidelity, and performance reliability requirements.

## Technical Context

**Language/Version**: Python 3.10+ (backend), TypeScript (frontend), JavaScript (Docusaurus)
**Primary Dependencies**: OpenAI ChatKit Agents SDK, FastAPI, Qdrant, Neon Postgres, uv (Python package manager), Docusaurus
**Storage**: Qdrant Cloud (vector database), Neon Serverless Postgres (conversation history), Docusaurus docs (content source)
**Testing**: pytest (backend), Jest/React Testing Library (frontend)
**Target Platform**: Web application (Linux server backend, browser frontend)
**Project Type**: Web application (backend + frontend)
**Performance Goals**: 95% of queries respond within 5 seconds under normal load, 99.9% system availability measured monthly, support 100+ concurrent users
**Constraints**: 1GB vector storage limit (Qdrant Free Tier), 100 requests per user per minute rate limiting, responses limited to textbook content only
**Scale/Scope**: 1000+ students, 5 textbook chapters, 512-token content chunks for embedding

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Gate 1: Accuracy and Verifiability (Part 1)
✅ PASSED: The RAG system will provide accurate answers based on textbook content and properly cite sources, aligning with the constitution's requirement for factually accurate information.

### Gate 2: Clarity and Educational Value (Part 1)
✅ PASSED: The chatbot will enhance educational value by providing contextual explanations and guided navigation, supporting the constitution's focus on clear, educational content targeting appropriate reading levels.

### Gate 3: Consistency and Uniformity (Part 1)
✅ PASSED: The chatbot responses will maintain consistency with textbook content, and the UI will match the Docusaurus design, meeting the constitution's consistency requirements.

### Gate 4: Reproducibility (Part 1)
✅ PASSED: The system uses standard technologies (FastAPI, Qdrant, Neon Postgres) with proper documentation and deployment processes, ensuring reproducibility as required by the constitution.

### Gate 5: Original Content Mandate (Part 1)
✅ PASSED: The system will only respond with information from the existing textbook content, not generating new content that would violate the original content mandate.

### Gate 6: No Hallucinations (Part 1 & 2)
✅ PASSED: The RAG methodology ensures responses are grounded in actual textbook content, with cross-cutting constraints specifically requiring responses to be limited to textbook content, preventing hallucinations as required by the constitution.

### Gate 7: Content Scope Validation (Part 1 & 2)
✅ PASSED: The system will verify that requested topics exist in the textbook before generating responses, and provide appropriate fallback messages when content is not available, meeting the constitution's content scope validation requirement.

### Gate 8: Docusaurus Deployment Readiness (Part 1)
✅ PASSED: The frontend component is designed specifically for integration with Docusaurus, meeting the deployment readiness requirement.

### Gate 9: API and Service Compliance (Part 1)
✅ PASSED: All integrations with OpenAI, Neon, and Qdrant comply with their respective terms of service as required.

### Gate 10: Gemini LLM Integration (Part 2)
✅ PASSED: The system will utilize OpenAI Agents SDK with a Gemini LLM provider via OpenAI-compatible endpoint, following the AsyncOpenAI base provider with OpenAIChatCompletionsModel wrapper on gemini-2.5-flash pattern as specified in the constitution.

### Gate 11: Student-Centric Design (Part 2)
✅ PASSED: The chatbot prioritizes student learning outcomes by providing immediate, contextual responses that enhance understanding of Physical AI and Humanoid Robotics concepts, with responses tailored to the educational context.

### Gate 12: Content Fidelity (Part 2)
✅ PASSED: All chatbot responses are strictly grounded in the Physical AI & Humanoid Robotics textbook content, with no responses that contradict or extend beyond the textbook's scope.

### Gate 13: Performance Reliability (Part 2)
✅ PASSED: The system maintains 95% of queries responding within 5 seconds under normal load conditions and 99.9% availability measured monthly, with performance metrics continuously monitored as required.

## Project Structure

### Documentation (this feature)

```text
specs/001-rag-chatbot/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
project-root/
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
├── book-write/               # Docusaurus project folder
│   ├── docs/                 # Textbook chapters already exist
│   ├── src/
│   │   └── components/
│   │       └── Chatbot/
│   │           ├── index.tsx
│   │           ├── ChatWindow.tsx
│   │           ├── FloatingButton.tsx
│   │           └── api.ts      # Frontend chat API integration
│   ├── docusaurus.config.js
│   ├── package.json
│   └── other Docusaurus files…
└── specs/
    └── 001-rag-chatbot/
        └── spec.md
```

**Structure Decision**: Web application with separate backend and frontend components. The backend uses FastAPI with agents, API routes, database models, and vector storage modules. The frontend is integrated into the existing Docusaurus textbook via React components in the src/components/Chatbot directory, following the folder structure requirements specified in Part-2 of the constitution.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |
