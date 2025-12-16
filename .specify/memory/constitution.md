<!--
Version change: 0.1.6 -> 0.1.7
PATCH: Added Authentication, Personalization & Translation Bonus Feature Governance

Added sections:
- Core Principle V: Authentication-First Access Control
- Key Standard VII: Authentication & User Profile Standard
- Key Standard VIII: Translation Feature Standard
- Success Criteria V: Bonus Feature Eligibility

Removed sections:
- None

Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending (review for alignment)
- .specify/templates/spec-template.md: ⚠ pending (review for alignment)
- .specify/templates/tasks-template.md: ⚠ pending (review for alignment)
- .specify/templates/commands/*.md: ⚠ pending (review for alignment)

Follow-up TODOs:
- Ensure Better-Auth integration aligns with authentication standard
- Verify translate_to_urdu skill exists and is functional
- Update feature specs to include bonus feature requirements
-->
# Project Constitution: AI/Spec-Driven Book with RAG Chatbot

**Version**: 0.1.7
**Previous Version**: 0.1.6
**Ratified**: 2025-12-09
**Last Amended**: 2025-12-15

---

## Purpose

This constitution defines the governing principles, standards, constraints, and success criteria for building a **Spec-Driven AI-powered technical textbook** with:

- A Retrieval-Augmented Generation (RAG) chatbot
- Secure user authentication
- Personalized learning paths
- Optional Urdu translation features

The goal is to ensure **educational quality**, **technical accuracy**, **security**, and **hackathon compliance**.

---

## Core Principles

### I. Accuracy and Verifiability
All textbook content, chatbot responses, personalized explanations, and translated output MUST be factually accurate and verifiable. Fabricated APIs, tools, or claims are strictly prohibited.

---

### II. Clarity and Educational Value
Content MUST be written for learners with foundational CS/AI knowledge.
Personalization and translation MUST enhance understanding without changing technical meaning.

---

### III. Consistency and Uniformity
The system MUST maintain consistent terminology, tone, and structure across:
- Original English content
- Personalized content
- Urdu translations
- RAG chatbot responses

---

### IV. Reproducibility
All code, infrastructure, indexing pipelines, authentication flows, and deployment steps MUST be reproducible.

---

### V. Authentication-First Access Control
All **bonus features** MUST require authentication.

- Anonymous users MAY read static textbook content
- Anonymous users MUST NOT access:
  - Translation features
  - Personalization
  - User-specific analytics
  - Bonus-point functionality

---

## Key Standards

### I. Docusaurus-Compatible Markdown
All textbook content MUST be written in GitHub-flavored Markdown compatible with Docusaurus.

---

### II. Structured Chapters
Each chapter MUST include:
- Introduction
- Core concepts
- Examples
- Optional exercises
- Summary

---

### III. Code Block Formatting
All code examples MUST use fenced blocks with proper language tags.

---

### IV. Diagram Inclusion
Mermaid or ASCII diagrams SHOULD be used where helpful.

---

### V. RAG Chatbot Integration
The chatbot MUST:
- Use FastAPI backend
- Use Qdrant for vector storage
- Follow Spec-Kit Plus and OpenAI Agents SDK patterns
- Respond strictly from textbook content only

---

### VI. Personalization Standard
Personalization MUST be based on **user background data collected at signup**, including:
- Software background
- Hardware background
- Experience level

Personalization MAY adjust:
- Explanation depth
- Examples
- Learning guidance

---

### VII. Authentication & User Profile Standard
The system MUST implement:
- Signup and Signin using **Better-Auth**
- Mandatory background questions during signup
- Secure session handling
- Access control enforcement for all bonus features

---

### VIII. Translation Feature Standard
When implemented:

- Translation MUST be user-triggered via a button at the start of each chapter
- Translation MUST be available **only to logged-in users**
- Translation MUST use the existing **`translate_to_urdu` skill**
- Technical terms and code blocks MUST NOT be altered
- Original English content MUST remain accessible

---

## Constraints

### I. Original Content Mandate
All textbook content MUST be original. External sources may only be used for reference and MUST be rephrased.

---

### II. No Hallucinations
The RAG system MUST NOT generate responses outside textbook scope.
Fallback responses are REQUIRED when content is unavailable.

---

### III. Content Scope Validation
The system MUST verify that a topic exists in the textbook before answering.

---

### IV. Docusaurus Deployment Readiness
The project MUST build and deploy cleanly using Docusaurus without manual fixes.

---

### V. API and Service Compliance
All integrations (Gemini, OpenAI, Qdrant, Neon, Better-Auth) MUST comply with their respective terms of service.

---

## Success Criteria

### I. Compilation Success
The textbook MUST compile successfully in Docusaurus.

---

### II. Educational Effectiveness
Content MUST meet clarity and learning effectiveness goals.

---

### III. RAG Chatbot Reliability
The chatbot MUST:
- Provide accurate answers
- Cite textbook sections
- Handle edge cases gracefully

---

### IV. Deployment Success
The system MUST be deployable without runtime or build errors.

---

### V. Bonus Feature Eligibility
To qualify for hackathon bonus points:

- Signup & Signin using Better-Auth MUST function correctly
- Background questions MUST be mandatory at signup
- Personalization MUST be observable post-login
- Urdu translation MUST:
  - Be user-triggered
  - Use `translate_to_urdu` skill
  - Require authentication

---

## Part-2: RAG Chatbot for Physical AI Textbook

All Part-2 principles, standards, constraints, and success criteria from **v0.1.6** remain valid and unchanged.

### Part-2 Core Principles

#### I. Gemini LLM Integration
The RAG chatbot MUST utilize the OpenAI Agents SDK with a Gemini LLM provider via OpenAI-compatible endpoint. Implementation MUST follow the pattern: `AsyncOpenAI` base provider with `OpenAIChatCompletionsModel` wrapper on `gemini-2.5-flash`.

#### II. Student-Centric Design
The chatbot MUST prioritize student learning outcomes by providing immediate, contextual responses that enhance understanding of Physical AI and Humanoid Robotics concepts. All responses MUST be tailored to the educational context.

#### III. Content Fidelity
All chatbot responses MUST be strictly grounded in the Physical AI & Humanoid Robotics textbook content. The system MUST NOT generate responses that contradict or extend beyond the textbook's scope.

#### IV. Performance Reliability
The system MUST maintain 95% of queries responding within 5 seconds under normal load conditions and 99.9% availability measured monthly. Performance metrics MUST be continuously monitored.

### Part-2 Key Standards

#### I. Folder Structure Requirements
The project MUST implement the following folder structure:
```
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

#### II. User Story Implementation Standards
All user stories MUST follow the Given-When-Then acceptance test pattern:

**User Story 1 - Student Asks Questions (Priority: P1)**
- Given: A student is reading a textbook chapter
- When: They ask a question about a concept in the chatbot
- Then: They receive a relevant answer with citations to the appropriate textbook sections

**User Story 2 - Student Navigation Guidance (Priority: P2)**
- Given: A student wants to explore related content
- When: They ask for guidance on various topics
- Then: The system provides links to relevant chapters and sections with brief explanations

**User Story 3 - Contextual Explanations (Priority: P3)**
- Given: A student requests explanations in different formats
- When: They submit a request for simplified explanations or examples
- Then: The system provides accurate responses based on textbook content

**User Story 4 - Educator Analytics (Priority: P4)**
- Given: Students have interacted with the chatbot
- When: An educator accesses analytics
- Then: They see reports on common questions and frequently accessed content

#### III. Edge Cases and Clarifications
The system MUST handle these edge cases:
- Questions that could be answered from multiple chapters with conflicting information
- Ambiguous queries that could refer to different concepts
- Queries when no relevant content exists in the textbook
- Inappropriate or off-topic questions from students
- Vector database unavailability during chat sessions
- Extremely long or complex questions exceeding token limits

#### IV. Gemini LLM Integration Standards
The implementation MUST follow this pattern:
```python
import os
from agents import AsyncOpenAI, OpenAIChatCompletionsModel
from dotenv import load_dotenv
from agents.run import RunConfig

load_dotenv()

gemini_api_key = os.getenv("GEMINI_API_KEY")

external_provider = AsyncOpenAI(
    api_key=gemini_api_key,
    base_url="https://generativelanguage.googleapis.com/v1beta/openai"
)

model = OpenAIChatCompletionsModel(
    openai_client=external_provider,
    model="gemini-2.5-flash",
)

config = RunConfig(
    model=model,
    model_provider=external_provider,
    tracing_disabled=True
)
```

### Part-2 Constraints

#### I. Content Processing Requirements
- **FR-076**: System MUST chunk textbook content into 512-token segments for embedding generation
- Content MUST be processed from Docusaurus docs directory
- Embeddings MUST use Gemini free embeddings model
- Content indexing MUST maintain referential integrity with source documents

#### II. Text-Selection Query Support
- **FR-035**: System MUST support text selection queries where users can select text and ask about it
- Selected text queries MUST be processed with the same content validation as regular queries
- Citations for selected text queries MUST reference the original source location

#### III. SSE Streaming Requirements
- **FR-008**: System MUST provide real-time chat responses using Server-Sent Events (SSE) streaming
- Streaming responses MUST include proper citation tracking during generation
- Streaming MUST maintain connection stability during long-running responses
- Error handling during streaming MUST provide graceful fallbacks

#### IV. Topic-Specific Functional Requirements (FR-046 to FR-070)
- **FR-046**: Handle questions about mathematical concepts and formulas from the textbook (only if content exists in textbook)
- **FR-047**: Provide context-aware responses based on the current chapter being read (only if content exists in textbook)
- **FR-048**: Handle questions about prerequisites and dependencies between concepts (only if content exists in textbook)
- **FR-049**: Provide examples and analogies based on the textbook content (only if content exists in textbook)
- **FR-050**: Support questions about practical applications of theoretical concepts (only if content exists in textbook)
- **FR-051**: Ensure responses are generated from accurate textbook content only
- **FR-052**: Provide a way to distinguish between direct quotes and paraphrased content (only if content exists in textbook)
- **FR-053**: Handle questions that require synthesis of information from multiple sources (only if content exists in textbook)
- **FR-054**: Maintain appropriate response length for readability (only if content exists in textbook)
- **FR-055**: Handle questions about diagrams and visual content in the textbook (only if content exists in textbook)
- **FR-056**: Provide a way to follow up on specific parts of previous responses (only if content exists in textbook)
- **FR-057**: Handle questions about code snippets and their implementations (only if content exists in textbook)
- **FR-058**: Provide consistent terminology matching the textbook's language (only if content exists in textbook)
- **FR-059**: Handle questions about ROS 2 concepts and architecture properly (only if content exists in textbook)
- **FR-060**: Provide appropriate responses for questions about humanoid robotics (only if content exists in textbook)
- **FR-061**: Handle questions about URDF and robot descriptions accurately (only if content exists in textbook)
- **FR-062**: Provide guidance for practical exercises mentioned in the textbook (only if content exists in textbook)
- **FR-063**: Handle questions about simulation environments and tools (only if content exists in textbook)
- **FR-064**: Provide appropriate responses for questions about hardware components (only if content exists in textbook)
- **FR-065**: Handle questions about Python integration with ROS 2 properly (only if content exists in textbook)
- **FR-066**: Maintain academic integrity in all generated responses
- **FR-067**: Handle questions about Physical AI concepts accurately (only if content exists in textbook)
- **FR-068**: Provide appropriate explanations for mathematical formulations (only if content exists in textbook)
- **FR-069**: Handle questions about sensor systems and integration properly (only if content exists in textbook)
- **FR-070**: Provide appropriate responses for questions about control systems (only if content exists in textbook)

#### V. Cross-Cutting Constraints for Part-2
All functional requirements that involve generating responses, providing explanations, or answering questions are subject to the following constraints:
- Responses MUST be limited to information available within the Physical AI & Humanoid Robotics textbook (FR-010)
- The RAG system MUST NOT generate responses that contradict textbook content (Constitution: No Hallucinations principle)
- Any response that cannot be grounded in textbook content MUST return an appropriate fallback response (FR-030)

#### VI. Security & Rate Limiting Requirements (FR-071 to FR-073)
- **FR-071**: System MUST implement rate limiting of 100 requests per user per minute to prevent abuse
- **FR-072**: System MUST validate all user inputs using sanitization filters to prevent injection attacks
- **FR-073**: System MUST log all security-relevant events for audit purposes

#### VII. Failure Handling Requirements (FR-074)
- **FR-074**: System MUST return a graceful fallback message when vector database is unavailable

#### VIII. Performance Requirements (FR-075)
- **FR-075**: System MUST respond to 95% of queries within 5 seconds under normal load conditions

#### IX. Availability Requirements (FR-077)
- **FR-077**: System MUST maintain 99.9% availability measured monthly

### Part-2 Success Criteria

#### I. Student Engagement Metrics
- **SC-001**: Students can receive accurate answers to textbook-related questions within 5 seconds of submission
- **SC-002**: The system achieves at least 90% accuracy in providing relevant responses based on textbook content
- **SC-003**: At least 85% of student questions receive responses with proper citations to textbook sections
- **SC-004**: The system handles 100+ concurrent users without performance degradation
- **SC-005**: Response quality scores (measured through user feedback) average above 4.0 out of 5.0

#### II. Content Retrieval Effectiveness
- **SC-006**: The chatbot successfully retrieves relevant content for 95% of well-formed questions about textbook topics
- **SC-007**: Users can initiate conversations and receive responses with 99% system availability
- **SC-008**: The system processes and embeds all textbook content during the initial setup phase

#### III. Learning Enhancement Metrics
- **SC-009**: Student engagement metrics show increased time spent on textbook content when the chatbot is available
- **SC-010**: The system responds to 99% of valid queries without errors or crashes

---

## Agent Behavior

### I. Clarification Seeking
The AI agent MUST ask for clarification when requirements are ambiguous.

---

### II. Internal Consistency
All outputs MUST remain internally consistent.

---

### III. Simplification and Visualization
The agent SHOULD prefer clear explanations, examples, and diagrams.

---

### IV. Utility and Accuracy
Every output MUST directly support project goals.

---

### V. Documentation Alignment
All outputs MUST align with existing documentation structure and Context7 references.

---

## Governance

This constitution is the authoritative governance document.
All future specs, plans, and tasks MUST comply with it.

Amendments require documented rationale, stakeholder approval, and a clear migration plan for affected content.
