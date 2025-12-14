<!--
Version change: 0.1.5 -> 0.1.6 (PATCH: Added Part-2 RAG Chatbot for Physical AI Textbook with Gemini LLM integration)
List of modified principles:
- Added Part-2: RAG Chatbot for Physical AI Textbook
- Added new section: Part-2 Core Principles
- Added new section: Part-2 Key Standards
- Added new section: Part-2 Constraints
- Added new section: Part-2 Success Criteria
Added sections: Part-2 (complete RAG Chatbot specification)
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending (review for alignment)
- .specify/templates/spec-template.md: ⚠ pending (review for alignment)
- .specify/templates/tasks-template.md: ⚠ pending (review for alignment)
- .specify/templates/commands/*.md: ⚠ pending (review for alignment)
Follow-up TODOs:
- Update templates to include Part-2 requirements
- Ensure all Part-2 functional requirements are implemented in artifacts
-->
# Project Constitution: AI/Spec-Driven Book with RAG Chatbot

## Purpose
This constitution outlines the core principles, standards, constraints, and success criteria for creating a comprehensive technical book with integrated RAG chatbot using Spec-Kit Plus, Claude Code, and Docusaurus. Its primary objective is to ensure the production of high-quality, accurate, and consistent educational content with advanced AI features for readers with foundational CS/AI knowledge.

## Core Principles

### I. Accuracy and Verifiability
All technical explanations, examples, and claims within the book and RAG chatbot MUST be factually accurate and verifiable. No fabricated information, APIs, or tools are permitted. The RAG system MUST provide accurate answers based on the book content and properly cite sources when applicable.

### II. Clarity and Educational Value
Content MUST be clear, concise, and designed to effectively educate readers. It MUST target a Flesch-Kincaid Grade Level of 10-12, maintain an average sentence length of no more than 20 words, and utilize active voice in at least 75% of sentences. Complex concepts MUST be broken down into understandable components, utilizing examples, diagrams, and simple language. Personalization features MUST enhance clarity for different user backgrounds.

### III. Consistency and Uniformity
A unified voice, writing style, and structural format MUST be maintained across all chapters, sections, and multi-language versions of the book. This includes consistent terminology, formatting, and presentation of code and diagrams. The RAG chatbot MUST maintain consistent responses aligned with the book content.

### IV. Reproducibility
All code examples, technical instructions, and RAG system deployment processes MUST be reproducible by the reader. This ensures that readers can follow along, validate the concepts presented, and deploy the complete system.

## Key Standards

### I. Docusaurus-Compatible Markdown
All book content MUST be authored using GitHub-flavored Markdown, compatible with Docusaurus for proper rendering and deployment.

### II. Structured Chapters
Each chapter MUST adhere to a consistent structure, including (but not limited to): an introduction, detailed explanation of concepts, practical examples, optional exercises, and a summary of key takeaways.

### III. Code Block Formatting
Code examples MUST be presented in fenced code blocks with appropriate language tags (e.g., `python`, `javascript`).

### IV. Diagram Inclusion
Diagrams (e.g., Mermaid, ASCII art) are permitted and encouraged where they enhance clarity and understanding of complex systems or flows.

### V. RAG Chatbot Integration
The RAG chatbot MUST be seamlessly integrated into the book interface, utilize OpenAI Agents/ChatKit SDKs, FastAPI, Neon Serverless Postgres, and Qdrant Cloud, and be able to answer questions based on book content and user-selected text.

### VI. Personalization Features
When implemented, personalization features MUST adapt content based on user background information collected during signup and provide appropriate learning paths for different skill levels.

### VII. Translation Features
When implemented, translation features MUST maintain accuracy and readability while preserving technical terminology and code examples appropriately for the target language.

## Constraints

### I. Original Content Mandate
All book content MUST be original writing. Copying from external sources, including documentation or articles, is strictly prohibited. Information gathered from external sources MUST be synthesized and rephrased.

### II. No Hallucinations
The AI agent MUST NOT generate or include any hallucinated content, unverifiable claims, or non-existent APIs/tools. All information MUST be grounded in verifiable facts. The RAG system MUST NOT generate responses that contradict the book content. The RAG system MUST implement content validation to ensure all responses are grounded in actual textbook content before delivery. The system MUST provide fallback responses when no relevant textbook content is found to answer a query.

### IV. Content Scope Validation
The RAG system MUST only respond to questions about topics that are actually covered in the textbook content. The system MUST verify that requested topics exist in the textbook before attempting to generate responses. If a user asks about a topic not covered in the textbook, the system MUST clearly indicate that the information is not available in the textbook and may suggest related topics that are covered.

### V. Docusaurus Deployment Readiness
The entire output, including content and structure, MUST be ready for compilation and deployment via Docusaurus on GitHub Pages without requiring significant manual intervention.

### VI. API and Service Compliance
All integrations with OpenAI, Neon, Qdrant, and Better Auth MUST comply with their respective terms of service and usage policies.

## Success Criteria

### I. Successful Docusaurus Compilation
The complete book project MUST compile cleanly within the Docusaurus framework, producing a functional website without errors or warnings.

### II. Structural and Stylistic Cohesion
All chapters and sections MUST consistently adhere to the defined structural and stylistic guidelines, ensuring a seamless reading experience.

### III. Technical Accuracy
The technical content throughout the book MUST be correct, up-to-date, and free from errors that would mislead or confuse readers.

### IV. Effective Educational Content
The explanations MUST be clear, engaging, and effectively convey complex technical topics to the target audience with foundational CS/AI knowledge.

### V. RAG Chatbot Functionality
The integrated RAG chatbot MUST successfully answer user questions about the book content, support text selection-based queries, and provide accurate, contextually relevant responses.

### VI. Deployment Success
The complete system (book + RAG chatbot) MUST be deployable to GitHub Pages with all functionality intact.

## Part-2: RAG Chatbot for Physical AI Textbook

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

## Agent Behavior

### I. Clarification Seeking
The AI agent MUST proactively ask for clarification when encountering ambiguity, uncertainty, or missing information in the requirements or content generation process. Never guess.

### II. Internal Consistency
The AI agent MUST ensure internal consistency across all generated content, adhering to established terminology, style guides, and technical facts.

### III. Simplification and Visualization
The AI agent MUST prioritize clarity through the use of simple explanations, relevant examples, and appropriate diagrams to illustrate concepts. Avoid unnecessary complexity.

### IV. Utility and Accuracy
Every output from the AI agent MUST be useful, well-structured, accurate, and directly contribute to the project's objectives.

### V. Documentation Structure Adherence
For all documentation, book-writing, or Docusaurus-related tasks, the AI agent MUST always read and follow the existing documentation structure before producing new content. The agent MUST always connect to the Context7 MCP server first, retrieve the current documentation from the project, and ensure all new or edited content strictly aligns with the existing structure, style, and organization found in Context7.

## Output Rules
The project constitution MUST be presented in a cleanly structured Markdown format, adhering to the specified headings: Purpose, Core Principles, Key Standards, Constraints, Success Criteria, Part-2, Agent Behavior, and Output Rules.

## Governance
This constitution serves as the foundational governance document for the AI/Spec-Driven Book with RAG Chatbot project. Amendments require documented rationale, stakeholder approval, and a clear migration plan for affected content.

**Version**: 0.1.6 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-13