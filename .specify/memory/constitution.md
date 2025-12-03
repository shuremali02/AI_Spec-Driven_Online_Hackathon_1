<!--
Version change: 0.1.2 -> 0.1.3 (PATCH: Confirmed documentation structure adherence rule and Context7 integration)
List of modified principles:
- V. Documentation Structure Adherence: Confirmed existing agent behavior rule for Context7.
Added sections: None
Removed sections: None
Templates requiring updates:
- .specify/templates/plan-template.md: ⚠ pending (review for alignment)
- .specify/templates/spec-template.md: ⚠ pending (review for alignment)
- .specify/templates/tasks-template.md: ⚠ pending (review for alignment)
- .specify/templates/commands/*.md: ⚠ pending (review for alignment)
Follow-up TODOs: None
-->
# Project Constitution: Technical Book Creation

## Purpose
This constitution outlines the core principles, standards, constraints, and success criteria for creating a comprehensive technical book using Spec-Kit Plus, Claude Code, and Docusaurus. Its primary objective is to ensure the production of high-quality, accurate, and consistent educational content for readers with foundational CS/AI knowledge.

## Core Principles
### I. Accuracy and Verifiability
All technical explanations, examples, and claims within the book MUST be factually accurate and verifiable. No fabricated information, APIs, or tools are permitted.

### II. Clarity and Educational Value
Content MUST be clear, concise, and designed to effectively educate readers. It MUST target a Flesch-Kincaid Grade Level of 10-12, maintain an average sentence length of no more than 20 words, and utilize active voice in at least 75% of sentences. Complex concepts MUST be broken down into understandable components, utilizing examples, diagrams, and simple language.

### III. Consistency and Uniformity
A unified voice, writing style, and structural format MUST be maintained across all chapters and sections of the book. This includes consistent terminology, formatting, and presentation of code and diagrams.

### IV. Reproducibility
All code examples and technical instructions MUST be reproducible by the reader. This ensures that readers can follow along and validate the concepts presented.

## Key Standards
### I. Docusaurus-Compatible Markdown
All book content MUST be authored using GitHub-flavored Markdown, compatible with Docusaurus for proper rendering and deployment.

### II. Structured Chapters
Each chapter MUST adhere to a consistent structure, including (but not limited to): an introduction, detailed explanation of concepts, practical examples, optional exercises, and a summary of key takeaways.

### III. Code Block Formatting
Code examples MUST be presented in fenced code blocks with appropriate language tags (e.g., ````python`, ````javascript````).

### IV. Diagram Inclusion
Diagrams (e.g., Mermaid, ASCII art) are permitted and encouraged where they enhance clarity and understanding of complex systems or flows.

## Constraints
### I. Original Content Mandate
All book content MUST be original writing. Copying from external sources, including documentation or articles, is strictly prohibited. Information gathered from external sources MUST be synthesized and rephrased.

### II. No Hallucinations
The AI agent MUST NOT generate or include any hallucinated content, unverifiable claims, or non-existent APIs/tools. All information MUST be grounded in verifiable facts.

### III. Docusaurus Deployment Readiness
The entire output, including content and structure, MUST be ready for compilation and deployment via Docusaurus on GitHub Pages without requiring significant manual intervention.

## Success Criteria
### I. Successful Docusaurus Compilation
The complete book project MUST compile cleanly within the Docusaurus framework, producing a functional website without errors or warnings.

### II. Structural and Stylistic Cohesion
All chapters and sections MUST consistently adhere to the defined structural and stylistic guidelines, ensuring a seamless reading experience.

### III. Technical Accuracy
The technical content throughout the book MUST be correct, up-to-date, and free from errors that would mislead or confuse readers.

### IV. Effective Educational Content
The explanations MUST be clear, engaging, and effectively convey complex technical topics to the target audience with foundational CS/AI knowledge.

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
The project constitution MUST be presented in a cleanly structured Markdown format, adhering to the specified headings: Purpose, Core Principles, Key Standards, Constraints, Success Criteria, Agent Behavior, and Output Rules.

## Governance
This constitution serves as the foundational governance document for the technical book project. Amendments require documented rationale, stakeholder approval, and a clear migration plan for affected content.

**Version**: 0.1.3 | **Ratified**: 2025-12-04 | **Last Amended**: 2025-12-04
