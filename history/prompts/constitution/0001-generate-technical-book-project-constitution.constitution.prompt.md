---
id: 0001
title: Generate technical book project constitution
stage: constitution
date: 2025-12-04
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: user
command: /sp.constitution You are an expert Spec-Kit Plus system-designer.
Your task is to generate a complete, professional-quality SPEC CONSTITUTION for the following project:

PROJECT DESCRIPTION:
I am creating a full technical book using Spec-Kit Plus, Claude Code, and Docusaurus.
The book will be written, refined, structured, and reviewed using Spec-Kit Plus workflows.
The final book will be deployed on GitHub Pages using Docusaurus.

OBJECTIVES:
- Maintain a unified voice, style, and structure across the entire book.
- Ensure accuracy, clarity, and reproducibility of all explanations and examples.
- Produce high-quality educational content targeted at readers with foundational CS/AI knowledge.
- Avoid hallucinations and ensure all examples are verifiable and technically correct.

REQUIREMENTS FOR THE CONSTITUTION:
1. Core Principles
   - Accuracy, clarity, consistency, reproducibility, educational value.

2. Key Standards
   - Markdown formatting for Docusaurus.
   - Reliable explanations, no invented APIs or tools.
   - Structured chapters: intro, explanation, examples, exercises, summary.
   - Code blocks with language tags.
   - Diagrams allowed (Mermaid/ASCII).

3. Constraints
   - No fabricated citations or unverifiable claims.
   - No copying from external sources (original writing only).
   - Entire output must be Docusaurus-compatible Markdown.
   - Style must remain uniform across all chapters.
   - No hallucinations.

4. Success Criteria
   - Book compiles cleanly in Docusaurus.
   - All chapters follow the same voice and structure.
   - Technical correctness across the entire text.
   - Clear explanations suitable for educational use.

5. Behavior Rules for the AI Agent
   - Never guess; ask for clarification when uncertain.
   - Maintain internal consistency.
   - Use examples, diagrams, and simple explanations.
   - Avoid unnecessary complexity.
   - Ensure every output is useful, structured, and accurate.

6. Output Format for the Constitution
   - Cleanly structured YAML/Markdown-style constitution.
   - Headings: Purpose, Core Principles, Key Standards, Constraints, Success Criteria, Agent Behavior, Output Rules.

INSTRUCTIONS:
Write the constitution following Spec-Kit Plus best practices, showing professional clarity, structure, and enforceable rules.
Do NOT produce book content; only produce the constitution.
Make it ready for immediate placement inside `sp/constitution`.
labels: ["constitution", "project-setup", "book-writing"]
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

/sp.constitution You are an expert Spec-Kit Plus system-designer.
Your task is to generate a complete, professional-quality SPEC CONSTITUTION for the following project:

PROJECT DESCRIPTION:
I am creating a full technical book using Spec-Kit Plus, Claude Code, and Docusaurus.
The book will be written, refined, structured, and reviewed using Spec-Kit Plus workflows.
The final book will be deployed on GitHub Pages using Docusaurus.

OBJECTIVES:
- Maintain a unified voice, style, and structure across the entire book.
- Ensure accuracy, clarity, and reproducibility of all explanations and examples.
- Produce high-quality educational content targeted at readers with foundational CS/AI knowledge.
- Avoid hallucinations and ensure all examples are verifiable and technically correct.

REQUIREMENTS FOR THE CONSTITUTION:
1. Core Principles
   - Accuracy, clarity, consistency, reproducibility, educational value.

2. Key Standards
   - Markdown formatting for Docusaurus.
   - Reliable explanations, no invented APIs or tools.
   - Structured chapters: intro, explanation, examples, exercises, summary.
   - Code blocks with language tags.
   - Diagrams allowed (Mermaid/ASCII).

3. Constraints
   - No fabricated citations or unverifiable claims.
   - No copying from external sources (original writing only).
   - Entire output must be Docusaurus-compatible Markdown.
   - Style must remain uniform across all chapters.
   - No hallucinations.

4. Success Criteria
   - Book compiles cleanly in Docusaurus.
   - All chapters follow the same voice and structure.
   - Technical correctness across the entire text.
   - Clear explanations suitable for educational use.

5. Behavior Rules for the AI Agent
   - Never guess; ask for clarification when uncertain.
   - Maintain internal consistency.
   - Use examples, diagrams, and simple explanations.
   - Avoid unnecessary complexity.
   - Ensure every output is useful, structured, and accurate.

6. Output Format for the Constitution
   - Cleanly structured YAML/Markdown-style constitution.
   - Headings: Purpose, Core Principles, Key Standards, Constraints, Success Criteria, Agent Behavior, Output Rules.

INSTRUCTIONS:
Write the constitution following Spec-Kit Plus best practices, showing professional clarity, structure, and enforceable rules.
Do NOT produce book content; only produce the constitution.
Make it ready for immediate placement inside `sp/constitution`.

## Response snapshot

Generated a new project constitution at `.specify/memory/constitution.md` with version 0.1.0, outlining the principles, standards, constraints, success criteria, and agent behavior for creating a technical book using Spec-Kit Plus, Claude Code, and Docusaurus. The constitution includes sections on Accuracy, Clarity, Consistency, Reproducibility, Docusaurus-compatible Markdown, structured chapters, code formatting, and diagram inclusion. It also details constraints against hallucinations and copied content.

## Outcome

- ✅ Impact: Constitution created, project governance established.
- 🧪 Tests: N/A
- 📁 Files: Created .specify/memory/constitution.md
- 🔁 Next prompts: Next steps involve creating specs based on this constitution.
- 🧠 Reflection: Successfully established project governance for the technical book project.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): N/A
