# AI Spec-Driven Online Hackathon Project

This project is built around Spec-Driven Development (SDD) principles, leveraging AI agents to guide development, generate documentation, and capture knowledge throughout its lifecycle.

## Project Structure

-   **`CLAUDE.md`**:
    -   Outlines the operational rules and guidelines for the Claude Code AI assistant, covering SDD, task context, core guarantees (PHRs, ADRs), development guidelines, default policies, architect guidelines, and code standards.

-   **`book_write/`**:
    -   A Docusaurus project dedicated to generating comprehensive project documentation.
    -   **Key Contents**:
        -   `blog/`: Contains blog posts.
        -   `docs/`: Primary documentation content.
        -   `docusaurus.config.ts`: Main Docusaurus configuration.
        -   `package.json`: Project metadata and dependencies for Docusaurus.
        -   `sidebars.ts`: Configures documentation navigation.
        -   `src/`: Custom Docusaurus components/pages.
        -   `static/`: Static assets.

-   **`history/`**:
    -   Dedicated to systematic knowledge capture.
    -   **Subdirectories**:
        -   `prompts/`: Stores Prompt History Records (PHRs) of AI assistant interactions (organized into `constitution/`, feature-specific, and `general/`).
        -   `adr/`: Contains Architectural Decision Records (ADRs).

-   **`.specify/`**:
    -   Contains SpecKit Plus templates and scripts for the SDD workflow.
    -   **Key Contents**:
        -   `memory/constitution.md`: Defines project's core principles and code standards.
        -   `scripts/bash/create-phr.sh`: Bash script for PHR automation.
        -   `templates/phr-template.prompt.md`: Template for new PHRs.

-   **`.claude/agents/`**:
    -   Contains configurations or definitions for various Claude AI agents.
    -   **Key Files**:
        -   `content-architect.md`: Defines the Content Architect AI agent.
        -   `tech-writer.md`: Defines the Technical Writer AI agent.

## Key Entry Points and Configuration Files

-   `CLAUDE.md` (Primary AI instructions and project rules)
-   `book_write/docusaurus.config.ts` (Docusaurus site configuration)
-   `book_write/package.json` (Docusaurus project dependencies)
-   `.specify/memory/constitution.md` (Project's core principles)
