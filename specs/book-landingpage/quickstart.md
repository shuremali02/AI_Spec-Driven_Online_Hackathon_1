# Quickstart: Book Landing Page - Physical AI & Humanoid Robotics

## Prerequisites
- Node.js 16+ installed on your system
- Completed course setup (ROS 2 Humble, Ubuntu 22.04)
- Basic understanding of Docusaurus documentation framework
- Git repository access for the textbook project

## Setup Your Development Environment
```bash
# Navigate to the book-write directory
cd /mnt/e/AI_Spec-Driven_Online_Hackathon_1/book-write

# Install dependencies
npm install

# Start the development server
npm start
```

## Verify Landing Page Location
The landing page should be located at:
- **Source**: `book-write/docs/index.md` (this will override the default Docusaurus index)
- **URL**: `http://localhost:3000/` when running locally
- **Production**: Root URL of the deployed site

## Create the Landing Page File
```bash
# Create the landing page file
touch docs/index.md
```

## Essential Components to Include
1. **Frontmatter**: Docusaurus configuration at the top of the file
2. **ModuleCard Component**: Import and use for displaying modules
3. **Hero Section**: Compelling introduction with CTAs
4. **Content Sections**: All required sections from the spec

## Basic Landing Page Structure
```markdown
---
id: intro
title: "Physical AI & Humanoid Robotics"
sidebar_position: 0
sidebar_label: "🏠 Home"
description: "Learn Physical AI & Humanoid Robotics - from ROS 2 fundamentals to conversational AI robots"
keywords: [Physical AI, Humanoid Robotics, ROS 2, Isaac Sim, Embodied Intelligence]
slug: /
---

# Physical AI & Humanoid Robotics

## Your main content here...
```

## Import Custom Components
```markdown
import ModuleCard from '@site/src/components/ModuleCard';

<div className="module-grid">
  <ModuleCard
    icon="🧠"
    number="1"
    title="The Robotic Nervous System (ROS 2)"
    duration="Weeks 1-5"
    purpose="Master ROS 2 middleware for robust robot communication and control."
    outcomes={["Understand ROS 2 architecture (nodes, topics, services)", "Build Python-based ROS 2 applications with `rclpy`"]}
    link="/docs/module-1/"
  />
  <!-- More ModuleCards here -->
</div>
```

## Verify Component Availability
Check that the ModuleCard component exists:
```bash
ls src/components/ModuleCard.js
# or
ls src/components/ModuleCard.tsx
```

## Common Configuration Checks
- Ensure `docusaurus.config.ts` has proper sidebar configuration
- Verify that the landing page appears as the first item in navigation
- Check that all module links are correct and accessible

## Testing the Landing Page
1. Run `npm start` in the `book-write` directory
2. Navigate to `http://localhost:3000/`
3. Verify all sections appear correctly
4. Test all links and CTAs
5. Check responsive design on different screen sizes

## Deployment Readiness Checklist
- [ ] Landing page loads without errors
- [ ] All module cards display properly
- [ ] Links to modules work correctly
- [ ] Frontmatter is properly configured
- [ ] SEO elements are in place
- [ ] Page loads within 3 seconds
- [ ] Responsive design verified