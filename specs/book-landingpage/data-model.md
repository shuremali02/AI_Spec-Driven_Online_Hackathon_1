# Data Model: Book Landing Page - Physical AI & Humanoid Robotics

## Content Structure

### Landing Page Metadata
- **ID**: book-landing
- **Title**: "Physical AI & Humanoid Robotics"
- **Position**: 0 (first page in documentation)
- **Description**: "Learn Physical AI & Humanoid Robotics - from ROS 2 fundamentals to conversational AI robots"
- **Keywords**: [Physical AI, Humanoid Robotics, ROS 2, Gazebo, NVIDIA Isaac, VLA]
- **Slug**: "/"
- **Target Audience**: Students, educators, and professionals in robotics and AI

### Content Entities

#### 1. Hero Section (200-250 words)
- **Fields**: headline, subheading, primary_cta, secondary_cta
- **Relationships**: Sets the tone for the entire page
- **Validation**: Must include clear value proposition

#### 2. Course Overview Section (300-400 words)
- **Fields**: physical_ai_explanation, course_structure, key_technologies, real_world_applications
- **Relationships**: Provides context for the entire course
- **Validation**: Must clearly explain what Physical AI is

#### 3. Module Breakdown Section (300-400 words)
- **Fields**: modules (array of module objects with number, title, duration, focus, outcomes)
- **Module Object**:
  - number: integer (1-4)
  - title: string
  - duration: string (e.g., "Weeks 1-5")
  - focus: string (description of module focus)
  - outcomes: array of strings (learning outcomes)
  - link: string (relative path to module)
- **Relationships**: Core content that defines the course structure
- **Validation**: All 4 modules must be represented

#### 4. Learning Outcomes Section (200-250 words)
- **Fields**: outcomes (array of outcome objects)
- **Outcome Object**:
  - number: integer (1-7)
  - text: string (the learning outcome)
- **Relationships**: Defines what students will achieve
- **Validation**: Must align with course specification

#### 5. Technical Requirements Section (150-200 words)
- **Fields**: workstation, os, software, edge_kit, budget_options
- **Relationships**: Helps students prepare for the course
- **Validation**: Must match hardware requirements from course spec

#### 6. Call-to-Action Section (100-150 words)
- **Fields**: primary_cta, secondary_options, support_links
- **Relationships**: Drives user engagement and next steps
- **Validation**: Must include clear pathways to start learning

### Component Models

#### ModuleCard Component
- **Props**:
  - icon: string (emoji or icon)
  - number: integer
  - title: string
  - duration: string
  - purpose: string
  - outcomes: array of strings
  - link: string
- **Function**: Displays module information in a card format
- **Validation**: Must be responsive and accessible

#### Hero Button Component
- **Props**:
  - text: string
  - link: string
  - type: string (primary, secondary, info)
- **Function**: Provides clear navigation options
- **Validation**: Must be visually distinct and accessible

## Navigation Structure

### Internal Links
- **Module Links**: Links to each module overview page
- **Chapter Links**: Indirect access through module pages
- **Getting Started**: Link to setup and prerequisites
- **Syllabus**: Link to detailed curriculum

### External Links
- **GitHub Repository**: Link to course materials
- **Community**: Link to discussion forums
- **Contact**: Link to support resources

## Frontmatter Schema
- **id**: string (unique identifier)
- **title**: string (page title)
- **sidebar_position**: integer (0 for landing page)
- **sidebar_label**: string ("🏠 Home")
- **description**: string (SEO meta description)
- **keywords**: array of strings (SEO keywords)
- **slug**: string ("/" for root)

## Validation Rules
- All links must be valid and accessible
- All modules from the course spec must be represented
- Technical requirements must match the course specification
- Page must load within 3 seconds
- Must be responsive on all device sizes
- Must pass accessibility standards (WCAG 2.1 AA)