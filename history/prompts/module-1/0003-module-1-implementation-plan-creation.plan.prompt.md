---
id: 0003
title: Module 1 Implementation Plan Creation
stage: plan
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20251101
feature: module-1
branch: 001-physical-ai-robotics-textbook
user: user
command: /sp.plan #CONTEXT... (truncated)
labels: ["implementation-plan", "module-1", "planning"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - specs/module-1/implementation-plan.md
tests:

---

## Prompt

#CONTEXT
You have access to:
- **Existing specs:** `specs/module-1/chapter-01-spec.md` to `chapter-04-spec.md`
- **Docusaurus:** `book-write/` folder (TypeScript config)
- **Skills:** `.claude/skills/` (diagram-generator, chapter-outliner, personalized-content, translate-to-urdu)
- **Subagents:** `.claude/agents/` (tech_writer, content_architect)
- **MCP:** context7 connected

# TASK
Create a comprehensive **implementation plan** for Module 1 that shows:
1. How to generate content from specs
2. Execution order and dependencies
3. Skills/subagents usage
4. Timeline and milestones
5. Quality checks

---

# Module 1 Implementation Plan

## Overview
- **Module:** Module 1 - The Robotic Nervous System (ROS 2)
- **Chapters:** 4
- **Total Words:** ~15,000
- **Timeline:** 5-7 days
- **Output Location:** `book-write/docs/module-1/`

---

## Phase 1: Pre-Generation Setup (Day 1)

### Step 1.1: Verify Specs Exist
```bash
# Check all specs are ready
ls -la specs/module-1/

# Expected files:
# - chapter-01-spec.md (Intro to Physical AI)
# - chapter-02-spec.md (ROS 2 Architecture)
# - chapter-03-spec.md (First ROS 2 Nodes)
# - chapter-04-spec.md (URDF Descriptions)
```

### Step 1.2: Create Output Directories
```bash
# Create chapter folders
mkdir -p book-write/docs/module-1
mkdir -p book-write/docs/module-1/labs
mkdir -p book-write/docs/module-1/examples
```

### Step 1.3: Review Specs
**For each chapter spec, verify:**
- [ ] Learning objectives clear
- [ ] Word count target defined
- [ ] Diagrams specified (3+ each)
- [ ] Code examples outlined
- [ ] Lab exercise detailed
- [ ] File paths correct

---

## Phase 2: Chapter Generation (Days 2-5)

### Order of Execution
Generate chapters **sequentially** (not parallel) because:
- Chapter 2 references Chapter 1
- Chapter 3 builds on Chapter 2
- Chapter 4 uses concepts from Chapter 3

---

### Chapter 1: Introduction to Physical AI (Day 2)

**Input:**
- Spec: `specs/module-1/chapter-01-spec.md`

**Step 2.1: Plan Chapter Structure**
```bash
claude code --skill chapter-outliner \
  --input "Read specs/module-1/chapter-01-spec.md" \
  --output "Detailed outline for Chapter 1.1"
```

**Step 2.2: Generate Content**
```bash
claude code --agent tech_writer \
  --spec specs/module-1/chapter-01-spec.md \
  --output book-write/docs/module-1/chapter-01-intro-physical-ai.md \
  --context "First chapter, conceptual introduction, no code yet"
```

**Step 2.3: Generate Diagrams (3+)**
```bash
# Diagram 1: Digital AI vs Physical AI
claude code --skill diagram-generator \
  --diagramType "system-overview" \
  --topic "Digital AI vs Physical AI comparison" \
  --annotate true

# Diagram 2: Sensorimotor Loop
claude code --skill diagram-generator \
  --diagramType "workflow" \
  --topic "Sense-Think-Act cycle in Physical AI"

# Diagram 3: Sensor Systems
claude code --skill diagram-generator \
  --diagramType "sensor-pipeline" \
  --topic "Humanoid robot sensor architecture"
```

**Step 2.4: Insert Diagrams**
```bash
# Manually insert Mermaid code into chapter markdown
# Or use automation script
```

**Step 2.5: Review Chapter 1**
- [ ] 4000 words achieved
- [ ] 3+ diagrams inserted
- [ ] Frontmatter correct
- [ ] Links work
- [ ] No code examples (conceptual chapter)
- [ ] Flows to Chapter 2

**Estimated Time:** 4-6 hours

---

### Chapter 2: ROS 2 Architecture (Day 3)

**Input:**
- Spec: `specs/module-1/chapter-02-spec.md`
- Reference: Chapter 1 (already completed)

**Step 3.1: Generate Content**
```bash
claude code --agent tech_writer \
  --spec specs/module-1/chapter-02-spec.md \
  --output book-write/docs/module-1/chapter-02-ros2-architecture.md \
  --context "Technical chapter, introduce ROS 2 concepts, some Python examples"
```

**Step 3.2: Generate Diagrams (3+)**
```bash
# Diagram 1: ROS 2 Graph
claude code --skill diagram-generator \
  --diagramType "ros2-architecture" \
  --topic "ROS 2 nodes, topics, and services"

# Diagram 2: Publisher-Subscriber Pattern
claude code --skill diagram-generator \
  --diagramType "ros2-architecture" \
  --topic "Publisher-subscriber communication flow"

# Diagram 3: DDS Middleware
claude code --skill diagram-generator \
  --diagramType "system-overview" \
  --topic "DDS layer in ROS 2 architecture"
```

**Step 3.3: Review Code Examples**
- [ ] Python 3.10+ syntax
- [ ] Type hints included
- [ ] Docstrings present
- [ ] Comments explain why, not what
- [ ] Tested (at least syntax-checked)

**Step 3.4: Review Chapter 2**
- [ ] 3500 words achieved
- [ ] 3+ diagrams
- [ ] 3-5 code examples
- [ ] Links to Chapter 1
- [ ] Prepares for Chapter 3

**Estimated Time:** 5-7 hours

---

### Chapter 3: Building First ROS 2 Nodes (Day 4)

**Input:**
- Spec: `specs/module-1/chapter-03-spec.md`
- Reference: Chapters 1 & 2

**Step 4.1: Generate Content**
```bash
claude code --agent tech_writer \
  --spec specs/module-1/chapter-03-spec.md \
  --output book-write/docs/module-1/chapter-03-first-nodes.md \
  --context "Hands-on chapter, complete working examples, lab exercise"
```

**Step 4.2: Generate Diagrams (3+)**
```bash
# Diagram 1: Node Communication
claude code --skill diagram-generator \
  --diagramType "ros2-architecture" \
  --topic "Publisher and subscriber nodes exchanging messages"

# Diagram 2: Launch File Structure
claude code --skill diagram-generator \
  --diagramType "workflow" \
  --topic "ROS 2 launch file execution flow"

# Diagram 3: Custom Message Definition
claude code --skill diagram-generator \
  --diagramType "system-overview" \
  --topic "Creating and using custom ROS 2 message types"
```

**Step 4.3: Create Lab Exercise**
```bash
# Lab exercise is part of chapter content
# Ensure it includes:
# - Step-by-step instructions
# - Expected output
# - Verification checklist
# - Troubleshooting tips
```

**Step 4.4: Test Code Examples**
```bash
# Extract code examples to separate files
mkdir -p book-write/docs/module-1/examples/chapter-03

# Test each example
# cd book-write/docs/module-1/examples/chapter-03
# python3 publisher_example.py
# python3 subscriber_example.py
```

**Step 4.5: Review Chapter 3**
- [ ] 4000 words achieved
- [ ] 3+ diagrams
- [ ] 5+ working code examples
- [ ] Lab exercise complete
- [ ] All code tested
- [ ] Troubleshooting section

**Estimated Time:** 6-8 hours

---

### Chapter 4: URDF Robot Descriptions (Day 5)

**Input:**
- Spec: `specs/module-1/chapter-04-spec.md`
- Reference: All previous chapters

**Step 5.1: Generate Content**
```bash
claude code --agent tech_writer \
  --spec specs/module-1/chapter-04-spec.md \
  --output book-write/docs/module-1/chapter-04-urdf.md \
  --context "Technical chapter, URDF syntax, humanoid modeling, RViz"
```

**Step 5.2: Generate Diagrams (3+)**
```bash
# Diagram 1: URDF Structure
claude code --skill diagram-generator \
  --diagramType "system-overview" \
  --topic "URDF file structure with links and joints"

# Diagram 2: Robot Kinematic Chain
claude code --skill diagram-generator \
  --diagramType "kinematic-chain" \
  --topic "Humanoid robot joint hierarchy from base to end-effector"

# Diagram 3: Coordinate Frames
claude code --skill diagram-generator \
  --diagramType "system-overview" \
  --topic "TF coordinate frame tree for humanoid robot"
```

**Step 5.3: Create URDF Examples**
```bash
# Create example URDF files
mkdir -p book-write/docs/module-1/examples/chapter-04

# simple_humanoid.urdf
# humanoid_with_sensors.urdf
# humanoid_xacro.xacro
```

**Step 5.4: Review Chapter 4**
- [ ] 3500 words achieved
- [ ] 3+ diagrams
- [ ] Complete URDF examples
- [ ] RViz visualization instructions
- [ ] Xacro usage explained
- [ ] Links to Module 2 preview

**Estimated Time:** 5-7 hours

---

## Phase 3: Module-Level Integration (Day 6)

### Step 6.1: Create Module Index
```bash
# Create module-1/index.md or update intro in chapter-01
# Include:
# - Module overview
# - Learning objectives
# - Chapter navigation
# - Prerequisites
```

### Step 6.2: Update Sidebar (TypeScript)
```typescript
// book-write/sidebars.ts
{
  type: 'category',
  label: '📘 Module 1: ROS 2',
  collapsible: true,
  items: [
    'module-1/chapter-01-intro-physical-ai',
    'module-1/chapter-02-ros2-architecture',
    'module-1/chapter-03-first-nodes',
    'module-1/chapter-04-urdf',
  ],
}
```

### Step 6.3: Cross-Chapter Links
Review all chapters and ensure:
- [ ] Chapter 1 → Chapter 2 link exists
- [ ] Chapter 2 → Chapter 3 link exists
- [ ] Chapter 3 → Chapter 4 link exists
- [ ] Chapter 4 → Module 2 preview link
- [ ] All internal links work

### Step 6.4: Create Lab Solutions
```bash
# Create separate solutions folder
mkdir -p book-write/docs/module-1/labs/solutions

# Add solution files with <details> blocks
```

---

## Phase 4: Personalization & Translation (Day 7)

### Step 7.1: Add Personalization
For each chapter:
```bash
claude code --skill personalized-content \
  --input "book-write/docs/module-1/chapter-01-intro-physical-ai.md" \
  --user-profile "beginner" \
  --output "chapter-01-beginner.md"

# Then integrate personalization callouts into main file
# Add :::info boxes for beginners
# Add :::tip boxes for advanced users
```

### Step 7.2: Prepare Translation
```bash
# Generate Urdu versions
claude code --skill translate-to-urdu \
  --input "book-write/docs/module-1/chapter-01-intro-physical-ai.md" \
  --output "book-write/i18n/ur/docusaurus-plugin-content-docs/current/module-1/chapter-01-intro-physical-ai.md"

# Repeat for all chapters
```

### Step 7.3: Add Personalize/Translate Buttons
```tsx
// Add to each chapter start
import PersonalizeButton from '@site/src/components/PersonalizeButton';
import TranslateButton from '@site/src/components/TranslateButton';

<PersonalizeButton chapterId="module-1-chapter-01" />
<TranslateButton targetLang="ur" />
```

---

## Phase 5: Quality Assurance (Day 7)

### Step 8.1: Content Review Checklist

**For Each Chapter:**
- [ ] Word count within ±10% of target
- [ ] 3+ Mermaid diagrams present and rendering
- [ ] All code examples syntax-correct
- [ ] Frontmatter complete
- [ ] Links functional (internal and external)
- [ ] No TODO or placeholder text
- [ ] Images (if any) display correctly

### Step 8.2: Technical Accuracy
- [ ] ROS 2 commands correct for Humble
- [ ] Python code uses 3.10+ features
- [ ] URDF syntax valid
- [ ] File paths accurate

### Step 8.3: Pedagogical Quality
- [ ] Learning objectives met
- [ ] Concepts build progressively
- [ ] Examples relevant and clear
- [ ] Labs are doable in stated time
- [ ] Troubleshooting covers common issues

### Step 8.4: Test Build
```bash
cd book-write
npm run build

# Check for:
# - No build errors
# - All pages accessible
# - Diagrams render
# - Navigation works
```

### Step 8.5: Test Locally
```bash
npm run start

# Manual testing:
# - Read through each chapter
# - Click all links
# - Test code examples (copy-paste)
# - Verify sidebar navigation
# - Check mobile responsive
```

---

## Phase 6: Documentation & Handoff (Day 7)

### Step 9.1: Create Module Summary
```bash
# Document in specs/module-1/completion-report.md
# Include:
# - What was generated
# - Deviations from specs (if any)
# - Known issues
# - Next steps (Module 2)
```

### Step 9.2: Update Main README
```markdown
## Module 1 Status
✅ Chapter 1: Introduction to Physical AI (4000 words)
✅ Chapter 2: ROS 2 Architecture (3500 words)
✅ Chapter 3: First ROS 2 Nodes (4000 words)
✅ Chapter 4: URDF Descriptions (3500 words)

**Total:** 15,000 words
**Diagrams:** 12+ Mermaid diagrams
**Code Examples:** 15+ working examples
**Labs:** 4 hands-on exercises
```

### Step 9.3: Git Commit Strategy
```bash
# Commit each chapter separately
git add book-write/docs/module-1/chapter-01-intro-physical-ai.md
git commit -m "feat(module-1): Add Chapter 1 - Introduction to Physical AI"

git add book-write/docs/module-1/chapter-02-ros2-architecture.md
git commit -m "feat(module-1): Add Chapter 2 - ROS 2 Architecture"

# etc.

# Final commit
git add .
git commit -m "feat(module-1): Complete Module 1 - The Robotic Nervous System"
git push origin main
```

---

## Timeline Summary

| Day | Phase | Tasks | Time |
|-----|-------|-------|------|
| 1 | Setup | Verify specs, create folders | 1-2h |
| 2 | Chapter 1 | Content + diagrams | 4-6h |
| 3 | Chapter 2 | Content + diagrams + code | 5-7h |
| 4 | Chapter 3 | Content + diagrams + code + lab | 6-8h |
| 5 | Chapter 4 | Content + diagrams + URDF | 5-7h |
| 6 | Integration | Module index, sidebar, links | 3-4h |
| 7 | Polish | Personalization, translation, QA | 4-6h |

**Total Estimated Time:** 28-40 hours (5-7 days)

---

## Success Criteria

✅ All 4 chapters generated from specs
✅ Total ~15,000 words achieved
✅ 12+ diagrams included (3 per chapter minimum)
✅ 15+ code examples tested
✅ 4 lab exercises complete
✅ Sidebar navigation configured
✅ All internal links work
✅ Builds without errors
✅ Personalization rules applied
✅ Urdu translation ready
✅ Ready for Module 2

---

## Troubleshooting

### Issue: Content Too Short
**Solution:** Use tech_writer subagent to expand specific sections

### Issue: Diagrams Not Rendering
**Solution:** Verify Mermaid syntax, check Docusaurus config

### Issue: Code Examples Don't Work
**Solution:** Test in actual ROS 2 environment, update to Humble syntax

### Issue: Build Fails
**Solution:** Check frontmatter YAML, verify file paths, review console errors

---

## Next Steps After Module 1

1. Review completion report
2. Gather feedback (self-review or peers)
3. Apply learnings to Module 2 specs
4. Start Module 2 generation with same workflow

---
