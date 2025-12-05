---
id: 0006
title: Module 1 Implementation Plan Generated
stage: plan
date: 2025-12-05
surface: agent
model: claude-sonnet-4-5-20251101
feature: module-1
branch: 001-physical-ai-robotics-textbook
user: user
command: /sp.plan # Module 1 Implementation Plan... (truncated)
labels: ["implementation-plan", "module-1", "planning"]
links:
  spec: specs/module-1/spec.md
  ticket: null
  adr: null
  pr: null
files:
 - specs/module-1/plan.md
tests:

---

## Prompt

# Module 1 Implementation Plan

## Overview
- **Module:** Module 1 - The Robotic Nervous System (ROS 2)
- **Chapters:** 4
- **Total Words:** ~15,000
- **Timeline:** 5-7 days
- **Output Location:** `book-write/docs/module-1/`

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

## Phase 2: Chapter Generation (Days 2-5)

### Order of Execution
Generate chapters **sequentially** (not parallel) because:
- Chapter 2 references Chapter 1
- Chapter 3 builds on Chapter 2
- Chapter 4 uses concepts from Chapter 3

### Chapter 1: Introduction to Physical AI (Day 2)

**Input:**
- Spec: `specs/module-1/spec.md` (relevant Chapter 1 section)

**Step 2.1: Plan Chapter Structure**
```bash
claude code --skill chapter-outliner \
  --input "Read specs/module-1/spec.md (Chapter 1 section)" \
  --output "Detailed outline for Chapter 1.1"
```

**Step 2.2: Generate Content**
```bash
claude code --agent tech_writer \
  --spec specs/module-1/spec.md (Chapter 1 section) \
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

### Chapter 2: ROS 2 Architecture (Day 3)

**Input:**
- Spec: `specs/module-1/spec.md` (relevant Chapter 2 section)
- Reference: Chapter 1 (already completed)

**Step 3.1: Generate Content**
```bash
claude code --agent tech_writer \
  --spec specs/module-1/spec.md (Chapter 2 section) \
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

### Chapter 3: Building First ROS 2 Nodes (Day 4)

**Input:**
- Spec: `specs/module-1/spec.md` (relevant Chapter 3 section)
- Reference: Chapters 1 & 2

**Step 4.1: Generate Content**
```bash
claude code --agent tech_writer \
  --spec specs/module-1/spec.md (Chapter 3 section) \
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

### Chapter 4: URDF Robot Descriptions (Day 5)

**Input:**
- Spec: `specs/module-1/spec.md` (relevant Chapter 4 section)
- Reference: All previous chapters

**Step 5.1: Generate Content**
```bash
claude code --agent tech_writer \
  --spec specs/module-1/spec.md (Chapter 4 section) \
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

### Step 8.5: Test Locally\n```bash\nnpm run start\n\n# Manual testing:\n# - Read through each chapter\n# - Click all links\n# - Test code examples (copy-paste)\n# - Verify sidebar navigation\n# - Check mobile responsive\n```\n\n## Phase 6: Documentation & Handoff (Day 7)\n\n### Step 9.1: Create Module Summary\n```bash\n# Document in specs/module-1/completion-report.md\n# Include:\n# - What was generated\n# - Deviations from specs (if any)\n# - Known issues\n# - Next steps (Module 2)\n```\n\n### Step 9.2: Update Main README\n```markdown\n## Module 1 Status\n✅ Chapter 1: Introduction to Physical AI (4000 words)\n✅ Chapter 2: ROS 2 Architecture (3500 words)\n✅ Chapter 3: First ROS 2 Nodes (4000 words)\n✅ Chapter 4: URDF Descriptions (3500 words)\n\n**Total:** 15,000 words\n**Diagrams:** 12+ Mermaid diagrams\n**Code Examples:** 15+ working examples\n**Labs:** 4 hands-on exercises\n```\n\n### Step 9.3: Git Commit Strategy\n```bash\n# Commit each chapter separately\ngit add book-write/docs/module-1/chapter-01-intro-physical-ai.md\ngit commit -m \"feat(module-1): Add Chapter 1 - Introduction to Physical AI\"\n\ngit add book-write/docs/module-1/chapter-02-ros2-architecture.md\ngit commit -m \"feat(module-1): Add Chapter 2 - ROS 2 Architecture\"\n\n# etc.\n\n# Final commit\ngit add .\ngit commit -m \"feat(module-1): Complete Module 1 - The Robotic Nervous System\"\ngit push origin main\n```\n\n## Timeline Summary\n\n| Day | Phase | Tasks | Time |\n|-----|-------|-------|------|\n| 1 | Setup | Verify specs, create folders | 1-2h |\n| 2 | Chapter 1 | Content + diagrams | 4-6h |\n| 3 | Chapter 2 | Content + diagrams + code | 5-7h |\n| 4 | Chapter 3 | Content + diagrams + code + lab | 6-8h |\
| 5 | Chapter 4 | Content + diagrams + URDF | 5-7h |\n| 6 | Integration | Module index, sidebar, links | 3-4h |\n| 7 | Polish | Personalization, translation, QA | 4-6h |\n\n**Total Estimated Time:** 28-40 hours (5-7 days)\n\n## Success Criteria\n\n✅ All 4 chapters generated from specs\n✅ Total ~15,000 words achieved\n✅ 12+ Mermaid diagrams included (3 per chapter minimum)\n✅ 15+ code examples tested\n✅ 4 lab exercises complete\n✅ Sidebar navigation configured\n✅ All internal links work\n✅ Builds without errors\n✅ Personalization rules applied\n✅ Urdu translation ready\n✅ Ready for Module 2\n\n## Troubleshooting\n\n### Issue: Content Too Short\n**Solution:** Use tech_writer subagent to expand specific sections\n\n### Issue: Diagrams Not Rendering\n**Solution:** Verify Mermaid syntax, check Docusaurus config\n\n### Issue: Code Examples Don\'t Work\n**Solution:** Test in actual ROS 2 environment, update to Humble syntax\n\n### Issue: Build Fails\n**Solution:** Check frontmatter YAML, verify file paths, review console errors\n\n## Next Steps After Module 1\n\n1. Review completion report\n2. Gather feedback (self-review or peers)\n3. Apply learnings to Module 2 specs\n4. Start Module 2 generation with same workflow\n