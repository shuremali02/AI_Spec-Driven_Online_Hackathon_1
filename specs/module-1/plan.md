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
  --topic "Publisher-subscriber communication flow\"\

# Diagram 3: DDS Middleware\nclaude code --skill diagram-generator \\\n  --diagramType \"system-overview\" \\\n  --topic \"DDS layer in ROS 2 architecture\"\n```\n\n**Step 3.3: Review Code Examples**\n- [ ] Python 3.10+ syntax\n- [ ] Type hints included\n- [ ] Docstrings present\n- [ ] Comments explain why, not what\n- [ ] Tested (at least syntax-checked)\n\n**Step 3.4: Review Chapter 2**\n- [ ] 3500 words achieved\n- [ ] 3+ diagrams\n- [ ] 3-5 code examples\n- [ ] Links to Chapter 1\n- [ ] Prepares for Chapter 3\n\n**Estimated Time:** 5-7 hours\n\n### Chapter 3: Building First ROS 2 Nodes (Day 4)\n\n**Input:**\n- Spec: `specs/module-1/spec.md` (relevant Chapter 3 section)\n- Reference: Chapters 1 & 2\n\n**Step 4.1: Generate Content**\n```bash\nclaude code --agent tech_writer \\\n  --spec specs/module-1/spec.md (Chapter 3 section) \\\n  --output book-write/docs/module-1/chapter-03-first-nodes.md \\\n  --context \"Hands-on chapter, complete working examples, lab exercise\"\n```\n\n**Step 4.2: Generate Diagrams (3+)**\n```bash\n# Diagram 1: Node Communication\nclaude code --skill diagram-generator \\\n  --diagramType \"ros2-architecture\" \\\n  --topic \"Publisher and subscriber nodes exchanging messages\"\n\n# Diagram 2: Launch File Structure\nclaude code --skill diagram-generator \\\n  --diagramType \"workflow\" \\\n  --topic \"ROS 2 launch file execution flow\"\n\n# Diagram 3: Custom Message Definition\nclaude code --skill diagram-generator \\\n  --diagramType \"system-overview\" \\\n  --topic \"Creating and using custom ROS 2 message types\"\n```\n\n**Step 4.3: Create Lab Exercise**\n```bash\n# Lab exercise is part of chapter content\n# Ensure it includes:\n# - Step-by-step instructions\n# - Expected output\n# - Verification checklist\n# - Troubleshooting tips\n```\n\n**Step 4.4: Test Code Examples**\n```bash\n# Extract code examples to separate files\nmkdir -p book-write/docs/module-1/examples/chapter-03\n\n# Test each example\n# cd book-write/docs/module-1/examples/chapter-03\n# python3 publisher_example.py\n# python3 subscriber_example.py\n```\n\n**Step 4.5: Review Chapter 3**\n- [ ] 4000 words achieved\n- [ ] 3+ diagrams\n- [ ] 5+ working code examples\n- [ ] Lab exercise complete\n- [ ] All code tested\n- [ ] Troubleshooting section\n\n**Estimated Time:** 6-8 hours\n\n### Chapter 4: URDF Robot Descriptions (Day 5)\n\n**Input:**\n- Spec: `specs/module-1/spec.md` (relevant Chapter 4 section)\n- Reference: All previous chapters\n\n**Step 5.1: Generate Content**\n```bash\nclaude code --agent tech_writer \\\n  --spec specs/module-1/spec.md (Chapter 4 section) \\\n  --output book-write/docs/module-1/chapter-04-urdf.md \\\n  --context \"Technical chapter, URDF syntax, humanoid modeling, RViz\"\n```\n\n**Step 5.2: Generate Diagrams (3+)**\n```bash\n# Diagram 1: URDF Structure\nclaude code --skill diagram-generator \\\n  --diagramType \"system-overview\" \\\n  --topic \"URDF file structure with links and joints\"\n\n# Diagram 2: Robot Kinematic Chain\nclaude code --skill diagram-generator \\\n  --diagramType \"kinematic-chain\" \\\n  --topic \"Humanoid robot joint hierarchy from base to end-effector\"\n\n# Diagram 3: Coordinate Frames\nclaude code --skill diagram-generator \\\n  --diagramType \"system-overview\" \\\n  --topic \"TF coordinate frame tree for humanoid robot\"\n```\n\n**Step 5.3: Create URDF Examples**\n```bash\n# Create example URDF files\nmkdir -p book-write/docs/module-1/examples/chapter-04\n\n# simple_humanoid.urdf\n# humanoid_with_sensors.urdf\n# humanoid_xacro.xacro\n```\n\n**Step 5.4: Review Chapter 4**\n- [ ] 3500 words achieved\n- [ ] 3+ diagrams\n- [ ] Complete URDF examples\n- [ ] RViz visualization instructions\n- [ ] Xacro usage explained\n- [ ] Links to Module 2 preview\n\n**Estimated Time:** 5-7 hours\n\n## Phase 3: Module-Level Integration (Day 6)\n\n### Step 6.1: Create Module Index\n```bash\n# Create module-1/index.md or update intro in chapter-01\n# Include:\n# - Module overview\n# - Learning objectives\n# - Chapter navigation\n# - Prerequisites\n```\n\n### Step 6.2: Update Sidebar (TypeScript)\n```typescript\n// book-write/sidebars.ts\n{\n  type: 'category',\n  label: '📘 Module 1: ROS 2',\n  collapsible: true,\n  items: [\n    'module-1/chapter-01-intro-physical-ai',\n    'module-1/chapter-02-ros2-architecture',\n    'module-1/chapter-03-first-nodes',\n    'module-1/chapter-04-urdf',\n  ],\n}\n```\n\n### Step 6.3: Cross-Chapter Links\nReview all chapters and ensure:\n- [ ] Chapter 1 → Chapter 2 link exists\n- [ ] Chapter 2 → Chapter 3 link exists\n- [ ] Chapter 3 → Chapter 4 link exists\n- [ ] Chapter 4 → Module 2 preview link\n- [ ] All internal links work\n\n### Step 6.4: Create Lab Solutions\n```bash\n# Create separate solutions folder\nmkdir -p book-write/docs/module-1/labs/solutions\n\n# Add solution files with <details> blocks\n```\n\n## Phase 4: Personalization & Translation (Day 7)\n\n### Step 7.1: Add Personalization\nFor each chapter:\n```bash\nclaude code --skill personalized-content \\\n  --input \"book-write/docs/module-1/chapter-01-intro-physical-ai.md\" \\\n  --user-profile \"beginner\" \\\n  --output \"chapter-01-beginner.md\"\n\n# Then integrate personalization callouts into main file\n# Add :::info boxes for beginners\n# Add :::tip boxes for advanced users\n```\n\n### Step 7.2: Prepare Translation\n```bash\nclaude code --skill translate-to-urdu \\\n  --input \"book-write/docs/module-1/chapter-01-intro-physical-ai.md\" \\\n  --output \"book-write/i18n/ur/docusaurus-plugin-content-docs/current/module-1/chapter-01-intro-physical-ai.md\"\n\n# Repeat for all chapters\n```\n\n### Step 7.3: Add Personalize/Translate Buttons\n```tsx\n// Add to each chapter start\nimport PersonalizeButton from '@site/src/components/PersonalizeButton';\nimport TranslateButton from '@site/src/components/TranslateButton';\n\n<PersonalizeButton chapterId=\"module-1-chapter-01\" />\n<TranslateButton targetLang=\"ur\" />\n```\n\n## Phase 5: Quality Assurance (Day 7)\n\n### Step 8.1: Content Review Checklist\n\n**For Each Chapter:**\n- [ ] Word count within ±10% of target\n- [ ] 3+ Mermaid diagrams present and rendering\n- [ ] All code examples syntax-correct\n- [ ] Frontmatter complete\n- [ ] Links functional (internal and external)\n- [ ] No TODO or placeholder text\n- [ ] Images (if any) display correctly\n\n### Step 8.2: Technical Accuracy\n- [ ] ROS 2 commands correct for Humble\n- [ ] Python code uses 3.10+ features\n- [ ] URDF syntax valid\n- [ ] File paths accurate\n\n### Step 8.3: Pedagogical Quality\n- [ ] Learning objectives met\n- [ ] Concepts build progressively\n- [ ] Examples relevant and clear\n- [ ] Labs are doable in stated time\n- [ ] Troubleshooting covers common issues\n\n### Step 8.4: Test Build\n```bash\ncd book-write\nnpm run build\n\n# Check for:\n# - No build errors\n# - All pages accessible\n# - Diagrams render\n# - Navigation works\n```\n\n### Step 8.5: Test Locally\n```bash\nnpm run start\n\n# Manual testing:\n# - Read through each chapter\n# - Click all links\n# - Test code examples (copy-paste)\n# - Verify sidebar navigation\n# - Check mobile responsive\n```\n\n## Phase 6: Documentation & Handoff (Day 7)\n\n### Step 9.1: Create Module Summary\n```bash\n# Document in specs/module-1/completion-report.md\n# Include:\n# - What was generated\n# - Deviations from specs (if any)\n# - Known issues\n# - Next steps (Module 2)\n```\n\n### Step 9.2: Update Main README\n```markdown\n## Module 1 Status\n✅ Chapter 1: Introduction to Physical AI (4000 words)\n✅ Chapter 2: ROS 2 Architecture (3500 words)\n✅ Chapter 3: First ROS 2 Nodes (4000 words)\n✅ Chapter 4: URDF Descriptions (3500 words)\n\n**Total:** 15,000 words\n**Diagrams:** 12+ Mermaid diagrams\n**Code Examples:** 15+ working examples\n**Labs:** 4 hands-on exercises\n```\n\n### Step 9.3: Git Commit Strategy\n```bash\n# Commit each chapter separately\ngit add book-write/docs/module-1/chapter-01-intro-physical-ai.md\ngit commit -m \"feat(module-1): Add Chapter 1 - Introduction to Physical AI\"\n\ngit add book-write/docs/module-1/chapter-02-ros2-architecture.md\ngit commit -m \"feat(module-1): Add Chapter 2 - ROS 2 Architecture\"\n\n# etc.\n\n# Final commit\ngit add .\ngit commit -m \"feat(module-1): Complete Module 1 - The Robotic Nervous System\"\ngit push origin main\n```\n\n## Timeline Summary\n\n| Day | Phase | Tasks | Time |\n|-----|-------|-------|------|\n| 1 | Setup | Verify specs, create folders | 1-2h |\n| 2 | Chapter 1 | Content + diagrams | 4-6h |\n| 3 | Chapter 2 | Content + diagrams + code | 5-7h |\n| 4 | Chapter 3 | Content + diagrams + code + lab | 6-8h |\
| 5 | Chapter 4 | Content + diagrams + URDF | 5-7h |\n| 6 | Integration | Module index, sidebar, links | 3-4h |\n| 7 | Polish | Personalization, translation, QA | 4-6h |\n\n**Total Estimated Time:** 28-40 hours (5-7 days)\n\n## Success Criteria\n\n✅ All 4 chapters generated from specs\n✅ Total ~15,000 words achieved\n✅ 12+ Mermaid diagrams included (3 per chapter minimum)\n✅ 15+ code examples tested\n✅ 4 lab exercises complete\n✅ Sidebar navigation configured\n✅ All internal links work\n✅ Builds without errors\n✅ Personalization rules applied\n✅ Urdu translation ready\n✅ Ready for Module 2\n\n## Troubleshooting\n\n### Issue: Content Too Short\n**Solution:** Use tech_writer subagent to expand specific sections\n\n### Issue: Diagrams Not Rendering\n**Solution:** Verify Mermaid syntax, check Docusaurus config\n\n### Issue: Code Examples Don\'t Work\n**Solution:** Test in actual ROS 2 environment, update to Humble syntax\n\n### Issue: Build Fails\n**Solution:** Check frontmatter YAML, verify file paths, review console errors\n\n## Next Steps After Module 1\n\n1. Review completion report\n2. Gather feedback (self-review or peers)\n3. Apply learnings to Module 2 specs\n4. Start Module 2 generation with same workflow\n