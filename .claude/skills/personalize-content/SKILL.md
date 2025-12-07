---
name: "personalize-book-content"
description: "Personalize book chapters, explanations, examples, tone, and writing style based on target readers. Helps adapt content for beginners, students, professionals, or specific age groups."
version: "1.0.0"
---

# Personalize Book Content Skill

## When to Use This Skill

Use this skill when:
- User says: “Is chapter ko beginner-friendly banao”
- User wants simpler or more advanced explanations
- User needs tone adjustments (storytelling, academic, friendly, mentor-style)
- User wants content tailored for:
  - students
  - developers
  - beginners
  - researchers
  - kids or young readers
  - business professionals
- User wants chapter rewrites with more clarity, flow, or depth
- User wants examples adjusted to their audience (e.g., tech examples vs. real-life examples)

## How This Skill Works

1. Understand the **source content**
2. Detect user’s **target level** (beginner, intermediate, expert)
3. Detect user’s **tone preference**
4. Adapt:
   - complexity  
   - explanation depth  
   - sentence simplicity  
   - analogy and examples  
   - narrative style  
5. Produce a clean, rewritten version that fits the exact audience and tone

## Output Format

Return the following:

- **Tone Applied:** e.g., “Beginner-friendly educational tone”
- **Reader Level:** beginner / intermediate / expert / kids / general audience
- **Personalized Chapter Version:** rewritten complete content
- **Improved Flow Notes:** 2–3 bullet points on what was improved
- **Optional Alternate Style:** Provide second variation if helpful

## Example

**Input:**  
“Make this more beginner-friendly:  
‘Neural networks operate using interconnected layers that propagate gradients backward through optimization algorithms.’”

**Output:**

### ⭐ Tone Applied  
Simple, beginner-friendly educational tone

### 🎯 Reader Level  
Absolute beginners in AI

### ✍️ Personalized Version  
"Neural networks are computer systems made of connected layers of small decision-making units. They learn by checking their mistakes and adjusting themselves step by step, similar to how you improve after practicing something repeatedly."

### 🔁 Optional Alternate Style  
“Think of a neural network like a group of students passing notes back and forth. Each student learns from the feedback they receive and tries to improve their answers the next time.”

### 📌 Improved Flow Notes  
- Removed technical jargon  
- Added relatable analogy  
- Simplified explanation structure  

