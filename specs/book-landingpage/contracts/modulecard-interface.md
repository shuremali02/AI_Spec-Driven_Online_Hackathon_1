# Contract: ModuleCard Component Interface

## Purpose
Define the interface contract for the ModuleCard React component used on the book landing page to ensure consistent module representation across the Physical AI & Humanoid Robotics textbook.

## Component Signature
```typescript
interface ModuleCardProps {
  icon: string;           // Emoji or icon representing the module
  number: number;         // Module number (1-4)
  title: string;          // Module title with technology focus
  duration: string;       // Time commitment (e.g., "Weeks 1-5")
  purpose: string;        // Brief description of module focus
  outcomes: string[];     // Array of 1-3 key learning outcomes
  link: string;           // Relative path to module overview page
}

const ModuleCard: React.FC<ModuleCardProps> = ({
  icon,
  number,
  title,
  duration,
  purpose,
  outcomes,
  link
}) => {
  // Implementation
};
```

## Required Behaviors

### Rendering
- Display the icon prominently at the top of the card
- Show the module number with clear visual hierarchy
- Present the title in a readable font size
- Include duration in a smaller, secondary text style
- Render purpose as a descriptive paragraph
- List outcomes as bullet points or small text items
- Make the entire card clickable to navigate to the link

### Styling Requirements
- Responsive design that works on mobile and desktop
- Consistent spacing and padding across all cards
- Hover effects for interactive feedback
- Accessibility-compliant color contrast
- Proper focus states for keyboard navigation

### Accessibility Contract
- Proper ARIA labels for screen readers
- Keyboard navigable (tab index)
- Semantic HTML structure
- Alt text equivalent for icon representation
- Focus indicators for interactive elements

## Input Validation

### Required Props
All props are required and must be provided:

```typescript
// Validation checks
if (typeof icon !== 'string' || !icon.trim()) {
  throw new Error('icon must be a non-empty string');
}

if (typeof number !== 'number' || number < 1 || number > 4) {
  throw new Error('number must be between 1 and 4');
}

if (typeof title !== 'string' || !title.trim()) {
  throw new Error('title must be a non-empty string');
}

if (typeof duration !== 'string' || !duration.trim()) {
  throw new Error('duration must be a non-empty string');
}

if (typeof purpose !== 'string' || !purpose.trim()) {
  throw new Error('purpose must be a non-empty string');
}

if (!Array.isArray(outcomes) || outcomes.length === 0 || outcomes.length > 5) {
  throw new Error('outcomes must be an array with 1-5 items');
}

if (typeof link !== 'string' || !link.startsWith('/')) {
  throw new Error('link must be a relative path starting with "/"');
}
```

## Error Handling
- Graceful degradation if props are invalid
- Console warnings for development
- Fallback rendering for missing critical props
- No runtime exceptions that break page rendering

## Performance Contract
- Component should render in < 16ms
- Minimal re-renders when possible
- Efficient prop handling
- No memory leaks

## Integration Contract with Landing Page

### Import Statement
```typescript
import ModuleCard from '@site/src/components/ModuleCard';
```

### Usage Pattern
```jsx
<div className="module-grid">
  <ModuleCard
    icon="🧠"
    number={1}
    title="The Robotic Nervous System (ROS 2)"
    duration="Weeks 1-5"
    purpose="Master ROS 2 middleware for robust robot communication and control."
    outcomes={[
      "Understand ROS 2 architecture (nodes, topics, services)",
      "Build Python-based ROS 2 applications with `rclpy`"
    ]}
    link="/docs/module-1/"
  />
</div>
```

### CSS Requirements
The parent container should have:
- `module-grid` class for responsive grid layout
- Proper CSS grid or flexbox styling
- Responsive breakpoints for different screen sizes

## Version Compatibility
- Compatible with React 17+
- Works with Docusaurus v2.0+
- TypeScript definitions available (optional)

## Testing Contract
- Unit tests covering prop validation
- Snapshot tests for visual consistency
- Accessibility tests
- Responsive design verification