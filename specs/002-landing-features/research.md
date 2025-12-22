# Research: Landing Page Features Section

**Feature**: 002-landing-features
**Date**: 2025-12-22

## Research Summary

This document captures research findings for implementing the interactive features showcase section on the landing page.

---

## 1. Existing Landing Page Structure

### Question
What is the current landing page structure and where should the features section be placed?

### Decision
Add a new `InteractiveFeatures` component after the existing `HomepageFeatures` section in `index.tsx`

### Rationale
- Landing page already has `HomepageFeatures` for course content features
- New section will focus on *interactive features* (Personalization, Translation, Chatbot)
- Keeps separation of concerns - course features vs. interactive features
- Position after `HomepageFeatures` and `WelcomeSection` is natural flow

### Code Reference
```typescript
// book-write/src/pages/index.tsx:56-69
export default function Home(): ReactNode {
  return (
    <Layout>
      <HomepageHeader />
      <main>
        <WelcomeSection />
        <HomepageFeatures />  // Course features (existing)
        <InteractiveFeatures /> // NEW - Interactive features section
      </main>
    </Layout>
  );
}
```

---

## 2. Component Architecture

### Question
Should we reuse existing `FeatureCard` or create a new component?

### Decision
Create a new `InteractiveFeatureCard` component with CTA button support

### Rationale
- Existing `FeatureCard` doesn't have CTA button support
- New component needs auth-aware navigation (anonymous vs logged-in)
- Need to integrate with `useAuth` hook for conditional navigation
- Keeps existing FeatureCard unchanged (backward compatible)

### Alternatives Considered

| Alternative | Rejected Because |
|-------------|------------------|
| Extend existing FeatureCard | Would require modifying all existing usages |
| Add optional CTA to FeatureCard | Increases complexity for simple cards |
| Inline everything in section | Poor reusability and maintainability |

### Component Structure
```
book-write/src/components/
├── InteractiveFeatures/
│   ├── index.tsx           # Main section component
│   ├── InteractiveFeatureCard.tsx  # Individual card with CTA
│   └── styles.module.css   # Styling with dark mode
```

---

## 3. Auth-Aware Navigation

### Question
How to handle different navigation for anonymous vs logged-in users?

### Decision
Use `useAuth` hook from existing AuthProvider to determine navigation

### Rationale
- AuthProvider already exists and exports `useAuth` hook
- Same pattern used in ChapterPersonalizer and other components
- Provides `user` object to check login status
- No additional dependencies needed

### Implementation Pattern
```typescript
// InteractiveFeatureCard.tsx
import { useAuth } from '@site/src/components/Auth/AuthProvider';

const { user } = useAuth();

// Anonymous user -> sign-up page
// Logged-in user -> feature page
const handleClick = () => {
  if (!user) {
    navigate('/auth');
  } else {
    navigate(featureUrl);
  }
};
```

### Feature URLs (logged-in users)
| Feature | URL |
|---------|-----|
| Personalization | `/docs/module-1/chapter-01-intro-physical-ai` (first chapter) |
| Translation | `/docs/module-1/chapter-01-intro-physical-ai` (first chapter) |
| Chatbot | Opens chatbot modal (no navigation) |

---

## 4. Icons Strategy

### Question
What icons to use for the three features?

### Decision
Use inline SVG icons (no external dependencies)

### Rationale
- No additional library needed (like Heroicons or FontAwesome)
- Consistent with existing Docusaurus patterns
- Faster loading (no network requests)
- Full control over styling and theming

### Icon Designs
- **Personalization**: User with gear/settings icon
- **Translation**: Globe with language/text icon
- **Chatbot**: Chat bubble with robot/AI icon

---

## 5. Responsive Design

### Question
How should cards layout on different screen sizes?

### Decision
CSS Grid with responsive breakpoints

### Rationale
- Clean 3-column layout on desktop
- 2-column on tablet (easier scanning)
- 1-column on mobile (vertical stack)
- Matches existing FeatureCard grid pattern

### Breakpoints
```css
.featureGrid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);  /* Desktop: 3 columns */
  gap: 2rem;
}

@media (max-width: 996px) {
  grid-template-columns: repeat(2, 1fr);  /* Tablet: 2 columns */
}

@media (max-width: 768px) {
  grid-template-columns: 1fr;  /* Mobile: 1 column */
}
```

---

## 6. Dark Mode Support

### Question
How to ensure dark mode compatibility?

### Decision
Use CSS custom properties and `[data-theme='dark']` selector

### Rationale
- Docusaurus uses `data-theme` attribute for theming
- Existing components follow this pattern
- CSS custom properties allow easy theming
- Consistent with site-wide dark mode

### Implementation
```css
.card {
  background: var(--ifm-background-surface-color);
  border: 1px solid var(--ifm-color-emphasis-200);
}

[data-theme='dark'] .card {
  background: var(--ifm-background-surface-color);
  border-color: var(--ifm-color-emphasis-300);
}
```

---

## 7. Chatbot Integration

### Question
How should the Chatbot CTA work since it doesn't navigate to a page?

### Decision
Use the existing chatbot open mechanism via a custom event or direct function call

### Rationale
- Chatbot is a modal/overlay, not a page
- Need to trigger the FloatingButton's click or set open state
- Can dispatch custom event that Chatbot component listens to
- Keep it simple - just open the chatbot modal

### Implementation Options
1. **Custom Event**: Dispatch `open-chatbot` event, FloatingButton listens
2. **Context**: Create ChatbotContext to share open state (overkill)
3. **Direct Link**: Link to any chapter - chatbot is always accessible (simplest)

**Selected**: Option 3 - Link to first chapter with instruction to use chatbot button

---

## Conclusion

All research questions resolved. No NEEDS CLARIFICATION items remain.
Ready for implementation.
