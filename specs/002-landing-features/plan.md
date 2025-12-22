# Implementation Plan: Landing Page Features Section

**Branch**: `002-landing-features` | **Date**: 2025-12-22 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/002-landing-features/spec.md`

## Summary

Add an interactive features showcase section to the landing page that highlights three key features (Personalization, Urdu Translation, AI Chatbot) to anonymous and new users, encouraging sign-ups. The section will use auth-aware navigation to direct anonymous users to sign-up and logged-in users directly to features.

## Technical Context

**Language/Version**: TypeScript 5.x, React 18
**Primary Dependencies**: Docusaurus 3.x, existing AuthProvider, existing FeatureCard patterns
**Storage**: N/A (static content, no data storage)
**Testing**: Manual testing (no automated tests required per spec)
**Target Platform**: Web (desktop, tablet, mobile)
**Project Type**: Web application (frontend only)
**Performance Goals**: Cards load within 1 second, smooth hover animations at 60fps
**Constraints**: Must support light/dark mode, responsive down to 320px width
**Scale/Scope**: 3 feature cards, single section component

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| V. Authentication-First Access Control | PASS | Anonymous users see features but CTAs redirect to auth page |
| VI. Personalization Standard | PASS | Feature card describes personalization, links to chapter |
| VIII. Translation Feature Standard | PASS | Feature card describes translation, links to chapter |
| V. RAG Chatbot Integration | PASS | Feature card describes chatbot, links to chapter where chatbot is accessible |

**All gates passed. No violations to justify.**

## Project Structure

### Documentation (this feature)

```text
specs/002-landing-features/
├── plan.md              # This file
├── research.md          # Phase 0 output (complete)
├── quickstart.md        # Phase 1 output (complete)
└── tasks.md             # Phase 2 output (pending /sp.tasks)
```

### Source Code (repository root)

```text
book-write/src/
├── components/
│   ├── InteractiveFeatures/      # NEW - this feature
│   │   ├── index.tsx             # Main section component
│   │   ├── InteractiveFeatureCard.tsx  # Individual card with CTA
│   │   └── styles.module.css     # Responsive + dark mode styles
│   ├── HomepageFeatures/         # Existing (unchanged)
│   ├── FeatureCard/              # Existing (unchanged)
│   └── Auth/
│       └── AuthProvider.tsx      # Existing (use useAuth hook)
└── pages/
    └── index.tsx                 # Existing (add InteractiveFeatures import)
```

**Structure Decision**: Frontend-only feature. New component directory `InteractiveFeatures` follows existing pattern (HomepageFeatures, FeatureCard). No backend changes required.

## Design Decisions

### 1. Separate Component vs Extending Existing

**Decision**: Create new `InteractiveFeatures` component instead of extending `HomepageFeatures`

**Rationale**:
- Clear separation between course content features and interactive features
- New cards need CTA buttons with auth-aware navigation
- Keeps existing components unchanged (no regression risk)
- Different visual hierarchy (section title, larger icons, prominent CTAs)

### 2. Auth-Aware Navigation Pattern

**Decision**: Use `useAuth` hook in `InteractiveFeatureCard` to determine navigation target

**Pattern**:
```typescript
const { user } = useAuth();
const targetUrl = user ? featureUrl : '/auth';
const buttonText = user ? ctaText : 'Sign Up to Access';
```

**Rationale**:
- Consistent with existing auth patterns in codebase
- No additional state management needed
- Clear user feedback (button text changes based on auth state)

### 3. Icon Implementation

**Decision**: Inline SVG icons instead of icon library

**Rationale**:
- No additional dependencies
- Faster loading (no external requests)
- Full control over styling and theming
- Matches existing Docusaurus patterns

### 4. Chatbot CTA Behavior

**Decision**: Link to first chapter page where chatbot is accessible via floating button

**Rationale**:
- Chatbot is a modal/overlay accessible from any docs page
- Simplest implementation without complex event handling
- User lands on content page and can immediately use chatbot button
- Consistent with other feature CTAs (all go to chapter pages)

## Component Specifications

### InteractiveFeatures (Section Component)

**Props**: None (self-contained)

**Responsibilities**:
- Render section with title "Interactive Features"
- Render 3 InteractiveFeatureCard components
- Apply responsive grid layout
- Handle fade-in animations

### InteractiveFeatureCard (Card Component)

**Props**:
```typescript
interface InteractiveFeatureCardProps {
  title: string;           // Card title
  description: string;     // 2-3 sentence description
  icon: React.ReactNode;   // SVG icon component
  ctaText: string;         // Button text for logged-in users
  featureUrl: string;      // Target URL for logged-in users
  animationDelay?: number; // Staggered animation delay
}
```

**Responsibilities**:
- Render icon, title, description, CTA button
- Use `useAuth` to determine navigation behavior
- Apply hover effects and animations

## Styling Specifications

### Colors
- Gradient: `linear-gradient(135deg, #667eea 0%, #764ba2 100%)` (matches existing chatbot theme)
- Card background: `var(--ifm-background-surface-color)`
- Text: `var(--ifm-heading-color)`, `var(--ifm-color-emphasis-600)`

### Responsive Breakpoints
- Desktop (>996px): 3 columns
- Tablet (768-996px): 2 columns
- Mobile (<768px): 1 column

### Animations
- Card fade-in: 0.6s ease-out, staggered by 100ms
- Hover: translateY(-8px), box-shadow increase
- CTA button: scale(1.05) on hover

## Implementation Order

1. **Create directory structure** - mkdir InteractiveFeatures
2. **Create styles** - styles.module.css (can be parallelized)
3. **Create InteractiveFeatureCard** - individual card component
4. **Create index.tsx** - main section with icons and feature data
5. **Update index.tsx (pages)** - import and add section
6. **Test** - manual testing per quickstart.md

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Auth hook not working in pages | High | Verify AuthProvider wraps entire app in Root.tsx |
| Icons not displaying correctly | Medium | Use simple SVG paths, test in both themes |
| Animation performance on mobile | Low | Use transform/opacity only (GPU accelerated) |

## Files to Create/Modify

| File | Action | Description |
|------|--------|-------------|
| `components/InteractiveFeatures/index.tsx` | CREATE | Main section component |
| `components/InteractiveFeatures/InteractiveFeatureCard.tsx` | CREATE | Card component |
| `components/InteractiveFeatures/styles.module.css` | CREATE | Styling |
| `pages/index.tsx` | MODIFY | Add import and render InteractiveFeatures |

## Next Steps

Run `/sp.tasks` to generate implementation tasks.
