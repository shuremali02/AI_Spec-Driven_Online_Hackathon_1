# Research: Frontend Fixes & UI Enhancements

**Feature**: frontend-fixes
**Date**: 2025-12-16
**Status**: Complete

---

## 1. Navbar Authentication Integration

### Decision: Theme Swizzling with Custom Component

**Rationale**: Docusaurus supports custom navbar items via configuration, but for dynamic auth state handling (showing different UI based on login status), theme swizzling provides full control over rendering logic.

**Approach**:
1. Create a custom `NavbarAuth` component that reads auth state via `useAuth()` hook
2. Render either Sign Up/Sign In buttons or Profile icon based on auth state
3. Inject component via swizzled Navbar or custom navbar item type

**Alternatives Considered**:

| Approach | Pros | Cons | Decision |
|----------|------|------|----------|
| Config-based HTML items | Simple, no swizzling | Static HTML, no React state | Rejected |
| Full Navbar swizzle | Complete control | Heavy, maintenance burden | Rejected |
| Custom component + Root wrapper | Clean separation | Requires auth context at root | **Selected** |

**Implementation Pattern**:
```tsx
// NavbarAuth component pattern
const NavbarAuth = () => {
  const { user, isLoading } = useAuth();

  if (isLoading) return <Skeleton />;
  if (user) return <ProfileIcon />;
  return <AuthButtons />;
};
```

**Key Finding**: Docusaurus navbar items support `type: 'html'` for custom content, but this renders static HTML. For React components with state, we need to either:
1. Swizzle `@theme/Navbar/Content` to inject our component
2. Use `@theme/Root` wrapper to provide auth context and render floating elements

---

## 2. Auth Context Provider Strategy

### Decision: Root-level AuthProvider Wrapper

**Rationale**: The existing `AuthProvider` in `src/components/Auth/AuthProvider.tsx` manages session state via Better-Auth. To make auth state available throughout the app (navbar, sidebar, pages), wrap at Root level.

**Current State**:
- AuthProvider exists and is functional
- Currently only used on `/auth` and `/profile` pages
- Uses `useSession()` from Better-Auth client

**Implementation**:
```tsx
// src/theme/Root.tsx
import { AuthProvider } from '@site/src/components/Auth/AuthProvider';

export default function Root({ children }) {
  return <AuthProvider>{children}</AuthProvider>;
}
```

**Consideration**: Root.tsx already exists with chatbot integration. Extend it to include AuthProvider.

---

## 3. URL Parameter Handling for Auth Mode

### Decision: React hook with URLSearchParams

**Rationale**: The `/auth` page must read `?mode=signup` or `?mode=signin` to display the appropriate form.

**Implementation**:
```tsx
// In auth.tsx
const [searchParams] = useSearchParams();
const initialMode = searchParams.get('mode') === 'signup' ? 'signup' : 'signin';
```

**Note**: Docusaurus supports standard React Router hooks. Use `useLocation` or browser's `URLSearchParams`.

---

## 4. Mobile Sidebar Auth Section

### Decision: Swizzle NavbarSidebar/Content or use CSS injection

**Rationale**: Mobile sidebar needs auth links (when unauthenticated) or profile section (when authenticated).

**Approaches Evaluated**:

| Approach | Complexity | Maintainability | Decision |
|----------|------------|-----------------|----------|
| Swizzle NavbarSidebar | Medium | Medium | Consider |
| CSS-positioned overlay | Low | Low | Rejected |
| Custom sidebar plugin | High | Low | Rejected |
| Swizzle Navbar/MobileSidebar | Medium | Medium | **Selected** |

**Finding**: Docusaurus mobile sidebar is rendered by `@theme/Navbar/MobileSidebar`. Swizzling this component allows inserting auth section at top.

---

## 5. Sidebar CSS Transparency Fix

### Decision: Explicit background-color with theme variables

**Root Cause Analysis**:
The current custom.css uses `var(--ifm-background-color)` which should resolve correctly. However, the issue may be:
1. Specificity conflict with backdrop-filter
2. Missing explicit light mode value
3. Inheritance from parent with transparency

**Fix Strategy**:
```css
/* Light mode - explicit white background */
.navbar-sidebar {
  background-color: #ffffff;
}

/* Dark mode - use theme variable */
[data-theme='dark'] .navbar-sidebar {
  background-color: var(--ifm-background-color);
}

/* Remove any backdrop-filter that might cause issues */
.navbar-sidebar,
.navbar-sidebar__items {
  backdrop-filter: none;
}
```

**Testing Checklist**:
- [ ] Fresh page load in light mode
- [ ] Fresh page load in dark mode
- [ ] Theme switch light → dark
- [ ] Theme switch dark → light
- [ ] Scroll behind sidebar (parallax effect)

---

## 6. Landing Page Card Conversion

### Decision: Create FeatureCard component with CSS animations

**Current Structure**:
- `HomepageFeatures` component renders 3 feature items
- Each item has SVG icon, title, description
- Layout uses Bootstrap-like grid (`col col--4`)

**New Structure**:
- Create `FeatureCard` component with card styling
- Use CSS Grid for responsive layout
- Add CSS animations for hover and entrance

**Animation Strategy**:
```css
/* Hover animation */
.featureCard {
  transition: transform 300ms ease-out, box-shadow 300ms ease-out;
}

.featureCard:hover {
  transform: translateY(-4px);
  box-shadow: var(--ifm-global-shadow-tl);
}

/* Entrance animation */
@keyframes fadeInUp {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.featureCard {
  animation: fadeInUp 400ms ease-out;
  animation-fill-mode: both;
}

/* Stagger */
.featureCard:nth-child(1) { animation-delay: 0ms; }
.featureCard:nth-child(2) { animation-delay: 100ms; }
.featureCard:nth-child(3) { animation-delay: 200ms; }
```

**Reduced Motion**:
```css
@media (prefers-reduced-motion: reduce) {
  .featureCard {
    animation: none;
    transition: none;
  }
  .featureCard:hover {
    transform: none;
  }
}
```

---

## 7. Responsive Breakpoints

### Decision: Use Docusaurus/Infima standard breakpoints

**Docusaurus Default Breakpoints**:
- Mobile: ≤996px (Docusaurus uses 996px for mobile nav toggle)
- Desktop: >996px

**Custom Breakpoints for Cards**:
- Mobile: ≤768px → Single column
- Tablet: 769px - 1199px → Two columns
- Desktop: ≥1200px → Three columns

**CSS Grid Implementation**:
```css
.featureGrid {
  display: grid;
  gap: 24px;
  grid-template-columns: repeat(3, 1fr);
}

@media (max-width: 1199px) {
  .featureGrid {
    grid-template-columns: repeat(2, 1fr);
  }
}

@media (max-width: 768px) {
  .featureGrid {
    grid-template-columns: 1fr;
    gap: 16px;
  }
}
```

---

## 8. Accessibility Considerations

### Decision: Follow WCAG AA compliance

**Requirements**:
1. **Focus States**: All interactive elements have visible focus indicators
2. **Color Contrast**: 4.5:1 minimum for text
3. **Touch Targets**: 44x44px minimum
4. **Reduced Motion**: Respect `prefers-reduced-motion`
5. **Keyboard Navigation**: All buttons/links focusable

**Implementation**:
```css
/* Focus states */
.authButton:focus-visible,
.profileIcon:focus-visible {
  outline: 2px solid var(--ifm-color-primary);
  outline-offset: 2px;
}

/* Touch targets */
.authButton {
  min-height: 44px;
  min-width: 44px;
  padding: 8px 16px;
}
```

---

## 9. Theme Consistency

### Decision: Use existing CSS variables

**Variables to Use**:
- `--ifm-color-primary` - Primary button/link color
- `--ifm-background-color` - Page/sidebar background
- `--ifm-background-surface-color` - Card background
- `--ifm-font-color-base` - Text color
- `--ifm-global-shadow-md` - Card shadow
- `--ifm-card-border-radius` - Card corners (12px)

**Theme-Aware Styling**:
```css
/* Card styling */
.featureCard {
  background: var(--ifm-background-surface-color);
  border: 1px solid var(--ifm-toc-border-color);
  border-radius: var(--ifm-card-border-radius);
  box-shadow: var(--ifm-global-shadow-md);
}

[data-theme='dark'] .featureCard {
  background: var(--ifm-background-surface-color);
  border-color: var(--ifm-toc-border-color);
}
```

---

## 10. File Structure Decision

### Decision: Component-based organization

**New Files**:
```
book-write/src/
├── components/
│   ├── NavbarAuth/
│   │   ├── index.tsx          # Main auth component
│   │   └── styles.module.css  # Scoped styles
│   └── FeatureCard/
│       ├── index.tsx          # Card component
│       └── styles.module.css  # Card styles
└── theme/
    └── Root.tsx               # Existing, extend with AuthProvider
```

**Modified Files**:
```
book-write/src/
├── pages/
│   └── auth.tsx               # Add URL param handling
├── components/
│   └── HomepageFeatures/
│       ├── index.tsx          # Use FeatureCard
│       └── styles.module.css  # Update grid layout
└── css/
    └── custom.css             # Sidebar fix, animations
```

---

## 11. Dependencies

### No New Dependencies Required

**Existing Dependencies Used**:
- `@docusaurus/Link` - Navigation
- `clsx` - Conditional classnames
- `react` - UI components
- Better-Auth client (existing)

**Not Adding**:
- Animation libraries (Framer Motion, etc.)
- Icon libraries (using inline SVG or existing)
- CSS preprocessors (using CSS modules + custom.css)

---

## Summary of Technical Decisions

| Area | Decision | Rationale |
|------|----------|-----------|
| Navbar Auth | Custom component + Root wrapper | Full React state control |
| Auth Context | Root-level provider | App-wide auth state |
| URL Params | URLSearchParams | Standard browser API |
| Mobile Sidebar | Swizzle MobileSidebar | Insert auth section |
| Sidebar CSS | Explicit bg + remove backdrop-filter | Fix transparency bug |
| Feature Cards | CSS Grid + CSS animations | No JS dependencies |
| Breakpoints | 768px / 1199px | Standard responsive tiers |
| Accessibility | WCAG AA | Required for production |
| Theme | Existing CSS variables | Consistency |
| File Structure | Component folders | Maintainability |
