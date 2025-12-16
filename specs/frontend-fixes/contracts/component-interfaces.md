# Component Interface Contracts: Frontend Fixes

**Feature**: frontend-fixes
**Date**: 2025-12-16
**Version**: 1.0.0

---

## Overview

This document defines the TypeScript interface contracts for React components in the frontend-fixes feature. These contracts serve as binding agreements for component implementation.

---

## 1. NavbarAuth Component

### File: `src/components/NavbarAuth/index.tsx`

```typescript
/**
 * @component NavbarAuth
 * @description Renders authentication controls in navbar/sidebar
 *
 * @behavior
 * - Loading: Renders skeleton placeholder
 * - Unauthenticated: Renders Sign Up and Sign In buttons
 * - Authenticated: Renders Profile icon with user info
 *
 * @accessibility
 * - All buttons have aria-label
 * - Focus states visible (outline)
 * - Minimum touch target 44x44px
 */

export interface NavbarAuthProps {
  /**
   * Display variant
   * - 'navbar': Horizontal layout for desktop navbar
   * - 'sidebar': Vertical layout for mobile sidebar
   * @default 'navbar'
   */
  variant?: 'navbar' | 'sidebar';

  /**
   * Additional CSS class name
   */
  className?: string;

  /**
   * Callback when a navigation action occurs
   * Used by sidebar to close menu after navigation
   */
  onNavigate?: () => void;
}

export interface NavbarAuthReturn {
  /**
   * The component renders one of:
   * - Loading skeleton (when isLoading)
   * - Auth buttons (when !user)
   * - Profile icon (when user exists)
   */
  element: React.ReactElement;
}
```

### Usage Contract

```tsx
// Desktop navbar usage
<NavbarAuth variant="navbar" />

// Mobile sidebar usage
<NavbarAuth
  variant="sidebar"
  onNavigate={() => closeSidebar()}
/>
```

---

## 2. ProfileIcon Component

### File: `src/components/NavbarAuth/ProfileIcon.tsx`

```typescript
/**
 * @component ProfileIcon
 * @description Circular user avatar/icon with navigation to profile
 *
 * @behavior
 * - Click: Navigate to /profile
 * - Hover: Show tooltip with user name
 * - Display: User initials or generic icon
 */

export interface ProfileIconProps {
  /**
   * User's display name
   * Used for initials and tooltip
   */
  displayName?: string | null;

  /**
   * User's email
   * Fallback when displayName not available
   */
  email?: string | null;

  /**
   * Icon size variant
   * - small: 24px (mobile compact)
   * - medium: 32px (desktop navbar)
   * - large: 36px (mobile sidebar)
   * @default 'medium'
   */
  size?: 'small' | 'medium' | 'large';

  /**
   * Destination URL
   * @default '/profile'
   */
  href?: string;

  /**
   * Additional CSS class name
   */
  className?: string;
}
```

### Rendering Contract

```typescript
/**
 * ProfileIcon MUST render:
 * 1. A clickable element (Link or button)
 * 2. Either:
 *    a. User initials (first letter of displayName or email)
 *    b. Generic user SVG icon (if no name/email)
 * 3. Circular shape with theme-appropriate background
 */

// Visual specification
const PROFILE_ICON_SPEC = {
  shape: 'circle',
  background: 'var(--ifm-color-primary)',
  textColor: 'white',
  sizes: {
    small: { diameter: 24, fontSize: 12 },
    medium: { diameter: 32, fontSize: 14 },
    large: { diameter: 36, fontSize: 16 },
  },
};
```

---

## 3. FeatureCard Component

### File: `src/components/FeatureCard/index.tsx`

```typescript
/**
 * @component FeatureCard
 * @description Animated card for displaying feature highlights
 *
 * @behavior
 * - Hover: translateY(-4px), shadow increase
 * - Enter: Fade in from bottom
 * - Reduced motion: No animations
 */

export interface FeatureCardProps {
  /**
   * Card title text
   * @required
   */
  title: string;

  /**
   * Card description content
   * Can be string or React nodes
   * @required
   */
  description: React.ReactNode;

  /**
   * SVG icon component
   * Receives standard SVG props
   * @required
   */
  icon: React.ComponentType<React.SVGProps<SVGSVGElement>>;

  /**
   * Optional link URL
   * If provided, entire card is clickable
   */
  href?: string;

  /**
   * Animation delay in milliseconds
   * For staggered entrance animation
   * @default 0
   */
  animationDelay?: number;

  /**
   * Additional CSS class name
   */
  className?: string;
}
```

### Visual Contract

```typescript
/**
 * FeatureCard MUST render:
 * 1. Card container with:
 *    - Border radius: 16px
 *    - Padding: 24px
 *    - Background: theme-aware (white/dark)
 *    - Border: 1px solid theme-aware
 *    - Box shadow: subtle elevation
 *
 * 2. Icon section:
 *    - Centered above text
 *    - Max height: 200px
 *
 * 3. Text section:
 *    - Title: h3 level, centered
 *    - Description: paragraph, centered
 */

const FEATURE_CARD_SPEC = {
  container: {
    borderRadius: '16px',
    padding: '24px',
    background: {
      light: '#ffffff',
      dark: 'var(--ifm-background-surface-color)',
    },
    border: '1px solid var(--ifm-toc-border-color)',
    boxShadow: 'var(--ifm-global-shadow-md)',
  },
  icon: {
    maxHeight: '200px',
    marginBottom: '16px',
  },
  title: {
    fontSize: '1.25rem',
    fontWeight: 600,
    marginBottom: '8px',
  },
  description: {
    fontSize: '1rem',
    lineHeight: 1.6,
  },
};
```

### Animation Contract

```typescript
/**
 * Animation specifications
 */
const CARD_ANIMATIONS = {
  hover: {
    transform: 'translateY(-4px)',
    boxShadow: 'var(--ifm-global-shadow-tl)',
    transition: 'all 300ms ease-out',
  },
  enter: {
    keyframes: {
      from: { opacity: 0, transform: 'translateY(20px)' },
      to: { opacity: 1, transform: 'translateY(0)' },
    },
    duration: '400ms',
    easing: 'ease-out',
    fillMode: 'both',
  },
  reducedMotion: {
    animation: 'none',
    transition: 'none',
    transform: 'none',
  },
};
```

---

## 4. AuthButtons Component

### File: `src/components/NavbarAuth/AuthButtons.tsx`

```typescript
/**
 * @component AuthButtons
 * @description Sign Up and Sign In button pair
 *
 * @behavior
 * - Sign Up: Navigate to /auth?mode=signup
 * - Sign In: Navigate to /auth?mode=signin
 */

export interface AuthButtonsProps {
  /**
   * Layout direction
   * - 'horizontal': Side by side (navbar)
   * - 'vertical': Stacked (sidebar)
   * @default 'horizontal'
   */
  direction?: 'horizontal' | 'vertical';

  /**
   * Additional CSS class name
   */
  className?: string;

  /**
   * Callback after button click (before navigation)
   */
  onClick?: () => void;
}
```

### Button Specification

```typescript
/**
 * Button visual contract
 */
const AUTH_BUTTON_SPEC = {
  signUp: {
    label: 'Sign Up',
    href: '/auth?mode=signup',
    variant: 'secondary', // Outline style
    ariaLabel: 'Create a new account',
  },
  signIn: {
    label: 'Sign In',
    href: '/auth?mode=signin',
    variant: 'primary', // Filled style
    ariaLabel: 'Sign in to your account',
  },
  common: {
    minHeight: '44px',
    padding: '8px 16px',
    borderRadius: '8px',
    fontWeight: 500,
    transition: 'all 200ms ease',
  },
};
```

---

## 5. SidebarAuthSection Component

### File: `src/components/NavbarAuth/SidebarAuthSection.tsx`

```typescript
/**
 * @component SidebarAuthSection
 * @description Auth section for mobile sidebar
 *
 * @behavior
 * - Unauthenticated: Show auth buttons
 * - Authenticated: Show user info + sign out
 */

export interface SidebarAuthSectionProps {
  /**
   * Callback when user navigates away
   * Used to close sidebar
   */
  onNavigate?: () => void;

  /**
   * Additional CSS class name
   */
  className?: string;
}

export interface SidebarUserInfoProps {
  /**
   * User's display name
   */
  displayName: string;

  /**
   * User's email
   */
  email: string;

  /**
   * Sign out handler
   */
  onSignOut: () => void;
}
```

### Layout Contract

```typescript
/**
 * SidebarAuthSection layout
 */
const SIDEBAR_AUTH_SPEC = {
  container: {
    padding: '16px',
    borderBottom: '1px solid var(--ifm-toc-border-color)',
    marginBottom: '8px',
  },
  userInfo: {
    display: 'flex',
    alignItems: 'center',
    gap: '12px',
    marginBottom: '12px',
  },
  signOutButton: {
    width: '100%',
    textAlign: 'left',
    color: 'var(--ifm-color-danger)',
  },
};
```

---

## 6. Root Component Extension

### File: `src/theme/Root.tsx`

```typescript
/**
 * @component Root
 * @description Extended to wrap app with AuthProvider
 *
 * @existing Chatbot integration
 * @added AuthProvider wrapper
 */

export interface RootProps {
  children: React.ReactNode;
}

/**
 * Root component contract
 * MUST wrap children with:
 * 1. AuthProvider (for auth state)
 * 2. Chatbot (existing)
 */

// Implementation pattern
function Root({ children }: RootProps): React.ReactElement {
  return (
    <AuthProvider>
      <>
        {children}
        <Chatbot />
      </>
    </AuthProvider>
  );
}
```

---

## 7. CSS Contracts

### Sidebar Fix Contract

```css
/**
 * Sidebar background MUST be:
 * - Fully opaque (opacity: 1)
 * - Correct color per theme
 * - No backdrop-filter artifacts
 *
 * File: src/css/custom.css
 */

/* REQUIRED: Light mode explicit background */
.navbar-sidebar {
  background-color: #ffffff !important;
  opacity: 1 !important;
  backdrop-filter: none !important;
}

/* REQUIRED: Dark mode background */
[data-theme='dark'] .navbar-sidebar {
  background-color: var(--ifm-background-color) !important;
}

/* REQUIRED: Sidebar items background */
.navbar-sidebar__items {
  background-color: inherit;
}
```

### Card Animation Contract

```css
/**
 * Card animations MUST:
 * 1. Use CSS transitions (no JS)
 * 2. Respect prefers-reduced-motion
 * 3. Not cause layout shift
 *
 * File: src/components/FeatureCard/styles.module.css
 */

/* REQUIRED: Hover transition */
.card {
  transition: transform 300ms ease-out, box-shadow 300ms ease-out;
}

.card:hover {
  transform: translateY(-4px);
}

/* REQUIRED: Reduced motion */
@media (prefers-reduced-motion: reduce) {
  .card {
    transition: none;
  }
  .card:hover {
    transform: none;
  }
}
```

### Grid Layout Contract

```css
/**
 * Feature grid MUST:
 * 1. 3 columns on desktop (≥1200px)
 * 2. 2 columns on tablet (769-1199px)
 * 3. 1 column on mobile (≤768px)
 *
 * File: src/components/HomepageFeatures/styles.module.css
 */

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

## 8. Integration Points

### AuthProvider → NavbarAuth

```typescript
/**
 * Contract: AuthProvider MUST provide these values
 * to NavbarAuth via useAuth() hook
 */
interface AuthProviderContract {
  // State
  user: AuthUser | null;       // Current user or null
  profile: ProfileResponse | null; // Profile data or null
  isLoading: boolean;          // True during session check

  // Methods
  signOut: () => Promise<void>; // Sign out handler
}
```

### URL Parameters → Auth Page

```typescript
/**
 * Contract: Auth page MUST handle URL params
 */
interface AuthPageURLContract {
  // Read on mount
  mode: 'signin' | 'signup';

  // Behavior
  'mode=signup': 'Display SignupForm initially';
  'mode=signin': 'Display SigninForm initially';
  'no mode param': 'Default to signin';
}
```

### Navigation Behavior

```typescript
/**
 * Contract: Navigation behavior after auth actions
 */
interface NavigationContract {
  afterSignIn: '/profile';
  afterSignUp: '/profile';
  afterSignOut: '/';
  unauthAccessProfile: '/auth?mode=signin';
  authAccessAuthPage: '/profile';
}
```

---

## 9. Error Handling

### Component Error States

```typescript
/**
 * Error handling contract for auth components
 */
interface ErrorHandlingContract {
  // NavbarAuth
  authError: 'Render auth buttons (allow retry)';
  networkError: 'Render auth buttons (allow retry)';

  // ProfileIcon
  noUserData: 'Render generic icon';

  // FeatureCard
  missingIcon: 'Render placeholder or skip icon';
  invalidProps: 'Console warning, render fallback';
}
```

---

## 10. Testing Contracts

### Component Test Requirements

```typescript
/**
 * Each component MUST pass these tests
 */
interface TestContract {
  NavbarAuth: [
    'renders auth buttons when unauthenticated',
    'renders profile icon when authenticated',
    'renders skeleton when loading',
    'navigates to correct routes',
    'handles sign out in sidebar variant',
  ];

  FeatureCard: [
    'renders title and description',
    'renders icon',
    'applies hover styles',
    'respects animation delay',
    'handles reduced motion',
  ];

  SidebarCSS: [
    'sidebar opaque in light mode',
    'sidebar opaque in dark mode',
    'sidebar correct color after theme switch',
  ];
}
```

---

## Summary

| Component | Props Interface | File Location |
|-----------|----------------|---------------|
| NavbarAuth | NavbarAuthProps | `src/components/NavbarAuth/index.tsx` |
| ProfileIcon | ProfileIconProps | `src/components/NavbarAuth/ProfileIcon.tsx` |
| AuthButtons | AuthButtonsProps | `src/components/NavbarAuth/AuthButtons.tsx` |
| FeatureCard | FeatureCardProps | `src/components/FeatureCard/index.tsx` |
| SidebarAuthSection | SidebarAuthSectionProps | `src/components/NavbarAuth/SidebarAuthSection.tsx` |
| Root | RootProps | `src/theme/Root.tsx` |
