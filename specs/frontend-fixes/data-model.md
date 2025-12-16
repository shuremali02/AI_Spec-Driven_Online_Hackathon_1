# Data Model: Frontend Fixes & UI Enhancements

**Feature**: frontend-fixes
**Date**: 2025-12-16
**Status**: Complete

---

## Overview

This document defines the data structures, state management, and component interfaces for the frontend fixes feature. Since this is a frontend-only feature, the data model focuses on React component props, state shapes, and TypeScript interfaces.

---

## 1. Authentication State

### AuthState Interface

```typescript
/**
 * Core authentication state shape
 * Source: src/components/Auth/types.ts (existing)
 */
interface AuthState {
  user: AuthUser | null;
  profile: ProfileResponse | null;
  isLoading: boolean;
  error: string | null;
}

interface AuthUser {
  id: string;
  email: string;
  name: string | null;
}

interface ProfileResponse {
  id: string;
  userId: string;
  displayName: string;
  softwareBackground: string;
  hardwareBackground: string;
  experienceLevel: string;
  createdAt: string;
  updatedAt: string;
}
```

### AuthContextValue Interface

```typescript
/**
 * Full auth context value including methods
 * Source: src/components/Auth/AuthProvider.tsx (existing)
 */
interface AuthContextValue extends AuthState {
  signOut: () => Promise<void>;
  refreshProfile: () => Promise<void>;
  setProfile: (profile: ProfileResponse | null) => void;
}
```

---

## 2. Navbar Auth Component

### NavbarAuthProps

```typescript
/**
 * Props for NavbarAuth component
 * Location: src/components/NavbarAuth/index.tsx
 */
interface NavbarAuthProps {
  /** Position context: desktop navbar or mobile sidebar */
  variant?: 'navbar' | 'sidebar';
  /** Custom class name for styling */
  className?: string;
}
```

### NavbarAuthState

```typescript
/**
 * Internal state (derived from AuthContext)
 */
type NavbarAuthState = {
  isAuthenticated: boolean;
  isLoading: boolean;
  userDisplayName: string | null;
  userEmail: string | null;
};
```

### AuthButtonConfig

```typescript
/**
 * Configuration for auth buttons
 */
interface AuthButtonConfig {
  label: string;
  href: string;
  variant: 'primary' | 'secondary';
  icon?: React.ReactNode;
}

const AUTH_BUTTONS: AuthButtonConfig[] = [
  {
    label: 'Sign Up',
    href: '/auth?mode=signup',
    variant: 'secondary',
  },
  {
    label: 'Sign In',
    href: '/auth?mode=signin',
    variant: 'primary',
  },
];
```

---

## 3. Feature Card Component

### FeatureCardProps

```typescript
/**
 * Props for FeatureCard component
 * Location: src/components/FeatureCard/index.tsx
 */
interface FeatureCardProps {
  /** Card title */
  title: string;
  /** Card description text */
  description: React.ReactNode;
  /** SVG icon component */
  icon: React.ComponentType<React.SVGProps<SVGSVGElement>>;
  /** Optional link for the card */
  href?: string;
  /** Animation delay in ms (for stagger effect) */
  animationDelay?: number;
  /** Custom class name */
  className?: string;
}
```

### FeatureItem (Existing Pattern)

```typescript
/**
 * Existing feature item structure from HomepageFeatures
 * Source: src/components/HomepageFeatures/index.tsx
 */
interface FeatureItem {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: React.ReactNode;
}
```

### FeatureGridProps

```typescript
/**
 * Props for feature grid container
 */
interface FeatureGridProps {
  items: FeatureItem[];
  columns?: 1 | 2 | 3;
  className?: string;
}
```

---

## 4. Profile Icon Component

### ProfileIconProps

```typescript
/**
 * Props for ProfileIcon component
 * Location: src/components/NavbarAuth/ProfileIcon.tsx
 */
interface ProfileIconProps {
  /** User display name (for avatar initials or tooltip) */
  displayName?: string | null;
  /** User email (fallback for display) */
  email?: string | null;
  /** Size variant */
  size?: 'small' | 'medium' | 'large';
  /** Click handler */
  onClick?: () => void;
  /** Link destination */
  href?: string;
  /** Custom class name */
  className?: string;
}
```

### Size Mapping

```typescript
const ICON_SIZES = {
  small: 24,
  medium: 32,
  large: 36,
} as const;
```

---

## 5. Auth Page State

### AuthPageMode

```typescript
/**
 * Auth page mode derived from URL params
 */
type AuthMode = 'signin' | 'signup';

interface AuthPageState {
  mode: AuthMode;
  isRedirecting: boolean;
}
```

### URL Parameter Extraction

```typescript
/**
 * Extract auth mode from URL
 */
function getAuthModeFromUrl(): AuthMode {
  if (typeof window === 'undefined') return 'signin';
  const params = new URLSearchParams(window.location.search);
  return params.get('mode') === 'signup' ? 'signup' : 'signin';
}
```

---

## 6. Mobile Sidebar Auth Section

### SidebarAuthSectionProps

```typescript
/**
 * Props for sidebar auth section
 */
interface SidebarAuthSectionProps {
  /** Whether sidebar is expanded */
  isExpanded?: boolean;
  /** Callback when navigation occurs (to close sidebar) */
  onNavigate?: () => void;
}
```

### SidebarUserInfo

```typescript
/**
 * User info displayed in sidebar
 */
interface SidebarUserInfo {
  displayName: string;
  email: string;
  avatarUrl?: string;
}
```

---

## 7. CSS Module Types

### NavbarAuth Styles

```typescript
/**
 * CSS Module type for NavbarAuth
 * Location: src/components/NavbarAuth/styles.module.css
 */
interface NavbarAuthStyles {
  authContainer: string;
  authButton: string;
  authButtonPrimary: string;
  authButtonSecondary: string;
  profileIcon: string;
  profileIconImage: string;
  profileIconInitials: string;
  sidebarAuthSection: string;
  sidebarUserInfo: string;
  sidebarSignOutButton: string;
  skeleton: string;
}
```

### FeatureCard Styles

```typescript
/**
 * CSS Module type for FeatureCard
 * Location: src/components/FeatureCard/styles.module.css
 */
interface FeatureCardStyles {
  card: string;
  cardLink: string;
  iconContainer: string;
  icon: string;
  title: string;
  description: string;
  cardAnimated: string;
  cardHover: string;
}
```

### FeatureGrid Styles

```typescript
/**
 * CSS Module type for FeatureGrid
 * Location: src/components/HomepageFeatures/styles.module.css (updated)
 */
interface FeatureGridStyles {
  features: string;
  featureGrid: string;
  featureGridSingle: string;
  featureGridDouble: string;
  featureGridTriple: string;
}
```

---

## 8. Route Configuration

### Route Definitions

```typescript
/**
 * Route configuration for auth flows
 */
const AUTH_ROUTES = {
  auth: '/auth',
  authSignup: '/auth?mode=signup',
  authSignin: '/auth?mode=signin',
  profile: '/profile',
} as const;

type AuthRoute = typeof AUTH_ROUTES[keyof typeof AUTH_ROUTES];
```

### Navigation Guards

```typescript
/**
 * Route guard conditions
 */
interface RouteGuard {
  path: string;
  requiresAuth: boolean;
  redirectTo: string;
}

const ROUTE_GUARDS: RouteGuard[] = [
  {
    path: '/profile',
    requiresAuth: true,
    redirectTo: '/auth?mode=signin',
  },
];
```

---

## 9. Animation Configuration

### Animation Timing

```typescript
/**
 * Animation timing constants
 */
const ANIMATION_CONFIG = {
  card: {
    hover: {
      duration: 300,
      easing: 'ease-out',
      transform: 'translateY(-4px)',
    },
    enter: {
      duration: 400,
      easing: 'ease-out',
      staggerDelay: 100,
    },
  },
  button: {
    hover: {
      duration: 200,
      easing: 'ease',
      transform: 'translateY(-2px)',
    },
  },
} as const;
```

### CSS Custom Properties

```typescript
/**
 * CSS custom properties for theming
 * Used in custom.css
 */
const CSS_VARIABLES = {
  // Card styling
  '--feature-card-bg': 'var(--ifm-background-surface-color)',
  '--feature-card-border': 'var(--ifm-toc-border-color)',
  '--feature-card-shadow': 'var(--ifm-global-shadow-md)',
  '--feature-card-radius': '16px',
  '--feature-card-padding': '24px',

  // Sidebar fix
  '--sidebar-bg-light': '#ffffff',
  '--sidebar-bg-dark': 'var(--ifm-background-color)',
} as const;
```

---

## 10. State Transitions

### Auth State Machine

```
┌─────────────┐     loadSession     ┌─────────────┐
│   Initial   │ ─────────────────▶ │   Loading   │
└─────────────┘                     └─────────────┘
                                          │
                        ┌─────────────────┴─────────────────┐
                        ▼                                   ▼
                ┌─────────────┐                     ┌─────────────┐
                │ Unauthenticated │                 │ Authenticated │
                └─────────────┘                     └─────────────┘
                        │                                   │
                        │ signIn                    signOut │
                        └──────────▶ ◀──────────────────────┘
```

### Navbar Render States

```
┌─────────────────────────────────────────────────────────┐
│                    NavbarAuth Component                  │
├─────────────────────────────────────────────────────────┤
│  State: Loading                                          │
│  └─▶ Render: Skeleton placeholder                        │
├─────────────────────────────────────────────────────────┤
│  State: Unauthenticated                                  │
│  └─▶ Render: [Sign Up] [Sign In] buttons                │
├─────────────────────────────────────────────────────────┤
│  State: Authenticated                                    │
│  └─▶ Render: Profile icon with user initials            │
└─────────────────────────────────────────────────────────┘
```

---

## 11. Validation Rules

### Form Validation (Existing)

The auth forms already have validation. No changes needed for this feature.

### UI Validation

```typescript
/**
 * Validate feature card props
 */
function validateFeatureCardProps(props: FeatureCardProps): boolean {
  return (
    typeof props.title === 'string' &&
    props.title.length > 0 &&
    props.icon !== undefined
  );
}
```

---

## Entity Relationship Diagram

```
┌─────────────────┐
│   AuthProvider  │
│   (Context)     │
└────────┬────────┘
         │ provides
         ▼
┌─────────────────┐       ┌─────────────────┐
│   NavbarAuth    │◀─────▶│  ProfileIcon    │
│   Component     │       │   Component     │
└────────┬────────┘       └─────────────────┘
         │ uses
         ▼
┌─────────────────┐
│   AuthButtons   │
│   (Sign Up/In)  │
└─────────────────┘

┌─────────────────┐       ┌─────────────────┐
│ HomepageFeatures│──────▶│   FeatureCard   │
│   Container     │ maps  │    Component    │
└─────────────────┘       └─────────────────┘
```

---

## Summary

| Entity | Type | Location | Purpose |
|--------|------|----------|---------|
| AuthState | Interface | types.ts | Core auth shape |
| NavbarAuthProps | Interface | NavbarAuth/index.tsx | Component props |
| FeatureCardProps | Interface | FeatureCard/index.tsx | Card props |
| ProfileIconProps | Interface | NavbarAuth/ProfileIcon.tsx | Icon props |
| AuthMode | Type | auth.tsx | URL param type |
| AUTH_ROUTES | Const | routes.ts | Route definitions |
| ANIMATION_CONFIG | Const | constants.ts | Animation timing |
