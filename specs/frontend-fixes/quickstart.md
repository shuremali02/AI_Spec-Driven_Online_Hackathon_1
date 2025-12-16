# Quickstart: Frontend Fixes & UI Enhancements

**Feature**: frontend-fixes
**Date**: 2025-12-16
**Estimated Time**: 4-6 hours

---

## Prerequisites

Before implementing this feature, ensure:

- [ ] Node.js 18+ installed
- [ ] Docusaurus dev server can start (`npm start` in book-write/)
- [ ] Better-Auth backend is running (for auth testing)
- [ ] Existing auth pages work (`/auth`, `/profile`)

---

## Implementation Order

```
1. Sidebar CSS Fix (Critical Bug)     [30 min]
2. Root AuthProvider Extension        [15 min]
3. NavbarAuth Component              [1-2 hrs]
4. Auth Page URL Parameter           [30 min]
5. FeatureCard Component             [1-2 hrs]
6. Integration Testing               [30 min]
```

---

## Step 1: Fix Sidebar CSS (Critical Bug)

**File**: `book-write/src/css/custom.css`

Add/update at the end of the file:

```css
/* =============================================
   Sidebar Transparency Fix (Critical)
   ============================================= */

/* Light mode - explicit solid background */
.navbar-sidebar {
  background-color: #ffffff !important;
}

/* Dark mode - theme background */
[data-theme='dark'] .navbar-sidebar {
  background-color: var(--ifm-background-color) !important;
}

/* Remove any backdrop-filter that may cause issues */
.navbar-sidebar,
.navbar-sidebar__brand,
.navbar-sidebar__items {
  backdrop-filter: none !important;
  -webkit-backdrop-filter: none !important;
}

/* Ensure full opacity */
.navbar-sidebar {
  opacity: 1 !important;
}
```

**Test**:
1. Open site in mobile viewport (≤996px)
2. Click hamburger menu
3. Verify sidebar has solid background in light mode
4. Toggle to dark mode, verify solid background
5. Toggle back to light mode, no transparency

---

## Step 2: Extend Root with AuthProvider

**File**: `book-write/src/theme/Root.tsx`

Update existing Root component:

```tsx
import React from 'react';
import { AuthProvider } from '@site/src/components/Auth/AuthProvider';
import Chatbot from '@site/src/components/Chatbot';

export default function Root({ children }: { children: React.ReactNode }) {
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

**Test**:
- Site still loads without errors
- Chatbot still works
- No console errors about AuthProvider

---

## Step 3: Create NavbarAuth Component

### 3.1 Create Component Directory

```bash
mkdir -p book-write/src/components/NavbarAuth
```

### 3.2 Create Main Component

**File**: `book-write/src/components/NavbarAuth/index.tsx`

```tsx
import React from 'react';
import Link from '@docusaurus/Link';
import { useAuth } from '@site/src/components/Auth/AuthProvider';
import styles from './styles.module.css';
import clsx from 'clsx';

interface NavbarAuthProps {
  variant?: 'navbar' | 'sidebar';
  className?: string;
  onNavigate?: () => void;
}

export default function NavbarAuth({
  variant = 'navbar',
  className,
  onNavigate,
}: NavbarAuthProps): React.ReactElement {
  const { user, profile, isLoading, signOut } = useAuth();

  const handleNavigation = () => {
    onNavigate?.();
  };

  const handleSignOut = async () => {
    await signOut();
    onNavigate?.();
  };

  // Loading state
  if (isLoading) {
    return (
      <div className={clsx(styles.authContainer, styles[variant], className)}>
        <div className={styles.skeleton} />
      </div>
    );
  }

  // Authenticated state
  if (user) {
    const displayName = profile?.displayName || user.name || user.email || 'User';
    const initials = displayName.charAt(0).toUpperCase();

    if (variant === 'sidebar') {
      return (
        <div className={clsx(styles.authContainer, styles.sidebar, className)}>
          <div className={styles.sidebarUserInfo}>
            <div className={styles.profileIcon}>{initials}</div>
            <div className={styles.userDetails}>
              <span className={styles.userName}>{displayName}</span>
              <span className={styles.userEmail}>{user.email}</span>
            </div>
          </div>
          <Link
            to="/profile"
            className={styles.sidebarLink}
            onClick={handleNavigation}
          >
            View Profile
          </Link>
          <button
            onClick={handleSignOut}
            className={styles.signOutButton}
            type="button"
          >
            Sign Out
          </button>
        </div>
      );
    }

    // Navbar variant
    return (
      <div className={clsx(styles.authContainer, styles.navbar, className)}>
        <Link
          to="/profile"
          className={styles.profileIconLink}
          aria-label={`Profile for ${displayName}`}
          onClick={handleNavigation}
        >
          <span className={styles.profileIcon}>{initials}</span>
        </Link>
      </div>
    );
  }

  // Unauthenticated state
  return (
    <div className={clsx(styles.authContainer, styles[variant], className)}>
      <Link
        to="/auth?mode=signup"
        className={clsx(styles.authButton, styles.secondary)}
        onClick={handleNavigation}
      >
        Sign Up
      </Link>
      <Link
        to="/auth?mode=signin"
        className={clsx(styles.authButton, styles.primary)}
        onClick={handleNavigation}
      >
        Sign In
      </Link>
    </div>
  );
}
```

### 3.3 Create Styles

**File**: `book-write/src/components/NavbarAuth/styles.module.css`

```css
/* Container */
.authContainer {
  display: flex;
  align-items: center;
  gap: 8px;
}

.authContainer.navbar {
  flex-direction: row;
}

.authContainer.sidebar {
  flex-direction: column;
  align-items: stretch;
  padding: 16px;
  border-bottom: 1px solid var(--ifm-toc-border-color);
  gap: 12px;
}

/* Auth Buttons */
.authButton {
  display: inline-flex;
  align-items: center;
  justify-content: center;
  padding: 8px 16px;
  border-radius: 8px;
  font-weight: 500;
  font-size: 0.875rem;
  text-decoration: none;
  transition: all 200ms ease;
  min-height: 36px;
}

.authButton.primary {
  background: rgba(255, 255, 255, 0.15);
  color: white;
  border: 1px solid rgba(255, 255, 255, 0.3);
}

.authButton.primary:hover {
  background: rgba(255, 255, 255, 0.25);
  color: white;
  text-decoration: none;
}

.authButton.secondary {
  background: transparent;
  color: rgba(255, 255, 255, 0.9);
  border: 1px solid transparent;
}

.authButton.secondary:hover {
  color: white;
  text-decoration: none;
}

/* Sidebar variant button styles */
.sidebar .authButton {
  width: 100%;
  justify-content: center;
}

.sidebar .authButton.primary {
  background: var(--ifm-color-primary);
  color: white;
  border-color: var(--ifm-color-primary);
}

.sidebar .authButton.secondary {
  background: transparent;
  color: var(--ifm-color-primary);
  border: 1px solid var(--ifm-color-primary);
}

/* Profile Icon */
.profileIcon {
  display: flex;
  align-items: center;
  justify-content: center;
  width: 32px;
  height: 32px;
  border-radius: 50%;
  background: rgba(255, 255, 255, 0.2);
  color: white;
  font-weight: 600;
  font-size: 14px;
}

.profileIconLink {
  display: flex;
  text-decoration: none;
}

.profileIconLink:hover .profileIcon {
  background: rgba(255, 255, 255, 0.3);
}

/* Sidebar user info */
.sidebarUserInfo {
  display: flex;
  align-items: center;
  gap: 12px;
}

.sidebar .profileIcon {
  background: var(--ifm-color-primary);
  width: 40px;
  height: 40px;
  font-size: 16px;
}

.userDetails {
  display: flex;
  flex-direction: column;
}

.userName {
  font-weight: 600;
  font-size: 0.9rem;
}

.userEmail {
  font-size: 0.75rem;
  color: var(--ifm-color-secondary-darkest);
}

.sidebarLink {
  padding: 8px 12px;
  border-radius: 6px;
  text-decoration: none;
  color: var(--ifm-font-color-base);
}

.sidebarLink:hover {
  background: var(--ifm-color-emphasis-100);
  text-decoration: none;
}

.signOutButton {
  padding: 8px 12px;
  border-radius: 6px;
  border: none;
  background: transparent;
  color: var(--ifm-color-danger);
  text-align: left;
  cursor: pointer;
  font-size: 0.875rem;
}

.signOutButton:hover {
  background: var(--ifm-color-danger-contrast-background);
}

/* Loading skeleton */
.skeleton {
  width: 120px;
  height: 36px;
  background: rgba(255, 255, 255, 0.1);
  border-radius: 8px;
  animation: pulse 1.5s ease-in-out infinite;
}

@keyframes pulse {
  0%, 100% { opacity: 0.5; }
  50% { opacity: 0.8; }
}

/* Focus states */
.authButton:focus-visible,
.profileIconLink:focus-visible,
.sidebarLink:focus-visible,
.signOutButton:focus-visible {
  outline: 2px solid var(--ifm-color-primary);
  outline-offset: 2px;
}

/* Touch targets */
@media (max-width: 996px) {
  .authButton {
    min-height: 44px;
    padding: 10px 20px;
  }

  .profileIcon {
    width: 36px;
    height: 36px;
  }
}
```

### 3.4 Integrate into Navbar

**Option A**: Add to `docusaurus.config.ts` (simpler, limited)

```typescript
// In navbar.items array, add:
{
  type: 'html',
  position: 'right',
  value: '<div id="navbar-auth-root"></div>',
},
```

Then render NavbarAuth into that div from Root.tsx.

**Option B**: Swizzle Navbar (recommended for full control)

```bash
cd book-write
npm run swizzle @docusaurus/theme-classic Navbar/Content -- --wrap --typescript
```

Edit the swizzled component to include NavbarAuth.

---

## Step 4: Add URL Parameter Handling to Auth Page

**File**: `book-write/src/pages/auth.tsx`

Update the component to read URL params:

```tsx
import React, { useState, useEffect } from 'react';
// ... existing imports

type AuthMode = 'signin' | 'signup';

function AuthContent(): React.ReactElement {
  // Read initial mode from URL
  const getInitialMode = (): AuthMode => {
    if (typeof window === 'undefined') return 'signin';
    const params = new URLSearchParams(window.location.search);
    return params.get('mode') === 'signup' ? 'signup' : 'signin';
  };

  const [mode, setMode] = useState<AuthMode>(getInitialMode);
  const { user, profile, isLoading } = useAuth();

  // Update mode when URL changes
  useEffect(() => {
    const handlePopState = () => {
      setMode(getInitialMode());
    };
    window.addEventListener('popstate', handlePopState);
    return () => window.removeEventListener('popstate', handlePopState);
  }, []);

  // ... rest of existing component
}
```

**Test**:
- Visit `/auth?mode=signup` → Shows signup form
- Visit `/auth?mode=signin` → Shows signin form
- Visit `/auth` → Shows signin form (default)

---

## Step 5: Create FeatureCard Component

### 5.1 Create Component Directory

```bash
mkdir -p book-write/src/components/FeatureCard
```

### 5.2 Create Component

**File**: `book-write/src/components/FeatureCard/index.tsx`

```tsx
import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

interface FeatureCardProps {
  title: string;
  description: React.ReactNode;
  Svg: React.ComponentType<React.SVGProps<SVGSVGElement>>;
  animationDelay?: number;
  className?: string;
}

export default function FeatureCard({
  title,
  description,
  Svg,
  animationDelay = 0,
  className,
}: FeatureCardProps): React.ReactElement {
  return (
    <article
      className={clsx(styles.card, className)}
      style={{ animationDelay: `${animationDelay}ms` }}
    >
      <div className={styles.iconContainer}>
        <Svg className={styles.icon} role="img" aria-hidden="true" />
      </div>
      <div className={styles.content}>
        <Heading as="h3" className={styles.title}>
          {title}
        </Heading>
        <p className={styles.description}>{description}</p>
      </div>
    </article>
  );
}
```

### 5.3 Create Styles

**File**: `book-write/src/components/FeatureCard/styles.module.css`

```css
.card {
  display: flex;
  flex-direction: column;
  align-items: center;
  text-align: center;
  padding: 24px;
  background: var(--ifm-background-surface-color);
  border: 1px solid var(--ifm-toc-border-color);
  border-radius: 16px;
  box-shadow: var(--ifm-global-shadow-md);
  transition: transform 300ms ease-out, box-shadow 300ms ease-out;
  animation: fadeInUp 400ms ease-out both;
}

.card:hover {
  transform: translateY(-4px);
  box-shadow: var(--ifm-global-shadow-tl);
}

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

/* Respect reduced motion preference */
@media (prefers-reduced-motion: reduce) {
  .card {
    animation: none;
    transition: none;
  }
  .card:hover {
    transform: none;
  }
}

.iconContainer {
  display: flex;
  align-items: center;
  justify-content: center;
  margin-bottom: 16px;
}

.icon {
  height: 150px;
  width: 150px;
}

.content {
  flex: 1;
}

.title {
  font-size: 1.25rem;
  font-weight: 600;
  margin-bottom: 8px;
  color: var(--ifm-heading-color);
}

.description {
  font-size: 1rem;
  line-height: 1.6;
  color: var(--ifm-font-color-base);
  margin: 0;
}

/* Dark mode adjustments */
[data-theme='dark'] .card {
  background: var(--ifm-background-surface-color);
}
```

### 5.4 Update HomepageFeatures

**File**: `book-write/src/components/HomepageFeatures/index.tsx`

```tsx
import type { ReactNode } from 'react';
import FeatureCard from '@site/src/components/FeatureCard';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI & Embodied Intelligence',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Learn how artificial intelligence transcends the digital realm to
        inhabit physical bodies through sophisticated humanoid robots.
      </>
    ),
  },
  {
    title: 'ROS 2 Fundamentals',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Master ROS 2 middleware for robust robot communication and control using
        Python and rclpy.
      </>
    ),
  },
  {
    title: 'Complete Learning Path',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        13-week structured course from ROS 2 basics to Vision-Language-Action
        (VLA) systems for conversational robots.
      </>
    ),
  },
];

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.featureGrid}>
          {FeatureList.map((props, idx) => (
            <FeatureCard
              key={props.title}
              title={props.title}
              description={props.description}
              Svg={props.Svg}
              animationDelay={idx * 100}
            />
          ))}
        </div>
      </div>
    </section>
  );
}
```

### 5.5 Update HomepageFeatures Styles

**File**: `book-write/src/components/HomepageFeatures/styles.module.css`

```css
.features {
  padding: 4rem 0;
}

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

  .features {
    padding: 2rem 0;
  }
}
```

---

## Step 6: Testing Checklist

### 6.1 Sidebar CSS Fix

- [ ] Light mode: Solid white background
- [ ] Dark mode: Solid dark background
- [ ] Theme switch: No transparency flicker
- [ ] Scroll content behind: No see-through

### 6.2 Navbar Auth

- [ ] Unauthenticated: See Sign Up + Sign In
- [ ] Click Sign Up: Go to `/auth?mode=signup`
- [ ] Click Sign In: Go to `/auth?mode=signin`
- [ ] Authenticated: See profile icon
- [ ] Click profile icon: Go to `/profile`

### 6.3 Mobile Sidebar Auth

- [ ] Unauthenticated: Auth buttons in sidebar
- [ ] Authenticated: User info + Sign Out in sidebar
- [ ] Sign Out works from sidebar

### 6.4 Feature Cards

- [ ] Three cards display correctly
- [ ] Hover animation works
- [ ] Entrance animation works
- [ ] Reduced motion: No animations
- [ ] Responsive: 3→2→1 columns

### 6.5 General

- [ ] No console errors
- [ ] No layout shift on load
- [ ] Focus states visible
- [ ] Touch targets 44px minimum

---

## Troubleshooting

### Issue: Auth buttons not showing

**Cause**: NavbarAuth not integrated into navbar

**Fix**: Either swizzle navbar or render into custom HTML item

### Issue: Sidebar still transparent

**Cause**: CSS specificity conflict

**Fix**: Add `!important` to background-color rules

### Issue: Cards not animating

**Cause**: prefers-reduced-motion enabled

**Fix**: Check system settings, or test with DevTools override

### Issue: Profile redirecting too early

**Cause**: isLoading not checked before redirect

**Fix**: Ensure `if (isLoading) return <Loading />` comes first

---

## Next Steps

After implementation:

1. Run `npm run build` in book-write/ to verify no build errors
2. Test on real mobile device if possible
3. Create pull request with screenshots
4. Run `/sp.tasks` to generate task breakdown for tracking
