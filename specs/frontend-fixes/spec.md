# Frontend Fixes & UI Enhancements Specification

---
id: frontend-fixes
title: Frontend Fixes & UI Enhancements
version: 1.0.0
status: draft
priority: high
framework: Docusaurus
created: 2025-12-16
---

## 1. Overview

This specification defines frontend fixes and UI enhancements for the Physical AI & Humanoid Robotics Docusaurus site. The scope includes navbar authentication controls, routing configuration, sidebar behavior, CSS transparency fixes, and landing page content restructure.

### 1.1 Scope

**In Scope:**
- Navbar authentication button integration (Signup/Signin)
- Profile icon for authenticated users
- Route definitions for auth flows
- Sidebar authentication controls (mobile)
- Sidebar CSS transparency fix (light/dark mode)
- Landing page content card conversion
- Responsive design across all breakpoints
- CSS-based animations

**Out of Scope:**
- Backend authentication logic
- API endpoints
- Database schema changes
- Third-party animation libraries

### 1.2 Dependencies

- Existing AuthProvider component (`src/components/Auth/AuthProvider.tsx`)
- Existing auth pages (`src/pages/auth.tsx`, `src/pages/profile.tsx`)
- Docusaurus theme swizzling system
- Custom CSS (`src/css/custom.css`)

---

## 2. Navbar Authentication Controls

### 2.1 Requirements

#### 2.1.1 Unauthenticated State

| Element | Position | Visibility | Behavior |
|---------|----------|------------|----------|
| Signup Button | Right side, before GitHub link | Visible when NOT authenticated | Navigate to `/auth` with signup mode |
| Signin Button | Right side, after Signup | Visible when NOT authenticated | Navigate to `/auth` with signin mode |

#### 2.1.2 Authenticated State

| Element | Position | Visibility | Behavior |
|---------|----------|------------|----------|
| Profile Icon | Right side, before GitHub link | Visible when authenticated | Navigate to `/profile` |

### 2.2 Desktop Behavior

- Signup and Signin buttons render as styled text links or buttons
- Buttons align horizontally in the navbar right section
- Profile icon displays as a circular avatar or user icon
- Profile icon includes visual indicator of authenticated state
- Hover states apply to all interactive elements

### 2.3 Mobile Behavior

- Signup and Signin buttons collapse into the mobile menu
- Profile icon displays in the mobile navbar header area
- Touch targets meet minimum 44x44px accessibility requirement

### 2.4 Button Specifications

**Signup Button:**
- Label: "Sign Up"
- Style: Secondary button (outline variant)
- Route: `/auth?mode=signup`

**Signin Button:**
- Label: "Sign In"
- Style: Primary button (filled variant)
- Route: `/auth?mode=signin`

**Profile Icon:**
- Icon: User initials in circular avatar (first letter of display name or email)
- Fallback: Generic user SVG icon if no name/email available
- Size: 32px (desktop), 36px (mobile)
- Shape: Circular with theme-appropriate background color
- Route: `/profile`

---

## 3. Routing Requirements

### 3.1 Route Definitions

| Route | Page | Auth Required | Description |
|-------|------|---------------|-------------|
| `/auth` | auth.tsx | No | Authentication page (signup/signin) |
| `/auth?mode=signup` | auth.tsx | No | Direct link to signup form |
| `/auth?mode=signin` | auth.tsx | No | Direct link to signin form |
| `/profile` | profile.tsx | Yes | User profile page |

### 3.2 Navigation Behavior

**Unauthenticated User:**
- Can access `/auth` freely
- Redirect to `/auth` when accessing `/profile`

**Authenticated User:**
- Redirect from `/auth` to `/profile`
- Can access `/profile` directly

### 3.3 URL Parameter Handling

The auth page MUST read the `mode` query parameter:
- `?mode=signup` - Display signup form initially
- `?mode=signin` - Display signin form initially
- No parameter - Default to signin form

---

## 4. Sidebar Behavior

### 4.1 Desktop Sidebar

- Standard Docusaurus docs sidebar behavior
- No auth controls in desktop sidebar
- Auth controls remain in navbar

### 4.2 Mobile Sidebar

#### 4.2.1 Unauthenticated State

| Element | Position | Behavior |
|---------|----------|----------|
| Signup Link | Top of sidebar, below site title | Navigate to `/auth?mode=signup` |
| Signin Link | Top of sidebar, after Signup | Navigate to `/auth?mode=signin` |

#### 4.2.2 Authenticated State

| Element | Position | Behavior |
|---------|----------|----------|
| Profile Section | Top of sidebar, below site title | Display user info and profile link |
| Sign Out Link | Within profile section | Execute sign out action |

### 4.3 Mobile Sidebar Layout

```
┌─────────────────────────────┐
│ [X] Site Title              │
├─────────────────────────────┤
│ ┌─────────────────────────┐ │
│ │ Auth Section            │ │  <- Unauthenticated: Signup/Signin
│ │ (or Profile Section)    │ │  <- Authenticated: User Profile
│ └─────────────────────────┘ │
├─────────────────────────────┤
│ Navigation Links            │
│ - Book                      │
│ - Docs                      │
│ - ...                       │
└─────────────────────────────┘
```

### 4.4 Visual Hierarchy

- Auth/Profile section visually separated from navigation
- Use subtle divider or spacing between sections
- Profile section includes user display name or email
- Touch-friendly link sizing (minimum 44px height)

---

## 5. Sidebar CSS Fix (Critical Bug)

### 5.1 Problem Statement

The mobile sidebar exhibits transparency issues:
- Correct appearance in dark mode
- Transparent or semi-transparent background in light mode
- Inconsistent rendering across theme switches

### 5.2 Root Cause

The sidebar background relies on CSS variables that may not resolve correctly in light mode, or opacity/backdrop-filter rules conflict with theme styles.

### 5.3 Required Fix

#### 5.3.1 Light Mode

```
Background: Solid opaque color (white or light gray)
No transparency
No backdrop-filter artifacts
```

#### 5.3.2 Dark Mode

```
Background: Solid opaque color (dark gray or slate)
No transparency
Consistent with existing dark theme
```

### 5.4 CSS Requirements

| Property | Light Mode Value | Dark Mode Value |
|----------|------------------|-----------------|
| background-color | `#ffffff` or `var(--ifm-background-color)` | `var(--ifm-background-color)` |
| opacity | 1 (no transparency) | 1 (no transparency) |
| backdrop-filter | none | none |

### 5.5 Affected Elements

- `.navbar-sidebar`
- `.navbar-sidebar__backdrop`
- `.navbar-sidebar__brand`
- `.navbar-sidebar__items`

### 5.6 Testing Criteria

- [ ] Sidebar background is fully opaque in light mode
- [ ] Sidebar background is fully opaque in dark mode
- [ ] Theme toggle does not cause transparency flicker
- [ ] Sidebar is readable against all page backgrounds
- [ ] Sidebar renders correctly on initial page load
- [ ] Sidebar renders correctly after navigation

---

## 6. Landing Page Content Restructure

### 6.1 Current Structure

The landing page contains:
1. Hero section with title and CTA
2. Welcome section with paragraphs
3. HomepageFeatures section with three feature items

### 6.2 Required Changes

#### 6.2.1 Welcome Section

**Current:** Two paragraphs of text within a centered container

**Required:** Convert paragraphs into styled introductory text (no change to structure, but ensure visual consistency with new cards below)

#### 6.2.2 Features Section (Three Cards)

**Current:** Three feature items displayed as simple text with SVG icons

**Required:** Convert to three distinct, animated cards

### 6.3 Card Specifications

#### 6.3.1 Card Layout

```
┌────────────────────────────────────┐
│           [Icon/Image]             │
│                                    │
│         Card Title                 │
│                                    │
│    Card description text that      │
│    explains the feature clearly    │
│    and concisely.                  │
│                                    │
└────────────────────────────────────┘
```

#### 6.3.2 Card Content

**Card 1: Physical AI & Embodied Intelligence**
- Icon: Existing SVG or appropriate icon
- Title: "Physical AI & Embodied Intelligence"
- Description: Content about AI inhabiting physical bodies through humanoid robots

**Card 2: ROS 2 Fundamentals**
- Icon: Existing SVG or appropriate icon
- Title: "ROS 2 Fundamentals"
- Description: Content about ROS 2 middleware for robot communication

**Card 3: Complete Learning Path**
- Icon: Existing SVG or appropriate icon
- Title: "Complete Learning Path"
- Description: Content about the 13-week course structure

#### 6.3.3 Card Styling

| Property | Value |
|----------|-------|
| Border Radius | 16px |
| Background (Light) | #ffffff |
| Background (Dark) | #1e293b |
| Border | 1px solid with theme-appropriate color |
| Box Shadow | Subtle shadow (elevation effect) |
| Padding | 24px |
| Min Height | Consistent across all cards |

### 6.4 Grid Layout

#### 6.4.1 Desktop (≥1200px)

- Three columns, equal width
- Horizontal arrangement
- Gap: 24px between cards

#### 6.4.2 Tablet (769px - 1199px)

- Two columns layout
- Gap: 20px between cards

#### 6.4.3 Mobile (≤768px)

- Single column, stacked vertically
- Full width cards
- Gap: 16px between cards

---

## 7. Animations & Responsiveness

### 7.1 Animation Requirements

#### 7.1.1 Card Animations

**Hover State:**
- Transform: translateY(-4px) or scale(1.02)
- Box Shadow: Increase elevation
- Transition: 300ms ease-out

**Enter Animation (on scroll into view):**
- Initial: opacity 0, translateY(20px)
- Final: opacity 1, translateY(0)
- Duration: 400ms
- Stagger: 100ms between cards

#### 7.1.2 Button Animations

**Hover State:**
- Transform: translateY(-2px)
- Box Shadow: Increase
- Transition: 200ms ease

**Active State:**
- Transform: translateY(0)
- Transition: 100ms ease

### 7.2 Reduced Motion Support

All animations MUST respect `prefers-reduced-motion`:

```
@media (prefers-reduced-motion: reduce) {
  - Disable transform animations
  - Disable opacity transitions for enter effects
  - Keep color/background transitions (instant or minimal)
}
```

### 7.3 Responsive Breakpoints

| Breakpoint | Width | Layout |
|------------|-------|--------|
| Mobile | ≤768px | Single column, stacked |
| Tablet | 769px - 1199px | Two columns or adaptive |
| Desktop | ≥1200px | Three columns |

### 7.4 Touch Device Considerations

- Hover effects degrade gracefully (no hover on touch)
- Touch targets minimum 44x44px
- No hover-dependent functionality

---

## 8. Non-Functional Requirements

### 8.1 Performance

- No JavaScript animation libraries
- CSS-only animations and transitions
- No layout shift on animation
- Animations do not block main thread

### 8.2 Accessibility

- Focus states visible on all interactive elements
- Color contrast meets WCAG AA (4.5:1 for text)
- Screen reader compatibility maintained
- Keyboard navigation preserved
- Reduced motion respected

### 8.3 Theme Consistency

- All elements support light and dark modes
- No visual regressions in either theme
- Consistent with existing Docusaurus theme
- CSS variables used for theme-aware values

### 8.4 Browser Support

- Chrome (latest 2 versions)
- Firefox (latest 2 versions)
- Safari (latest 2 versions)
- Edge (latest 2 versions)
- Mobile Safari (iOS 14+)
- Chrome for Android (latest)

---

## 9. Acceptance Criteria

### 9.1 Navbar Auth Controls

- [ ] Signup button visible in navbar when unauthenticated
- [ ] Signin button visible in navbar when unauthenticated
- [ ] Signup button navigates to `/auth?mode=signup`
- [ ] Signin button navigates to `/auth?mode=signin`
- [ ] Profile icon visible in navbar when authenticated
- [ ] Profile icon navigates to `/profile`
- [ ] Buttons/icon hidden appropriately based on auth state
- [ ] Desktop layout displays buttons inline
- [ ] Mobile layout displays buttons appropriately

### 9.2 Routing

- [ ] `/auth` route loads authentication page
- [ ] `/auth?mode=signup` displays signup form
- [ ] `/auth?mode=signin` displays signin form
- [ ] `/profile` route loads profile page
- [ ] Unauthenticated users redirected from `/profile` to `/auth`
- [ ] Authenticated users redirected from `/auth` to `/profile`

### 9.3 Sidebar

- [ ] Mobile sidebar displays auth links when unauthenticated
- [ ] Mobile sidebar displays profile section when authenticated
- [ ] Sidebar links navigate correctly
- [ ] Sign out functionality works from sidebar

### 9.4 Sidebar CSS Fix

- [ ] Sidebar fully opaque in light mode
- [ ] Sidebar fully opaque in dark mode
- [ ] No transparency on theme switch
- [ ] Background color matches theme
- [ ] No visual artifacts or flickering

### 9.5 Landing Page Cards

- [ ] Three cards display on landing page
- [ ] Cards have correct content (title, description, icon)
- [ ] Cards responsive: 3-col desktop, stacked mobile
- [ ] Card hover animations work
- [ ] Enter animations trigger on scroll
- [ ] Animations respect reduced motion
- [ ] Cards match theme (light/dark)

### 9.6 General

- [ ] No visual regressions
- [ ] All interactive elements have focus states
- [ ] Touch targets meet minimum size
- [ ] No console errors
- [ ] No layout shift during load

---

## 10. File Changes Summary

### 10.1 Files to Create

| File | Purpose |
|------|---------|
| `src/components/NavbarAuth/index.tsx` | Auth buttons/profile icon component |
| `src/components/NavbarAuth/styles.module.css` | Navbar auth styling |
| `src/components/FeatureCard/index.tsx` | Reusable feature card component |
| `src/components/FeatureCard/styles.module.css` | Feature card styling |

### 10.2 Files to Modify

| File | Changes |
|------|---------|
| `src/css/custom.css` | Sidebar CSS fix, card animations, responsive rules |
| `src/pages/auth.tsx` | Add URL parameter handling for mode |
| `src/components/HomepageFeatures/index.tsx` | Refactor to use FeatureCard |
| `src/components/HomepageFeatures/styles.module.css` | Update grid and card styles |
| `src/theme/Root.tsx` | Add AuthProvider wrapper for app-wide auth state |

### 10.3 Navbar Integration Approach

**Selected Approach**: Custom NavbarAuth component + Root-level AuthProvider

This approach:
- Creates a standalone `NavbarAuth` component that reads auth state via `useAuth()` hook
- Wraps the app with `AuthProvider` at the Root level for app-wide auth state
- Integrates NavbarAuth into navbar via `docusaurus.config.ts` custom HTML item or theme swizzle if needed
- Avoids complex navbar swizzling by using existing Docusaurus extension points

**Note**: Theme swizzling of `@theme/Navbar/MobileSidebar` may be required for mobile sidebar auth integration (US5)

---

## 11. Implementation Notes

### 11.1 Auth State Detection

- Use `useAuth()` hook from `AuthProvider`
- AuthProvider wrapped at Root level (`src/theme/Root.tsx`) for app-wide access
- Handle loading states appropriately (show skeleton while auth state loads)

### 11.2 Mobile Sidebar Integration

- Swizzle mobile sidebar components if needed
- Insert auth section as first item
- Maintain existing navigation structure

---

## 12. Testing Requirements

### 12.1 Manual Testing

| Test Case | Steps | Expected Result |
|-----------|-------|-----------------|
| Navbar unauthenticated | Load site, not logged in | Signup/Signin buttons visible |
| Navbar authenticated | Log in, return to homepage | Profile icon visible, no buttons |
| Signup navigation | Click Signup | Navigate to /auth with signup form |
| Signin navigation | Click Signin | Navigate to /auth with signin form |
| Profile navigation | Click Profile icon | Navigate to /profile |
| Sidebar light mode | Open mobile sidebar in light mode | Solid background, no transparency |
| Sidebar dark mode | Open mobile sidebar in dark mode | Solid background, no transparency |
| Cards responsive | Resize browser | Cards reflow correctly |
| Card hover | Hover over card (desktop) | Animation triggers |
| Reduced motion | Enable reduced motion, hover card | No animation |

### 12.2 Cross-Browser Testing

- Test all acceptance criteria in supported browsers
- Verify no browser-specific CSS issues
- Test mobile browsers on actual devices if possible

---

## 13. Revision History

| Version | Date | Author | Changes |
|---------|------|--------|---------|
| 1.0.0 | 2025-12-16 | Spec System | Initial specification |
