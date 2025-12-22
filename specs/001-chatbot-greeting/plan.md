# Implementation Plan: Personalized Chatbot Greeting

**Branch**: `001-chatbot-greeting` | **Date**: 2025-12-22 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/001-chatbot-greeting/spec.md`

## Summary

Add personalized greeting to the chatbot that displays the user's name with a time-appropriate message (Good morning/afternoon/evening). The implementation is frontend-only, leveraging the existing AuthProvider to get user information and the browser's local time for determining the greeting period.

## Technical Context

**Language/Version**: TypeScript 5.x (React 18)
**Primary Dependencies**: React, AuthProvider (existing), Docusaurus
**Storage**: N/A (uses existing auth context - no new storage)
**Testing**: Manual testing (React component)
**Target Platform**: Web browser (Docusaurus site)
**Project Type**: Web application (frontend only)
**Performance Goals**: Greeting renders within 100ms of chatbot opening
**Constraints**: Must work with existing AuthProvider, no backend changes
**Scale/Scope**: Single component modification (ChatWindow.tsx)

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Principle | Status | Notes |
|-----------|--------|-------|
| V. Authentication-First Access Control | ✅ PASS | Personalized greeting only shown to logged-in users |
| VI. Personalization Standard | ✅ PASS | Uses existing user profile data (name from auth) |
| II. Clarity and Educational Value | ✅ PASS | Enhances user experience without changing content |
| III. Consistency and Uniformity | ✅ PASS | Maintains consistent tone with rest of chatbot |

**Gate Result**: ✅ PASS - No violations. Proceed to Phase 0.

## Project Structure

### Documentation (this feature)

```text
specs/001-chatbot-greeting/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Phase 0 output
├── quickstart.md        # Phase 1 output
└── checklists/
    └── requirements.md  # Quality checklist
```

### Source Code (repository root)

```text
book-write/
└── src/
    └── components/
        └── Chatbot/
            ├── ChatWindow.tsx     # MODIFY: Add personalized greeting
            ├── ChatWindow.css     # MODIFY: Add greeting styles (if needed)
            ├── FloatingButton.tsx # NO CHANGE
            ├── index.tsx          # NO CHANGE
            └── api.ts             # NO CHANGE
```

**Structure Decision**: Frontend-only modification. Only `ChatWindow.tsx` needs changes to display personalized greeting. May need minor CSS additions for styling.

## Complexity Tracking

No violations - feature is simple and well-bounded.

---

## Phase 0: Research

### Research Tasks

1. **AuthProvider Integration**: How to access user name from existing AuthProvider
2. **Time-based Greeting**: Best practices for time period detection in browser
3. **Existing Component Structure**: Current ChatWindow welcome message implementation

### Research Findings

#### 1. AuthProvider Integration

**Decision**: Use existing `useAuth` hook from `@site/src/components/Auth/AuthProvider`

**Rationale**:
- AuthProvider already provides `user` object with `name` property
- Same pattern used successfully in ChapterPersonalizer component
- No additional dependencies required

**Code Reference**:
```typescript
// From AuthProvider.tsx (lines 20-24)
const user: AuthUser | null = session?.user ? {
  id: session.user.id,
  email: session.user.email,
  name: session.user.name,
} : null;
```

#### 2. Time-based Greeting

**Decision**: Use JavaScript `Date` object with `getHours()` method

**Rationale**:
- Native browser API, no dependencies
- Automatically uses user's local timezone
- Simple implementation with clear time boundaries

**Implementation Pattern**:
```typescript
const getGreetingPrefix = (): string => {
  const hour = new Date().getHours();
  if (hour >= 5 && hour < 12) return 'Good morning';
  if (hour >= 12 && hour < 18) return 'Good afternoon';
  return 'Good evening';
};
```

#### 3. Existing Component Structure

**Decision**: Modify the welcome message section in ChatWindow.tsx (lines 255-258)

**Current Implementation** (ChatWindow.tsx:255-258):
```tsx
{messages.length === 0 ? (
  <div className="chat-welcome-message">
    <p>Hello! I'm your textbook assistant. Ask me anything about the Physical AI & Humanoid Robotics content...</p>
  </div>
) : ( ... )}
```

**Rationale**:
- Welcome message already exists and is conditionally rendered
- Natural place to add personalization
- No structural changes needed to component

---

## Phase 1: Design

### Data Model

No new data entities required. Uses existing:
- **User** (from AuthProvider): `{ id: string, email: string, name: string }`

### API Contracts

No API changes required. This is a frontend-only feature.

### Implementation Design

#### Helper Function: `getPersonalizedGreeting`

```typescript
/**
 * Generates a personalized greeting based on time and user name
 * @param userName - User's display name (optional)
 * @returns Greeting message string
 */
const getPersonalizedGreeting = (userName?: string | null): string => {
  const hour = new Date().getHours();
  let prefix: string;

  if (hour >= 5 && hour < 12) {
    prefix = 'Good morning';
  } else if (hour >= 12 && hour < 18) {
    prefix = 'Good afternoon';
  } else {
    prefix = 'Good evening';
  }

  if (userName) {
    return `${prefix}, ${userName}! How can I help you today?`;
  }
  return `${prefix}! How can I help you today?`;
};
```

#### Component Changes

1. Import `useAuth` hook
2. Get user from auth context
3. Replace static welcome message with dynamic greeting

### File Changes Summary

| File | Change Type | Description |
|------|-------------|-------------|
| `ChatWindow.tsx` | MODIFY | Add useAuth hook, create greeting helper, update welcome message |
| `ChatWindow.css` | OPTIONAL | Add greeting-specific styles if needed |

---

## Implementation Checklist

- [ ] Add `useAuth` import to ChatWindow.tsx
- [ ] Create `getPersonalizedGreeting` helper function
- [ ] Update welcome message to use personalized greeting
- [ ] Test with logged-in user (morning, afternoon, evening)
- [ ] Test with anonymous user
- [ ] Test with user without name set
- [ ] Verify no performance impact

---

## Post-Design Constitution Re-Check

| Principle | Status | Notes |
|-----------|--------|-------|
| V. Authentication-First Access Control | ✅ PASS | Name only shown for authenticated users |
| VI. Personalization Standard | ✅ PASS | Uses user's name from auth profile |
| II. Clarity and Educational Value | ✅ PASS | Friendly greeting enhances UX |
| III. Consistency and Uniformity | ✅ PASS | Consistent with chatbot tone |

**Final Gate Result**: ✅ PASS - Ready for task generation (`/sp.tasks`)
