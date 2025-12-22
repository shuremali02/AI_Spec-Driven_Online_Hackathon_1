# Research: Personalized Chatbot Greeting

**Feature**: 001-chatbot-greeting
**Date**: 2025-12-22

## Research Summary

This document captures research findings for implementing personalized greetings in the chatbot component.

---

## 1. AuthProvider Integration

### Question
How to access user name from existing AuthProvider in Chatbot component?

### Decision
Use existing `useAuth` hook from `@site/src/components/Auth/AuthProvider`

### Rationale
- AuthProvider already provides `user` object with `name` property
- Same pattern used successfully in ChapterPersonalizer component
- No additional dependencies required
- Consistent with existing codebase patterns

### Alternatives Considered

| Alternative | Rejected Because |
|-------------|------------------|
| Direct localStorage access | Bypasses auth state management, inconsistent |
| Create new context | Unnecessary - auth context already exists |
| Prop drilling from parent | Over-complicated, useAuth is simpler |

### Code Reference
```typescript
// AuthProvider.tsx:20-24
const user: AuthUser | null = session?.user ? {
  id: session.user.id,
  email: session.user.email,
  name: session.user.name,
} : null;

// Usage pattern (from ChapterPersonalizer.tsx:30)
const { user, profile, isLoading: authLoading } = useAuth();
```

---

## 2. Time-Based Greeting

### Question
Best approach for determining time period (morning/afternoon/evening) in browser?

### Decision
Use JavaScript native `Date` object with `getHours()` method

### Rationale
- Native browser API, zero dependencies
- Automatically uses user's local timezone
- Simple, readable implementation
- Consistent behavior across browsers

### Alternatives Considered

| Alternative | Rejected Because |
|-------------|------------------|
| moment.js / dayjs | Overkill for simple hour check, adds bundle size |
| Server-side time | Adds latency, timezone complexity |
| UTC time | Doesn't match user's local experience |

### Time Period Boundaries
Based on spec requirements:
- **Morning**: 5:00 AM - 11:59 AM (hours 5-11)
- **Afternoon**: 12:00 PM - 5:59 PM (hours 12-17)
- **Evening**: 6:00 PM - 4:59 AM (hours 18-23, 0-4)

### Implementation
```typescript
const getGreetingPrefix = (): string => {
  const hour = new Date().getHours();
  if (hour >= 5 && hour < 12) return 'Good morning';
  if (hour >= 12 && hour < 18) return 'Good afternoon';
  return 'Good evening';
};
```

---

## 3. ChatWindow Component Structure

### Question
Where and how to integrate personalized greeting in existing ChatWindow component?

### Decision
Modify the welcome message section in ChatWindow.tsx (lines 255-258)

### Rationale
- Welcome message already exists and is conditionally rendered
- Natural place to add personalization
- Minimal code changes required
- No structural changes to component

### Current Implementation
```tsx
// ChatWindow.tsx:254-259
{messages.length === 0 ? (
  <div className="chat-welcome-message">
    <p>Hello! I'm your textbook assistant. Ask me anything about
    the Physical AI & Humanoid Robotics content, and I'll provide
    answers based on the textbook with citations.</p>
  </div>
) : ( ... )}
```

### Proposed Change
```tsx
{messages.length === 0 ? (
  <div className="chat-welcome-message">
    <p className="greeting">{getPersonalizedGreeting(user?.name)}</p>
    <p>I'm your textbook assistant. Ask me anything about
    the Physical AI & Humanoid Robotics content, and I'll provide
    answers based on the textbook with citations.</p>
  </div>
) : ( ... )}
```

---

## 4. Edge Case Handling

### Null/Empty Name
- If `user` is null → anonymous user → greeting without name
- If `user.name` is empty/null → greeting without name
- Implementation handles via optional chaining and conditional

### Special Characters in Name
- Display as-is (React handles XSS sanitization automatically)
- No additional sanitization needed for display-only content

### Auth State Changes
- Greeting is computed on render
- If user logs out while chatbot is open, next render shows generic greeting
- No special handling needed

---

## Conclusion

All research questions resolved. No NEEDS CLARIFICATION items remain.
Ready for implementation.
