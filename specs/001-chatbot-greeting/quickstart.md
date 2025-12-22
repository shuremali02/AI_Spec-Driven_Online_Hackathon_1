# Quickstart: Personalized Chatbot Greeting

**Feature**: 001-chatbot-greeting
**Estimated Time**: 15-30 minutes

## Prerequisites

- [ ] Node.js 18+ installed
- [ ] Project dependencies installed (`npm install` in book-write/)
- [ ] Development server can start (`npm run start`)

## Quick Implementation Steps

### Step 1: Open ChatWindow.tsx

```bash
# File location
book-write/src/components/Chatbot/ChatWindow.tsx
```

### Step 2: Add Import

Add at the top of the file (after existing imports):

```typescript
import { useAuth } from '@site/src/components/Auth/AuthProvider';
```

### Step 3: Add Helper Function

Add before the `ChatWindow` component definition:

```typescript
/**
 * Generates a personalized greeting based on time and user name
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

### Step 4: Use Auth Hook

Inside the `ChatWindow` component, add near the top:

```typescript
const { user } = useAuth();
```

### Step 5: Update Welcome Message

Replace the static welcome message with:

```tsx
{messages.length === 0 ? (
  <div className="chat-welcome-message">
    <p className="greeting">{getPersonalizedGreeting(user?.name)}</p>
    <p>I'm your textbook assistant. Ask me anything about the Physical AI & Humanoid Robotics content, and I'll provide answers based on the textbook with citations.</p>
  </div>
) : ( ... )}
```

## Testing

### Manual Tests

1. **Anonymous User (Morning)**
   - Log out / clear session
   - Open chatbot at 10 AM
   - Expected: "Good morning! How can I help you today?"

2. **Logged-in User (Afternoon)**
   - Log in as user "Shurem"
   - Open chatbot at 3 PM
   - Expected: "Good afternoon, Shurem! How can I help you today?"

3. **Logged-in User (Evening)**
   - Log in as any user
   - Open chatbot at 8 PM
   - Expected: "Good evening, [Name]! How can I help you today?"

4. **User Without Name**
   - Log in with user that has no name set
   - Open chatbot
   - Expected: Time-based greeting without name

## Troubleshooting

### useAuth Hook Error
If you get "useAuth must be used within an AuthProvider":
- Ensure AuthProvider wraps the Chatbot component in the component tree
- Check `book-write/src/theme/Root.tsx` for AuthProvider placement

### User Name Not Showing
- Check browser console for auth state
- Verify user is logged in: `console.log(user)` in component
- Check if `user.name` property exists in auth response

## Done!

After implementation, run the development server and test all scenarios:

```bash
cd book-write
npm run start
```
