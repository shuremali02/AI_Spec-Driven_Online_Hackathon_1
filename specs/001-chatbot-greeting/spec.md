# Feature Specification: Personalized Chatbot Greeting

**Feature Branch**: `001-chatbot-greeting`
**Created**: 2025-12-22
**Status**: Draft
**Input**: User description: "Personalized Chatbot Greeting Feature - Add user name in chatbot welcome message like 'Good afternoon, Shurem! How can I help you today?'"

## Overview

The chatbot currently displays a generic welcome message when users open it. This feature will personalize the greeting by:
1. Displaying the user's name if they are logged in
2. Including a time-appropriate greeting (Good morning/afternoon/evening)
3. Falling back to a generic greeting for anonymous users

## User Scenarios & Testing *(mandatory)*

### User Story 1 - Logged-in User Sees Personalized Greeting (Priority: P1)

As a logged-in user, when I open the chatbot, I want to see a personalized greeting with my name and a time-appropriate message so that the experience feels more personal and welcoming.

**Why this priority**: This is the core feature request - personalizing the experience for authenticated users provides immediate value and user engagement.

**Independent Test**: Can be fully tested by logging in as a user with a known name, opening the chatbot, and verifying the greeting displays their name with the correct time-based greeting.

**Acceptance Scenarios**:

1. **Given** a user "Ali" is logged in and the time is 10:00 AM, **When** they open the chatbot, **Then** they see "Good morning, Ali! How can I help you today?"
2. **Given** a user "Shurem" is logged in and the time is 3:00 PM, **When** they open the chatbot, **Then** they see "Good afternoon, Shurem! How can I help you today?"
3. **Given** a user "Maria" is logged in and the time is 8:00 PM, **When** they open the chatbot, **Then** they see "Good evening, Maria! How can I help you today?"

---

### User Story 2 - Anonymous User Sees Generic Greeting (Priority: P2)

As an anonymous user (not logged in), when I open the chatbot, I want to see a friendly generic greeting so that I can still use the chatbot without feeling excluded.

**Why this priority**: Important for users who haven't logged in yet - maintains usability for all users.

**Independent Test**: Can be tested by opening the chatbot without logging in and verifying a generic welcome message appears.

**Acceptance Scenarios**:

1. **Given** no user is logged in and the time is 10:00 AM, **When** they open the chatbot, **Then** they see "Good morning! How can I help you today?"
2. **Given** no user is logged in and the time is 3:00 PM, **When** they open the chatbot, **Then** they see "Good afternoon! How can I help you today?"

---

### User Story 3 - User Name Display from Profile (Priority: P3)

As a logged-in user, I want the chatbot to use my display name (first name) from my profile, so that the greeting feels natural and not overly formal.

**Why this priority**: Enhances the personalization quality by using the appropriate name format.

**Independent Test**: Can be tested by setting up users with different name formats and verifying the chatbot extracts and displays the appropriate name.

**Acceptance Scenarios**:

1. **Given** a user with profile name "Muhammad Ali Khan", **When** they open the chatbot, **Then** the greeting uses "Ali" or the first name component
2. **Given** a user with only first name "Shurem" in profile, **When** they open the chatbot, **Then** the greeting uses "Shurem"

---

### Edge Cases

- What happens when user's name is empty or null? → Display generic greeting without name
- What happens when user's name contains special characters? → Display name as-is (sanitized for XSS)
- How does system handle timezone differences? → Use browser's local time for time-based greeting
- What happens if auth state changes while chatbot is open? → Greeting remains until chatbot is closed and reopened

## Requirements *(mandatory)*

### Functional Requirements

- **FR-001**: System MUST display a personalized greeting with user's name when a logged-in user opens the chatbot
- **FR-002**: System MUST include a time-appropriate greeting prefix (morning/afternoon/evening) based on user's local time
- **FR-003**: System MUST fall back to a generic greeting (without name) for anonymous/logged-out users
- **FR-004**: System MUST extract user's name from the authenticated user context/profile
- **FR-005**: System MUST handle missing or null user names gracefully by omitting the name from greeting
- **FR-006**: System MUST use the user's browser local time to determine the appropriate greeting period

### Time Period Definitions

- **Morning**: 5:00 AM - 11:59 AM → "Good morning"
- **Afternoon**: 12:00 PM - 5:59 PM → "Good afternoon"
- **Evening**: 6:00 PM - 4:59 AM → "Good evening"

### Key Entities

- **User**: The authenticated user with name/profile information from the auth system
- **Greeting Message**: The dynamic welcome message displayed in the chatbot header/welcome area

## Success Criteria *(mandatory)*

### Measurable Outcomes

- **SC-001**: 100% of logged-in users with valid names see a personalized greeting with their name
- **SC-002**: Greeting time period is accurate to the user's local timezone 100% of the time
- **SC-003**: Anonymous users continue to see a welcoming message without errors
- **SC-004**: Greeting renders within 100ms of chatbot opening (no perceptible delay)
- **SC-005**: User satisfaction with personalization improves engagement (qualitative measure)

## Assumptions

- User's name is available from the existing AuthProvider/useAuth hook
- The chatbot component (ChatWindow.tsx) already has access to or can access the auth context
- Browser's local time is reliable for determining time of day
- No backend changes required - this is a frontend-only feature

## Out of Scope

- Custom greeting messages per user preference
- Multi-language greetings (future enhancement)
- Greeting based on user's activity history
- Backend API changes
