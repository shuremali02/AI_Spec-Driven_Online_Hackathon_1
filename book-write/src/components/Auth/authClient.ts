import { createAuthClient } from 'better-auth/react';

// Auth service URL - change this for production
const AUTH_BASE_URL = process.env.NODE_ENV === 'production'
  ? 'https://auth.your-domain.com'
  : 'http://localhost:3001';

export const authClient = createAuthClient({
  baseURL: AUTH_BASE_URL,
});

export const {
  useSession,
  signIn,
  signUp,
  signOut,
} = authClient;

// Profile API URL
export const PROFILE_API_URL = `${AUTH_BASE_URL}/api/profile`;
