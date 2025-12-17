import { createAuthClient } from 'better-auth/react';

// Auth service URL
// Toggle between local and production
function getAuthUrl(): string {
  // For local testing, use localhost:3001
  return 'http://localhost:3001';
  // For production, use HF Space:
  // return 'https://shurem-better-auth.hf.space';
}

export const AUTH_BASE_URL = getAuthUrl();

export const authClient = createAuthClient({
  baseURL: AUTH_BASE_URL,
  fetchOptions: {
    credentials: 'include',
  },
});

export const {
  useSession,
  signIn,
  signUp,
  signOut,
} = authClient;

// Profile API URL
export const PROFILE_API_URL = `${AUTH_BASE_URL}/api/profile`;
