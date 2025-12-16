import { createAuthClient } from 'better-auth/react';

// Auth service URL - always use HF Space for now
// Change to localhost:3001 for local backend development
function getAuthUrl(): string {
  return 'https://shurem-better-auth.hf.space';
}

export const AUTH_BASE_URL = getAuthUrl();

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
