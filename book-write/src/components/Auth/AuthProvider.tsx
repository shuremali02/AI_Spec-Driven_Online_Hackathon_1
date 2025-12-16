import React, { createContext, useContext, useState, useEffect, useCallback, type ReactNode } from 'react';
import { useSession, signOut as authSignOut } from './authClient';
import { PROFILE_API_URL } from './authClient';
import type { ProfileResponse, AuthUser, AuthState } from './types';

interface AuthContextValue extends AuthState {
  signOut: () => Promise<void>;
  refreshProfile: () => Promise<void>;
  setProfile: (profile: ProfileResponse | null) => void;
}

const AuthContext = createContext<AuthContextValue | undefined>(undefined);

export function AuthProvider({ children }: { children: ReactNode }) {
  const { data: session, isPending: sessionLoading } = useSession();
  const [profile, setProfile] = useState<ProfileResponse | null>(null);
  const [profileLoading, setProfileLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  const user: AuthUser | null = session?.user ? {
    id: session.user.id,
    email: session.user.email,
    name: session.user.name,
  } : null;

  const fetchProfile = useCallback(async () => {
    if (!user) {
      setProfile(null);
      return;
    }

    setProfileLoading(true);
    setError(null);

    try {
      const response = await fetch(PROFILE_API_URL, {
        method: 'GET',
        credentials: 'include',
        headers: {
          'Content-Type': 'application/json',
        },
      });

      if (response.ok) {
        const data = await response.json();
        setProfile(data);
      } else if (response.status === 404) {
        // Profile not found - user exists but hasn't created profile yet
        setProfile(null);
      } else if (response.status === 401) {
        // Session invalid
        setProfile(null);
      } else {
        const errorData = await response.json().catch(() => ({}));
        setError(errorData.message || 'Failed to fetch profile');
      }
    } catch (err) {
      console.error('Profile fetch error:', err);
      setError('Failed to connect to auth service');
    } finally {
      setProfileLoading(false);
    }
  }, [user?.id]);

  // Fetch profile when session changes
  useEffect(() => {
    if (session?.user) {
      fetchProfile();
    } else {
      setProfile(null);
    }
  }, [session?.user?.id, fetchProfile]);

  const handleSignOut = async () => {
    try {
      await authSignOut();
      setProfile(null);
      setError(null);
    } catch (err) {
      console.error('Sign out error:', err);
      setError('Failed to sign out');
    }
  };

  const value: AuthContextValue = {
    user,
    profile,
    isLoading: sessionLoading || profileLoading,
    error,
    signOut: handleSignOut,
    refreshProfile: fetchProfile,
    setProfile,
  };

  return (
    <AuthContext.Provider value={value}>
      {children}
    </AuthContext.Provider>
  );
}

export function useAuth(): AuthContextValue {
  const context = useContext(AuthContext);
  if (!context) {
    throw new Error('useAuth must be used within an AuthProvider');
  }
  return context;
}

export default AuthProvider;
