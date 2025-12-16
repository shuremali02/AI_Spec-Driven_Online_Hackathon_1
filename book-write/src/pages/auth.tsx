import React, { useState, useEffect } from 'react';
import Layout from '@theme/Layout';
import { AuthProvider, useAuth } from '../components/Auth/AuthProvider';
import { SignupForm } from '../components/Auth/SignupForm';
import { SigninForm } from '../components/Auth/SigninForm';

type AuthMode = 'signin' | 'signup';

// Extract auth mode from URL parameters
function getAuthModeFromUrl(): AuthMode {
  if (typeof window === 'undefined') return 'signin';
  const params = new URLSearchParams(window.location.search);
  return params.get('mode') === 'signup' ? 'signup' : 'signin';
}

function AuthContent(): React.ReactElement {
  const [mode, setMode] = useState<AuthMode>(getAuthModeFromUrl);
  const { user, profile, isLoading } = useAuth();

  // Update mode when URL changes (browser back/forward)
  useEffect(() => {
    const handlePopState = () => {
      setMode(getAuthModeFromUrl());
    };
    window.addEventListener('popstate', handlePopState);
    return () => window.removeEventListener('popstate', handlePopState);
  }, []);

  // Only redirect if user is logged in AND has completed profile
  React.useEffect(() => {
    if (user && profile && !isLoading) {
      window.location.href = '/profile';
    }
  }, [user, profile, isLoading]);

  // Called only after ALL signup steps complete (including profile creation)
  const handleSignupSuccess = () => {
    window.location.href = '/profile';
  };

  // Called after signin (user already has profile)
  const handleSigninSuccess = () => {
    window.location.href = '/profile';
  };

  if (isLoading) {
    return (
      <div style={{ textAlign: 'center', padding: '3rem' }}>
        Loading...
      </div>
    );
  }

  return (
    <div style={{ maxWidth: '600px', margin: '0 auto', padding: '2rem' }}>
      {mode === 'signin' ? (
        <SigninForm
          onSuccess={handleSigninSuccess}
          onSwitchToSignup={() => setMode('signup')}
        />
      ) : (
        <SignupForm
          onSuccess={handleSignupSuccess}
          onSwitchToSignin={() => setMode('signin')}
        />
      )}
    </div>
  );
}

export default function AuthPage(): React.ReactElement {
  return (
    <Layout
      title="Authentication"
      description="Sign in or create an account"
    >
      <main style={{ padding: '2rem 0' }}>
        <AuthProvider>
          <AuthContent />
        </AuthProvider>
      </main>
    </Layout>
  );
}
