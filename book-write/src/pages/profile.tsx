import React from 'react';
import Layout from '@theme/Layout';
import { AuthProvider } from '../components/Auth/AuthProvider';
import { UserProfile } from '../components/Profile/UserProfile';

export default function ProfilePage(): React.ReactElement {
  return (
    <Layout
      title="Profile"
      description="View and manage your user profile"
    >
      <main style={{ padding: '2rem 0' }}>
        <AuthProvider>
          <UserProfile />
        </AuthProvider>
      </main>
    </Layout>
  );
}
