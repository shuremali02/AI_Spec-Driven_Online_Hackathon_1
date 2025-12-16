import React from 'react';
import Link from '@docusaurus/Link';
import { useAuth } from '@site/src/components/Auth/AuthProvider';
import styles from './styles.module.css';
import clsx from 'clsx';

interface NavbarAuthProps {
  variant?: 'navbar' | 'sidebar';
  className?: string;
  onNavigate?: () => void;
}

export default function NavbarAuth({
  variant = 'navbar',
  className,
  onNavigate,
}: NavbarAuthProps): React.ReactElement {
  const { user, profile, isLoading, signOut } = useAuth();

  const handleNavigation = () => {
    onNavigate?.();
  };

  const handleSignOut = async () => {
    await signOut();
    onNavigate?.();
  };

  // Loading state - show skeleton
  if (isLoading) {
    return (
      <div className={clsx(styles.authContainer, styles[variant], className)}>
        <div className={styles.skeleton} />
      </div>
    );
  }

  // Authenticated state - show profile icon or user info
  if (user) {
    const displayName = profile?.displayName || user.name || user.email || 'User';
    const initials = displayName.charAt(0).toUpperCase();

    // Sidebar variant - show full user info with sign out
    if (variant === 'sidebar') {
      return (
        <div className={clsx(styles.authContainer, styles.sidebar, className)}>
          <div className={styles.sidebarUserInfo}>
            <div className={styles.profileIcon}>{initials}</div>
            <div className={styles.userDetails}>
              <span className={styles.userName}>{displayName}</span>
              <span className={styles.userEmail}>{user.email}</span>
            </div>
          </div>
          <Link
            to="/profile"
            className={styles.sidebarLink}
            onClick={handleNavigation}
          >
            View Profile
          </Link>
          <button
            onClick={handleSignOut}
            className={styles.signOutButton}
            type="button"
          >
            Sign Out
          </button>
        </div>
      );
    }

    // Navbar variant - show profile icon only
    return (
      <div className={clsx(styles.authContainer, styles.navbar, className)}>
        <Link
          to="/profile"
          className={styles.profileIconLink}
          aria-label={`Profile for ${displayName}`}
          onClick={handleNavigation}
        >
          <span className={styles.profileIcon}>{initials}</span>
        </Link>
      </div>
    );
  }

  // Unauthenticated state - show auth buttons
  return (
    <div className={clsx(styles.authContainer, styles[variant], className)}>
      <Link
        to="/auth?mode=signup"
        className={clsx(styles.authButton, styles.secondary)}
        onClick={handleNavigation}
      >
        Sign Up
      </Link>
      <Link
        to="/auth?mode=signin"
        className={clsx(styles.authButton, styles.primary)}
        onClick={handleNavigation}
      >
        Sign In
      </Link>
    </div>
  );
}
