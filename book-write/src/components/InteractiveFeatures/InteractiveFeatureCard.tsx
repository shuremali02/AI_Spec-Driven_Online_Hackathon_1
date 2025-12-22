import React from 'react';
import Link from '@docusaurus/Link';
import { useAuth } from '@site/src/components/Auth/AuthProvider';
import styles from './styles.module.css';

interface InteractiveFeatureCardProps {
  title: string;
  description: string;
  icon: React.ReactNode;
  ctaText: string;
  featureUrl: string;
  animationDelay?: number;
  requiresAuth?: boolean;
  opensChatbot?: boolean;
}

export default function InteractiveFeatureCard({
  title,
  description,
  icon,
  ctaText,
  featureUrl,
  animationDelay = 0,
  requiresAuth = true,
  opensChatbot = false,
}: InteractiveFeatureCardProps): React.ReactElement {
  const { user } = useAuth();

  // Features that require auth: anonymous users go to auth page, logged-in users go to feature
  const targetUrl = requiresAuth ? (user ? featureUrl : '/auth') : featureUrl;
  const buttonText = requiresAuth ? (user ? ctaText : 'Sign Up to Access') : ctaText;

  // Handle chatbot button click - dispatch event to open chatbot
  const handleChatbotClick = (e: React.MouseEvent) => {
    e.preventDefault();
    window.dispatchEvent(new CustomEvent('open-chatbot'));
  };

  return (
    <article
      className={styles.card}
      style={{ animationDelay: `${animationDelay}ms` }}
    >
      <div className={styles.iconContainer}>
        {icon}
      </div>
      <h3 className={styles.title}>{title}</h3>
      <p className={styles.description}>{description}</p>
      {opensChatbot ? (
        <button onClick={handleChatbotClick} className={styles.ctaButton}>
          {ctaText}
        </button>
      ) : (
        <Link to={targetUrl} className={styles.ctaButton}>
          {buttonText}
        </Link>
      )}
    </article>
  );
}
