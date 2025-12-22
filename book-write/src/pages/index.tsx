import React, { useState, useEffect, type ReactNode } from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import InteractiveFeatures from '@site/src/components/InteractiveFeatures';

import styles from './index.module.css';

// Typing effect hook
function useTypingEffect(texts: string[], typingSpeed = 100, deletingSpeed = 50, pauseDuration = 2000) {
  const [displayText, setDisplayText] = useState('');
  const [textIndex, setTextIndex] = useState(0);
  const [isDeleting, setIsDeleting] = useState(false);

  useEffect(() => {
    const currentText = texts[textIndex];

    const timeout = setTimeout(() => {
      if (!isDeleting) {
        if (displayText.length < currentText.length) {
          setDisplayText(currentText.slice(0, displayText.length + 1));
        } else {
          setTimeout(() => setIsDeleting(true), pauseDuration);
        }
      } else {
        if (displayText.length > 0) {
          setDisplayText(displayText.slice(0, -1));
        } else {
          setIsDeleting(false);
          setTextIndex((prev) => (prev + 1) % texts.length);
        }
      }
    }, isDeleting ? deletingSpeed : typingSpeed);

    return () => clearTimeout(timeout);
  }, [displayText, isDeleting, textIndex, texts, typingSpeed, deletingSpeed, pauseDuration]);

  return displayText;
}

// Animated counter hook
function useCountUp(end: number, duration = 2000, start = 0) {
  const [count, setCount] = useState(start);
  const [hasAnimated, setHasAnimated] = useState(false);

  useEffect(() => {
    if (hasAnimated) return;

    const startTime = Date.now();
    const animate = () => {
      const elapsed = Date.now() - startTime;
      const progress = Math.min(elapsed / duration, 1);
      // Easing function for smooth animation
      const easeOut = 1 - Math.pow(1 - progress, 3);
      setCount(Math.floor(start + (end - start) * easeOut));

      if (progress < 1) {
        requestAnimationFrame(animate);
      } else {
        setHasAnimated(true);
      }
    };

    requestAnimationFrame(animate);
  }, [end, duration, start, hasAnimated]);

  return count;
}

// SVG Icons
const RobotIcon = () => (
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <rect x="3" y="11" width="18" height="10" rx="2" />
    <circle cx="12" cy="5" r="3" />
    <path d="M12 8v3M7 15h.01M17 15h.01" />
  </svg>
);

const CodeIcon = () => (
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <polyline points="16,18 22,12 16,6" />
    <polyline points="8,6 2,12 8,18" />
  </svg>
);

const BrainIcon = () => (
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M12 2a4 4 0 0 1 4 4 4 4 0 0 1-1 2.65A4 4 0 0 1 16 12a4 4 0 0 1-4 4 4 4 0 0 1-4-4 4 4 0 0 1 1-2.65A4 4 0 0 1 8 6a4 4 0 0 1 4-4z" />
    <path d="M12 16v6M8 22h8" />
  </svg>
);


function HomepageHeader() {
  const { siteConfig } = useDocusaurusContext();

  const typingTexts = [
    'Build conversational AI robots',
    'Master ROS 2 middleware',
    'Create Vision-Language-Action systems',
    'Deploy on real humanoid hardware',
  ];

  const typedText = useTypingEffect(typingTexts, 80, 40, 2500);

  const weeksCount = useCountUp(13);
  const modulesCount = useCountUp(4);
  const labsCount = useCountUp(50);

  return (
    <header className={styles.heroBanner}>
      <div className="container">
        <div className={styles.heroContent}>
          <h1 className={styles.heroTitle}>{siteConfig.title}</h1>
          <p className={styles.heroSubtitle}>{siteConfig.tagline}</p>
          <p className={styles.typingText}>
            {typedText}
            <span className={styles.cursor} />
          </p>
          <div className={styles.buttons}>
            <Link className={styles.primaryButton} to="/docs/">
              Start Learning Now
            </Link>
            <Link className={styles.secondaryButton} to="/docs/module-1/">
              View Curriculum
            </Link>
          </div>
          <div className={styles.heroStats}>
            <div className={styles.statItem}>
              <span className={styles.statNumber}>{weeksCount}</span>
              <span className={styles.statLabel}>Weeks</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statNumber}>{modulesCount}</span>
              <span className={styles.statLabel}>Modules</span>
            </div>
            <div className={styles.statItem}>
              <span className={styles.statNumber}>{labsCount}+</span>
              <span className={styles.statLabel}>Labs</span>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

// Course highlights data
const courseHighlights = [
  {
    icon: <RobotIcon />,
    title: 'Physical AI Fundamentals',
    description: 'Understand embodied intelligence and how AI transcends digital to physical robots.',
  },
  {
    icon: <CodeIcon />,
    title: 'ROS 2 & Python',
    description: 'Master robot middleware with hands-on Python programming using rclpy.',
  },
  {
    icon: <BrainIcon />,
    title: 'Vision-Language-Action',
    description: 'Build conversational robots that see, understand, and act in the real world.',
  },
];

function WelcomeSection() {
  return (
    <section className={styles.welcomeSection}>
      <div className="container">
        <div className={styles.welcomeHeader}>
          <h2 className={styles.welcomeTitle}>Welcome to Your Robotics Journey</h2>
          <p className={styles.welcomeSubtitle}>
            A comprehensive 13-week course bridging advanced AI algorithms with physical humanoid robots.
            From ROS 2 basics to building conversational AI systems.
          </p>
        </div>

        <div className={styles.highlightsGrid}>
          {courseHighlights.map((highlight) => (
            <div key={highlight.title} className={styles.highlightCard}>
              <div className={styles.highlightIcon}>{highlight.icon}</div>
              <h3 className={styles.highlightTitle}>{highlight.title}</h3>
              <p className={styles.highlightText}>{highlight.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

// Target audience data
const targetAudience = [
  {
    emoji: '🎓',
    role: 'Students',
    description: 'CS/Robotics students wanting hands-on AI experience',
  },
  {
    emoji: '💻',
    role: 'Developers',
    description: 'Software engineers transitioning to robotics',
  },
  {
    emoji: '🔬',
    role: 'Researchers',
    description: 'AI researchers exploring physical embodiment',
  },
  {
    emoji: '🚀',
    role: 'Hobbyists',
    description: 'Tech enthusiasts building personal robot projects',
  },
];

function AudienceSection() {
  return (
    <section className={styles.audienceSection}>
      <div className="container">
        <div className={styles.audienceHeader}>
          <h2 className={styles.audienceTitle}>Who Is This For?</h2>
          <p className={styles.audienceSubtitle}>Perfect for anyone passionate about robotics and AI</p>
        </div>

        <div className={styles.audienceGrid}>
          {targetAudience.map((audience) => (
            <div key={audience.role} className={styles.audienceCard}>
              <span className={styles.audienceEmoji}>{audience.emoji}</span>
              <h3 className={styles.audienceRole}>{audience.role}</h3>
              <p className={styles.audienceDesc}>{audience.description}</p>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}


export default function Home(): ReactNode {
  const { siteConfig } = useDocusaurusContext();
  return (
    <Layout
      title="Physical AI & Humanoid Robotics"
      description="Learn Physical AI & Humanoid Robotics - from ROS 2 fundamentals to conversational AI robots"
    >
      <HomepageHeader />
      <main>
        <WelcomeSection />
        <HomepageFeatures />
        <AudienceSection />
        <InteractiveFeatures />
      </main>
    </Layout>
  );
}
