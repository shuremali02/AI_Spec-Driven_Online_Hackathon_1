import React from 'react';
import InteractiveFeatureCard from './InteractiveFeatureCard';
import styles from './styles.module.css';

// SVG Icons
const PersonalizationIcon = () => (
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <circle cx="12" cy="8" r="4" />
    <path d="M20 21v-2a4 4 0 0 0-4-4H8a4 4 0 0 0-4 4v2" />
    <path d="M16 3.13a4 4 0 0 1 0 7.75" />
  </svg>
);

const TranslationIcon = () => (
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <circle cx="12" cy="12" r="10" />
    <path d="M2 12h20M12 2a15.3 15.3 0 0 1 4 10 15.3 15.3 0 0 1-4 10 15.3 15.3 0 0 1-4-10 15.3 15.3 0 0 1 4-10z" />
  </svg>
);

const ChatbotIcon = () => (
  <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth="2">
    <path d="M21 15a2 2 0 0 1-2 2H7l-4 4V5a2 2 0 0 1 2-2h14a2 2 0 0 1 2 2z" />
    <circle cx="9" cy="10" r="1" fill="currentColor" />
    <circle cx="15" cy="10" r="1" fill="currentColor" />
  </svg>
);

// Feature data
const features = [
  {
    title: 'Personalized Learning',
    description: 'Customize your learning experience based on your skill level. Choose Beginner, Intermediate, or Advanced to get content tailored just for you.',
    icon: <PersonalizationIcon />,
    ctaText: 'Personalize Now',
    featureUrl: '/docs/module-1/chapter-01-intro-physical-ai',
  },
  {
    title: 'Read in Urdu',
    description: 'Access all chapters in Urdu. Our translation feature makes the content accessible to Urdu-speaking learners worldwide.',
    icon: <TranslationIcon />,
    ctaText: 'Translate to Urdu',
    featureUrl: '/docs/module-1/chapter-01-intro-physical-ai',
  },
  {
    title: 'AI Study Assistant',
    description: 'Ask questions about the textbook and get instant answers with citations. Your personal AI tutor available 24/7.',
    icon: <ChatbotIcon />,
    ctaText: 'Ask a Question',
    featureUrl: '',
    requiresAuth: false,
    opensChatbot: true, // Opens chatbot directly instead of navigating
  },
];

export default function InteractiveFeatures(): React.ReactElement {
  return (
    <section className={styles.section}>
      <div className="container">
        <div className={styles.header}>
          <h2 className={styles.sectionTitle}>Interactive Features</h2>
          <p className={styles.sectionSubtitle}>
            Enhance your learning with these powerful tools
          </p>
        </div>
        <div className={styles.featureGrid}>
          {features.map((feature, idx) => (
            <InteractiveFeatureCard
              key={feature.title}
              {...feature}
              animationDelay={idx * 100}
            />
          ))}
        </div>
      </div>
    </section>
  );
}
