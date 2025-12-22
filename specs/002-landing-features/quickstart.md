# Quickstart: Landing Page Features Section

**Feature**: 002-landing-features
**Estimated Time**: 20-30 minutes

## Prerequisites

- [ ] Node.js 18+ installed
- [ ] Project dependencies installed (`npm install` in book-write/)
- [ ] Development server can start (`npm run start`)
- [ ] Auth system working (useAuth hook available)

## Quick Implementation Steps

### Step 1: Create Component Directory

```bash
mkdir -p book-write/src/components/InteractiveFeatures
```

### Step 2: Create InteractiveFeatureCard Component

Create `book-write/src/components/InteractiveFeatures/InteractiveFeatureCard.tsx`:

```tsx
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
}

export default function InteractiveFeatureCard({
  title,
  description,
  icon,
  ctaText,
  featureUrl,
  animationDelay = 0,
}: InteractiveFeatureCardProps): React.ReactElement {
  const { user } = useAuth();

  // Anonymous users go to auth page, logged-in users go to feature
  const targetUrl = user ? featureUrl : '/auth';
  const buttonText = user ? ctaText : 'Sign Up to Access';

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
      <Link to={targetUrl} className={styles.ctaButton}>
        {buttonText}
      </Link>
    </article>
  );
}
```

### Step 3: Create Main Section Component

Create `book-write/src/components/InteractiveFeatures/index.tsx`:

```tsx
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
    featureUrl: '/docs/module-1/chapter-01-intro-physical-ai',
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
```

### Step 4: Create Styles

Create `book-write/src/components/InteractiveFeatures/styles.module.css`:

```css
.section {
  padding: 4rem 0;
  background: linear-gradient(180deg, var(--ifm-background-color) 0%, var(--ifm-background-surface-color) 100%);
}

.header {
  text-align: center;
  margin-bottom: 3rem;
}

.sectionTitle {
  font-size: 2.5rem;
  font-weight: 700;
  margin-bottom: 0.5rem;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
  background-clip: text;
}

.sectionSubtitle {
  font-size: 1.2rem;
  color: var(--ifm-color-emphasis-600);
}

.featureGrid {
  display: grid;
  grid-template-columns: repeat(3, 1fr);
  gap: 2rem;
  max-width: 1200px;
  margin: 0 auto;
}

@media (max-width: 996px) {
  .featureGrid {
    grid-template-columns: repeat(2, 1fr);
  }
}

@media (max-width: 768px) {
  .featureGrid {
    grid-template-columns: 1fr;
  }

  .sectionTitle {
    font-size: 2rem;
  }
}

/* Card Styles */
.card {
  background: var(--ifm-background-surface-color);
  border: 1px solid var(--ifm-color-emphasis-200);
  border-radius: 16px;
  padding: 2rem;
  text-align: center;
  transition: all 0.3s ease;
  animation: fadeInUp 0.6s ease-out both;
}

.card:hover {
  transform: translateY(-8px);
  box-shadow: 0 12px 40px rgba(102, 126, 234, 0.15);
  border-color: var(--ifm-color-primary);
}

@keyframes fadeInUp {
  from {
    opacity: 0;
    transform: translateY(20px);
  }
  to {
    opacity: 1;
    transform: translateY(0);
  }
}

.iconContainer {
  width: 64px;
  height: 64px;
  margin: 0 auto 1.5rem;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  border-radius: 50%;
  display: flex;
  align-items: center;
  justify-content: center;
}

.iconContainer svg {
  width: 32px;
  height: 32px;
  color: white;
}

.title {
  font-size: 1.5rem;
  font-weight: 600;
  margin-bottom: 0.75rem;
  color: var(--ifm-heading-color);
}

.description {
  font-size: 1rem;
  color: var(--ifm-color-emphasis-600);
  line-height: 1.6;
  margin-bottom: 1.5rem;
}

.ctaButton {
  display: inline-block;
  padding: 0.75rem 1.5rem;
  background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
  color: white !important;
  border-radius: 8px;
  font-weight: 600;
  text-decoration: none;
  transition: all 0.2s ease;
}

.ctaButton:hover {
  transform: scale(1.05);
  box-shadow: 0 4px 15px rgba(102, 126, 234, 0.4);
  text-decoration: none;
}

/* Dark mode */
[data-theme='dark'] .section {
  background: linear-gradient(180deg, var(--ifm-background-color) 0%, #1a1a2e 100%);
}

[data-theme='dark'] .card {
  background: #1e1e2e;
  border-color: var(--ifm-color-emphasis-300);
}

[data-theme='dark'] .card:hover {
  box-shadow: 0 12px 40px rgba(102, 126, 234, 0.25);
}

[data-theme='dark'] .sectionSubtitle {
  color: var(--ifm-color-emphasis-500);
}
```

### Step 5: Add to Landing Page

Update `book-write/src/pages/index.tsx`:

```tsx
import InteractiveFeatures from '@site/src/components/InteractiveFeatures';

// ... existing code ...

export default function Home(): ReactNode {
  return (
    <Layout>
      <HomepageHeader />
      <main>
        <WelcomeSection />
        <HomepageFeatures />
        <InteractiveFeatures />  {/* Add this line */}
      </main>
    </Layout>
  );
}
```

## Testing

### Manual Tests

1. **Anonymous User View**
   - Open landing page without logging in
   - Scroll to Interactive Features section
   - Verify all 3 cards visible with icons, titles, descriptions
   - Verify CTA buttons show "Sign Up to Access"
   - Click any CTA → should go to `/auth` page

2. **Logged-in User View**
   - Log in with test account
   - Navigate to landing page
   - Verify CTA buttons show feature-specific text
   - Click Personalize Now → should go to chapter page
   - Click Translate to Urdu → should go to chapter page
   - Click Ask a Question → should go to chapter page

3. **Dark Mode**
   - Toggle dark mode
   - Verify section background changes
   - Verify card backgrounds and borders adapt
   - Verify text remains readable

4. **Responsive**
   - Resize browser to tablet width (~900px) → 2 column grid
   - Resize to mobile width (~500px) → 1 column stack
   - Verify cards remain fully visible

## Done!

After implementation, run the development server and test:

```bash
cd book-write
npm run start
```

Navigate to `http://localhost:3000` and scroll down to see the Interactive Features section.
