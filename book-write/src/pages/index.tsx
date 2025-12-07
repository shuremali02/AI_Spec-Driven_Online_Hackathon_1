import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageFeatures from '@site/src/components/HomepageFeatures';
import Heading from '@theme/Heading';

import styles from './index.module.css';

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <Heading as="h1" className="hero__title">
          {siteConfig.title}
        </Heading>
        <p className="hero__subtitle">{siteConfig.tagline}</p>
        <div className={styles.buttons}>
          <Link
            className="button button--secondary button--lg"
            to="/docs/">
            Start Learning - 13 Week Course
          </Link>
        </div>
      </div>
    </header>
  );
}

function WelcomeSection() {
  return (
    <section className={styles.welcomeSection}>
      <div className="container">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <div className="text--center padding-horiz--md">
              <h2>Welcome to Physical AI & Humanoid Robotics</h2>
              <p>
                Welcome to the comprehensive 13-week course that bridges the gap between advanced AI algorithms and their physical embodiment in humanoid robots.
                This course will take you from ROS 2 fundamentals to building conversational AI robots with Vision-Language-Action (VLA) systems.
              </p>
              <p>
                Whether you're a beginner in robotics or an experienced developer looking to expand your skills,
                this course provides a structured learning path with hands-on projects and real-world applications.
              </p>
            </div>
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Physical AI & Humanoid Robotics`}
      description="Learn Physical AI & Humanoid Robotics - from ROS 2 fundamentals to conversational AI robots">
      <HomepageHeader />
      <main>
        <WelcomeSection />
        <HomepageFeatures />
      </main>
    </Layout>
  );
}
