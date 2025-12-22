import type {ReactNode} from 'react';
import FeatureCard from '@site/src/components/FeatureCard';
import styles from './styles.module.css';

type FeatureItem = {
  title: string;
  Svg: React.ComponentType<React.ComponentProps<'svg'>>;
  description: ReactNode;
};

const FeatureList: FeatureItem[] = [
  {
    title: 'Physical AI & Embodied Intelligence',
    Svg: require('@site/static/img/undraw_docusaurus_mountain.svg').default,
    description: (
      <>
        Learn how artificial intelligence transcends the digital realm to inhabit physical bodies through sophisticated humanoid robots.
      </>
    ),
  },
  {
    title: 'ROS 2 Fundamentals',
    Svg: require('@site/static/img/undraw_docusaurus_tree.svg').default,
    description: (
      <>
        Master ROS 2 middleware for robust robot communication and control using Python and rclpy.
      </>
    ),
  },
  {
    title: 'Complete Learning Path',
    Svg: require('@site/static/img/undraw_docusaurus_react.svg').default,
    description: (
      <>
        13-week structured course from ROS 2 basics to Vision-Language-Action (VLA) systems for conversational robots.
      </>
    ),
  },
];

export default function HomepageFeatures(): ReactNode {
  return (
    <section className={styles.features}>
      <div className="container">
        <div className={styles.header}>
          <h2 className={styles.sectionTitle}>What You'll Learn</h2>
          <p className={styles.sectionSubtitle}>
            Master Physical AI and Humanoid Robotics
          </p>
        </div>
        <div className={styles.featureGrid}>
          {FeatureList.map((props, idx) => (
            <FeatureCard
              key={props.title}
              title={props.title}
              description={props.description}
              Svg={props.Svg}
              animationDelay={idx * 100}
            />
          ))}
        </div>
      </div>
    </section>
  );
}
