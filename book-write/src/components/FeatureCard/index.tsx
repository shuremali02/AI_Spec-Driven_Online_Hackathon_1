import React from 'react';
import clsx from 'clsx';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

interface FeatureCardProps {
  title: string;
  description: React.ReactNode;
  Svg: React.ComponentType<React.SVGProps<SVGSVGElement>>;
  animationDelay?: number;
  className?: string;
}

export default function FeatureCard({
  title,
  description,
  Svg,
  animationDelay = 0,
  className,
}: FeatureCardProps): React.ReactElement {
  return (
    <article
      className={clsx(styles.card, className)}
      style={{ animationDelay: `${animationDelay}ms` }}
    >
      <div className={styles.iconContainer}>
        <Svg className={styles.icon} role="img" aria-hidden="true" />
      </div>
      <div className={styles.content}>
        <Heading as="h3" className={styles.title}>
          {title}
        </Heading>
        <p className={styles.description}>{description}</p>
      </div>
    </article>
  );
}
