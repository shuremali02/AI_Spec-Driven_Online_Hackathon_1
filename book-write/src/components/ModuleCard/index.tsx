import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

interface ModuleCardProps {
  icon: string;
  number: string;
  title: string;
  duration: string;
  purpose: string;
  outcomes: string[];
  link: string;
}

export default function ModuleCard({
  icon,
  number,
  title,
  duration,
  purpose,
  outcomes,
  link,
}: ModuleCardProps): ReactNode {
  return (
    <div className={styles.moduleCard}>
      <div className="card">
        <div className="card__header">
          <div className="text--center padding-horiz--md">
            <h3 className="text--center">
              {icon} <small>Module {number}</small><br/>{title}
            </h3>
          </div>
          <div className="text--center padding-horiz--md">
            <span className="badge badge--secondary">{duration}</span>
          </div>
        </div>
        <div className="card__body">
          <p><strong>Purpose:</strong> {purpose}</p>
          <p><strong>Key Learning Outcomes:</strong></p>
          <ul>
            {outcomes.map((outcome, index) => (
              <li key={index}>{outcome}</li>
            ))}
          </ul>
        </div>
        <div className="card__footer text--center">
          <Link className="button button--primary button--block" to={link}>
            Explore Module
          </Link>
        </div>
      </div>
    </div>
  );
}