import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import Heading from '@theme/Heading';
import styles from './styles.module.css';

interface ChapterCardProps {
  number: string;
  title: string;
  duration: string;
  difficulty: string;
  wordCount: string;
  link: string;
}

export default function ChapterCard({
  number,
  title,
  duration,
  difficulty,
  wordCount,
  link,
}: ChapterCardProps): ReactNode {
  return (
    <div className={clsx('col col--4 padding-horiz--md', styles.chapterCard)}>
      <div className="card">
        <div className="card__header">
          <div className="text--center padding-horiz--md">
            <h3 className="text--center">
              <small>Chapter {number}</small><br/>{title}
            </h3>
          </div>
          <div className="text--center padding-horiz--md">
            <span className="badge badge--secondary">{duration}</span>
            <span className="badge badge--info margin-left--sm">{difficulty}</span>
          </div>
        </div>
        <div className="card__body">
          <p><strong>Word Count:</strong> {wordCount}</p>
        </div>
        <div className="card__footer text--center">
          <Link className="button button--primary button--block" to={link}>
            Read Chapter
          </Link>
        </div>
      </div>
    </div>
  );
}