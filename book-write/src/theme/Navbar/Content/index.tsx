import React from 'react';
import Content from '@theme-original/Navbar/Content';
import type ContentType from '@theme/Navbar/Content';
import type { WrapperProps } from '@docusaurus/types';
import NavbarAuth from '@site/src/components/NavbarAuth';
import styles from './styles.module.css';

type Props = WrapperProps<typeof ContentType>;

export default function ContentWrapper(props: Props): JSX.Element {
  return (
    <>
      <Content {...props} />
      <div className={styles.navbarAuthWrapper}>
        <NavbarAuth variant="navbar" />
      </div>
    </>
  );
}
