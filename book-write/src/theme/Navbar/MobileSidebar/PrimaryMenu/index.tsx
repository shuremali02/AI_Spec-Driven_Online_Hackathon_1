import React from 'react';
import PrimaryMenu from '@theme-original/Navbar/MobileSidebar/PrimaryMenu';
import type PrimaryMenuType from '@theme/Navbar/MobileSidebar/PrimaryMenu';
import type { WrapperProps } from '@docusaurus/types';
import NavbarAuth from '@site/src/components/NavbarAuth';

type Props = WrapperProps<typeof PrimaryMenuType>;

// Callback to close sidebar after navigation
function useSidebarClose() {
  const closeSidebar = () => {
    // Docusaurus closes sidebar via clicking the backdrop,
    // but we can trigger the same effect by clicking outside
    const backdrop = document.querySelector('.navbar-sidebar__backdrop');
    if (backdrop instanceof HTMLElement) {
      backdrop.click();
    }
  };
  return closeSidebar;
}

export default function PrimaryMenuWrapper(props: Props): JSX.Element {
  const closeSidebar = useSidebarClose();

  return (
    <>
      {/* Book/navigation links first, then auth section below */}
      <PrimaryMenu {...props} />
      <NavbarAuth variant="sidebar" onNavigate={closeSidebar} />
    </>
  );
}
