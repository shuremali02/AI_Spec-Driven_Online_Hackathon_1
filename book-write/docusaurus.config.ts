import {themes as prismThemes} from 'prism-react-renderer';
import type {Config} from '@docusaurus/types';
import type * as Preset from '@docusaurus/preset-classic';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

const config: Config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Master the fusion of AI and robotics - from ROS 2 fundamentals to conversational AI robots',
  favicon: 'img/favicon.svg',

  // Add Google Font for Urdu translation (Noto Nastaliq Urdu)
  headTags: [
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.googleapis.com',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'preconnect',
        href: 'https://fonts.gstatic.com',
        crossorigin: 'anonymous',
      },
    },
    {
      tagName: 'link',
      attributes: {
        rel: 'stylesheet',
        href: 'https://fonts.googleapis.com/css2?family=Noto+Nastaliq+Urdu:wght@400;500;600;700&display=swap',
      },
    },
  ],

  // Custom fields for configuration (accessible via useDocusaurusContext)
  customFields: {
    // RAG Chatbot API endpoint - change this for different environments
    apiBaseUrl: process.env.API_BASE_URL || 'https://shurem-ragchatbot.hf.space/v1',
    // Auth service URL - loaded from .env
    authBaseUrl: process.env.AUTH_BASE_URL || 'http://localhost:3001',
  },

  // Future flags, see https://docusaurus.io/docs/api/docusaurus-config#future
  future: {
    v4: true, // Improve compatibility with the upcoming Docusaurus v4
  },

  // Set the production url of your site here
  url: 'https://your-docusaurus-site.example.com',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For GitHub pages deployment, it is often '/<projectName>/'
  baseUrl: '/',

  // GitHub pages deployment config.
  // If you aren't using GitHub pages, you don't need these.
  organizationName: 'facebook', // Usually your GitHub org/user name.
  projectName: 'docusaurus', // Usually your repo name.

  onBrokenLinks: 'throw',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      {
        docs: {
          sidebarPath: './sidebars.ts',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/shuremali02/AI_Spec-Driven_Online_Hackathon_1',
        },
        theme: {
          customCss: './src/css/custom.css',
        },
      } satisfies Preset.Options,
    ],
  ],

  themeConfig: {
    // Replace with your project's social card
    image: 'img/docusaurus-social-card.jpg',
    colorMode: {
      respectPrefersColorScheme: true,
    },
    navbar: {
      title: 'Physical AI & Humanoid Robotics',
      logo: {
        alt: 'Physical AI & Humanoid Robotics Logo',
        src: 'img/logo.svg',
      },
      items: [
        {
          type: 'docSidebar',
          sidebarId: 'tutorialSidebar',
          position: 'left',
          label: 'Book',
        },
        {
          href: 'https://github.com/shuremali02/AI_Spec-Driven_Online_Hackathon_1',
          label: 'GitHub',
          position: 'right',
        },
      ],
    },
    footer: {
      style: 'dark',
      links: [
        {
          title: '📚 Book',
          items: [
            {
              label: 'Getting Started',
              to: '/docs/',
            },
            {
              label: 'Module 1: ROS 2',
              to: '/docs/module-1',
            },
          ],
        },
        {
          title: '🤖 Resources',
          items: [
            {
              label: 'ROS 2 Documentation',
              href: 'https://docs.ros.org/en/humble/',
            },
            {
              label: 'Gazebo Simulation',
              href: 'https://gazebosim.org/',
            },
            {
              label: 'NVIDIA Isaac Sim',
              href: 'https://developer.nvidia.com/isaac-sim',
            },
          ],
        },
        {
          title: '🔗 Connect',
          items: [
            {
              label: 'GitHub',
              href: 'https://github.com/shuremali02/AI_Spec-Driven_Online_Hackathon_1',
            },
            {
              label: 'Report Issues',
              href: 'https://github.com/shuremali02/AI_Spec-Driven_Online_Hackathon_1/issues',
            },
          ],
        },
      ],
      copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built for AI Spec-Driven Online Hackathon.`,
    },
    prism: {
      theme: prismThemes.github,
      darkTheme: prismThemes.dracula,
    },
  } satisfies Preset.ThemeConfig,
};

export default config;
