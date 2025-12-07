const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro', // Refers to book-write/docs/index.md (homepage)
      label: '🏠 Home',
    },

    {
      type: 'category',
      label: 'Chapter 1:Foundations',
      collapsed: false,
      items: [
        'module-1/module-1-overview',
        'module-1/chapter-01-intro-physical-ai',
        
      ],
    },

    {
      type: 'category',
      label: 'Chapter 2: ROS2 Fandamentals',
      items: [
        'module-1/chapter-02-ros2-architecture',

      ],
    },

    {
      type: 'category',
      label: 'Chapter 3: Building Your First ROS 2 Nodes',
      items: [
        'module-1/chapter-03-first-nodes',

      ],
    },

    {
      type: 'category',
      label: 'Chapter 4: URDF Robot Descriptions',
      items: [
        'module-1/chapter-04-urdf',

      ],
    },

    {
      type: 'category',
      label: 'Tutorial - Extras',
      items: [
        'tutorial-extras/manage-docs-versions',
        'tutorial-extras/translate-your-site',
      ],
    },
  ],
};

export default sidebars;
