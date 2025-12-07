const sidebars = {
  tutorialSidebar: [
    {
      type: 'doc',
      id: 'intro', // Refers to book-write/docs/index.md (homepage)
      label: '🏠 Home',
    },

    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-1/index',  // Module 1 overview page
        'module-1/chapter-01-intro-physical-ai',
        'module-1/chapter-02-ros2-architecture',
        'module-1/chapter-03-first-nodes',
        'module-1/chapter-04-urdf',
      ],
    },

    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-2/index',  // Module 2 overview page

      ],
    },

    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac Sim)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-3/index',  // Module 3 overview page

      ],
    },

    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      collapsible: true,
      collapsed: false,
      items: [
        'module-4/index',  // Module 4 overview page

      ],
    },

  ],
};

export default sidebars;
