// @ts-check

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.

 @type {import('@docusaurus/plugin-content-docs').SidebarsConfig}
 */
const sidebars = {
  tutorialSidebar: [
    'intro',
    {
      type: 'category',
      label: 'Module 1: ROS 2 Core for Humanoid Control',
      collapsed: true,
      items: [
        'module-1/chapter-1-ros2-core',
        'module-1/chapter-2-python-bridging',
        'module-1/chapter-3-humanoid-urdf',
      ],
    },
    {
      type: 'category',
      label: 'Module 2: Physical AI Concepts',
      collapsed: true,
      items: [
        'module-2/perception',
        'module-2/planning',
        'module-2/control',
      ],
    },
    {
      type: 'category',
      label: 'Module 3: Advanced Humanoid Control',
      collapsed: true,
      items: [
        'module-3/locomotion',
        'module-3/manipulation',
        'module-3/balance',
      ],
    },
    'tutorials/hands-on-exercises',
    'appendices/glossary',
    'appendices/ros2-cheatsheet',
  ],
};

export default sidebars;
