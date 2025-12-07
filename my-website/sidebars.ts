import type {SidebarsConfig} from '@docusaurus/plugin-content-docs';

// This runs in Node.js - Don't use client-side code here (browser APIs, JSX...)

/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each doc of that group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */
const sidebars: SidebarsConfig = {
  // Manual sidebar configuration for the Physical AI textbook
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Introduction',
      items: ['intro'],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1/index',
        'module-1/nodes',
        'module-1/topics',
        'module-1/services',
        'module-1/rclpy',
        'module-1/urdf',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 2: Simulation Environments (Gazebo)',
      items: [
        // Will be populated in future modules
      ],
      link: {type: 'doc', id: 'intro'}, // Placeholder link to avoid error
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Module 3: Advanced Simulation (Isaac Sim)',
      items: [
        // Will be populated in future modules
      ],
      link: {type: 'doc', id: 'intro'}, // Placeholder link to avoid error
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action Models',
      items: [
        // Will be populated in future modules
      ],
      link: {type: 'doc', id: 'intro'}, // Placeholder link to avoid error
      collapsed: true,
    },
  ],
};

export default sidebars;
