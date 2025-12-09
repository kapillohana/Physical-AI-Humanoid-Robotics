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
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2/index',
        'module-2/gazebo-fundamentals',
        'module-2/urdf-sdf-simulation',
        'module-2/sensor-simulation',
        'module-2/unity-integration',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3/index',
        'module-3/isaac-sim-fundamentals',
        'module-3/isaac-ros-hardware-accel',
        'module-3/nav2-path-planning',
        'module-3/sim-to-real-transfer',
      ],
      collapsed: false,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4/index',
        'module-4/4-1-voice-to-ros-action',
        'module-4/4-2-llm-as-planner',
        'module-4/4-3-capstone-project',
        'module-4/4-4-ethics-and-the-future',
      ],
      collapsed: false,
    },
  ],
};

export default sidebars;
