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
  tutorialSidebar: [
    'course-intro',
    {
      type: 'category',
      label: 'Foundation',
      items: [
        'module-0-foundations/module-0-intro',
        'module-0-foundations/what-is-physical-ai',
        'module-0-foundations/why-physical-ai-matters',
        'module-0-foundations/humanoid-robotics-landscape',
        'module-0-foundations/module-0-learning-path',
        'module-0-foundations/module-0-summary',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Prerequisites',
      items: [
        'module-0-prerequisites/module-0-prerequisites-intro',
        'module-0-prerequisites/learning-path',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Module 1: The Robotic Nervous System (ROS 2)',
      items: [
        'module-1-ros2/module-1-intro',
        'module-1-ros2/ros2-architecture-overview',
        'module-1-ros2/module-1-nodes-topics-services',
        'module-1-ros2/module-1-actions-and-timers',
        'module-1-ros2/module-1-python-rclpy',
        'module-1-ros2/module-1-launch-parameters',
        'module-1-ros2/module-1-summary',
        {
          type: 'category',
          label: 'Labs',
          items: [
            'module-1-ros2/module-1-lab-1-1',
            'module-1-ros2/module-1-lab-1-2',
            'module-1-ros2/module-1-lab-1-3',
          ],
        },
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Module 2: The Digital Twin (Gazebo & Unity)',
      items: [
        'module-2-simulation/module-2-intro',
        'module-2-simulation/module-2-gazebo-basics',
        'module-2-simulation/module-2-urdf',
        'module-2-simulation/module-2-sensors',
        'module-2-simulation/module-2-unity',
        'module-2-simulation/module-2-sim-to-real',
        'module-2-simulation/module-2-summary',
        {
          type: 'category',
          label: 'Labs',
          items: [
            'module-2-simulation/module-2-lab-1',
            'module-2-simulation/module-2-lab-2',
            'module-2-simulation/module-2-lab-3',
          ],
        },
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
      items: [
        'module-3-isaac/module-3-intro',
        'module-3-isaac/module-3-isaac-overview',
        'module-3-isaac/module-3-environments',
        'module-3-isaac/module-3-slam',
        'module-3-isaac/module-3-isaac-ros',
        'module-3-isaac/module-3-detection',
        'module-3-isaac/module-3-summary',
        {
          type: 'category',
          label: 'Labs',
          items: [
            'module-3-isaac/module-3-lab-1',
            'module-3-isaac/module-3-lab-2',
            'module-3-isaac/module-3-lab-3',
          ],
        },
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Module 4: Vision-Language-Action (VLA)',
      items: [
        'module-4-vla/module-4-intro',
        'module-4-vla/module-4-vla-complete',
        'module-4-vla/module-4-summary',
        {
          type: 'category',
          label: 'Labs',
          items: [
            'module-4-vla/module-4-lab-1',
            'module-4-vla/module-4-lab-2',
            'module-4-vla/module-4-lab-3',
          ],
        },
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Capstone Project',
      items: [
        'capstone/capstone-requirements',
        'capstone/capstone-grading-rubrics',
        'capstone/capstone-example-projects',
        'capstone/capstone-deployment-guide',
      ],
      collapsed: true,
    },
    {
      type: 'category',
      label: 'Hardware Setup',
      items: [
        'hardware-setup/hw-minimum-requirements',
        'hardware-setup/hw-recommended-setup',
        'hardware-setup/hw-unitree-deployment',
        'hardware-setup/hw-safety-protocols',
      ],
      collapsed: true,
    },
    'glossary',
  ],
};

export default sidebars;
