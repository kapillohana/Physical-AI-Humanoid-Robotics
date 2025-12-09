import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import styles from './index.module.css';

// Module data
const modules = [
  {
    id: 1,
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    description: 'Learn the fundamentals of ROS 2 including nodes, topics, services, and message passing.',
    link: '/docs/module-1'
  },
  {
    id: 2,
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    description: 'Explore physics simulation, environment building, and sensor simulation in Gazebo and Unity.',
    link: '/docs/module-2'
  },
  {
    id: 3,
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
    description: 'Master NVIDIA Isaac platform, Isaac Sim, and Nav2 path planning for intelligent robots.',
    link: '/docs/module-3'
  },
  {
    id: 4,
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: 'Build conversational robots with LLM integration, speech recognition, and cognitive planning.',
    link: '/docs/module-4'
  }
];

// Weekly breakdown data
const weeklyBreakdown = [
  {
    weeks: 'Weeks 1-2',
    title: 'Introduction to Physical AI',
    description: 'Foundations of Physical AI and embodied intelligence; From digital AI to robots that understand physical laws; Overview of humanoid robotics landscape; Sensor systems: LIDAR, cameras, IMUs, force/torque sensors.'
  },
  {
    weeks: 'Weeks 3-5',
    title: 'ROS 2 Fundamentals',
    description: 'ROS 2 architecture and core concepts; Nodes, topics, services, and actions; Building ROS 2 packages with Python; Launch files and parameter management.'
  },
  {
    weeks: 'Weeks 6-7',
    title: 'Robot Simulation with Gazebo',
    description: 'Gazebo simulation environment setup; URDF and SDF robot description formats; Physics simulation and sensor simulation; Introduction to Unity for robot visualization.'
  },
  {
    weeks: 'Weeks 8-10',
    title: 'NVIDIA Isaac Platform',
    description: 'NVIDIA Isaac SDK and Isaac Sim; AI-powered perception and manipulation; Reinforcement learning for robot control; Sim-to-real transfer techniques.'
  },
  {
    weeks: 'Weeks 11-12',
    title: 'Humanoid Robot Development',
    description: 'Humanoid robot kinematics and dynamics; Bipedal locomotion and balance control; Manipulation and grasping with humanoid hands; Natural human-robot interaction design.'
  },
  {
    weeks: 'Week 13',
    title: 'Conversational Robotics',
    description: 'Integrating GPT models for conversational AI in robots; Speech recognition and natural language understanding; Multi-modal interaction: speech, gesture.'
  }
];

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <div className="container">
        <div className={styles.heroGrid}>
          {/* Left Column - Image Placeholder */}
          <div className={styles.heroImage}>
            {/* <!-- Image placeholder for technical/robotic illustration --> */}
            <div className={styles.imagePlaceholder}>
              <svg
                className={styles.placeholderSvg}
                viewBox="0 0 24 24"
                xmlns="http://www.w3.org/2000/svg"
              >
                <rect width="24" height="24" fill="var(--ifm-color-primary-lightest)" opacity="0.2" rx="4"/>
                <circle cx="12" cy="12" r="8" fill="var(--ifm-color-primary)" opacity="0.6"/>
                <path d="M8 12h8M12 8v8" stroke="var(--ifm-color-primary)" strokeWidth="2" strokeLinecap="round"/>
              </svg>
            </div>
          </div>

          {/* Right Column - Text */}
          <div className={styles.heroText}>
            <Heading as="h1" className="hero__title">
              {siteConfig.title}
            </Heading>
            <p className="hero__subtitle">{siteConfig.tagline}</p>
            <div className={styles.buttons}>
              <Link
                className="button button--secondary button--lg"
                to="/docs/intro">
                Get Started
              </Link>
            </div>
          </div>
        </div>
      </div>
    </header>
  );
}

function ModuleCards() {
  return (
    <section className={styles.modulesSection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('margin-bottom--lg', styles.sectionTitle)}>
              Course Modules
            </Heading>
          </div>
        </div>
        <div className="row">
          {modules.map((module) => (
            <div key={module.id} className="col col--6 margin-bottom--lg">
              <div className="module-card">
                <Heading as="h3">{module.title}</Heading>
                <p>{module.description}</p>
                <Link to={module.link} className="button button--primary button--sm">
                  Explore Module
                </Link>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function WeeklyBreakdown() {
  return (
    <section className={styles.weeklySection}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('margin-bottom--lg', styles.sectionTitle)}>
              Course Weekly Breakdown
            </Heading>
          </div>
        </div>
        <div className="row">
          <div className="col col--12">
            {weeklyBreakdown.map((week, index) => (
              <div key={index} className="weekly-breakdown-card">
                <Heading as="h3" className={styles.weekTitle}>
                  {week.weeks}: {week.title}
                </Heading>
                <p>{week.description}</p>
              </div>
            ))}
          </div>
        </div>
      </div>
    </section>
  );
}

export default function Home(): ReactNode {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Welcome to ${siteConfig.title}`}
      description="A comprehensive textbook on Physical AI & Humanoid Robotics">
      <HomepageHeader />
      <main>
        <ModuleCards />
        <WeeklyBreakdown />
      </main>
    </Layout>
  );
}
