import type {ReactNode} from 'react';
import clsx from 'clsx';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import Heading from '@theme/Heading';

import HeroVideoBackground from '@site/src/components/HeroVideoBackground';
import styles from './index.module.css';

// Module data
const modules = [
  {
    id: 1,
    title: 'Module 1: The Robotic Nervous System (ROS 2)',
    description: 'Learn the fundamentals of ROS 2 including nodes, topics, services, and message passing.',
    link: '/docs/module-1',
    icon: '🤖'
  },
  {
    id: 2,
    title: 'Module 2: The Digital Twin (Gazebo & Unity)',
    description: 'Explore physics simulation, environment building, and sensor simulation in Gazebo and Unity.',
    link: '/docs/module-2',
    icon: '📡'
  },
  {
    id: 3,
    title: 'Module 3: The AI-Robot Brain (NVIDIA Isaac™)',
    description: 'Master NVIDIA Isaac platform, Isaac Sim, and Nav2 path planning for intelligent robots.',
    link: '/docs/module-3',
    icon: '🧠'
  },
  {
    id: 4,
    title: 'Module 4: Vision-Language-Action (VLA)',
    description: 'Build conversational robots with LLM integration, speech recognition, and cognitive planning.',
    link: '/docs/module-4',
    icon: '💬'
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

// Hardware requirements data
const hardwareRequirements = [
  {
    id: 1,
    title: 'The "Digital Twin" Workstation',
    description: 'High-performance computing workstation with NVIDIA RTX 4090 GPU, 64GB+ RAM, Intel i9 or AMD Ryzen 9 processor for physics simulation and AI model training.'
  },
  {
    id: 2,
    title: 'The "Physical AI" Edge Kit',
    description: 'NVIDIA Jetson Orin AGX development kit with stereo cameras, IMU, and custom sensor integration for real-time AI inference on robots.'
  },
  {
    id: 3,
    title: 'The Robot Lab Options',
    description: 'Compatible humanoid robots (Unitree H1, Tesla Optimus, or custom platforms) with ROS 2 support and safety equipment for lab deployment.'
  }
];

// Hackathon features data
const hackathonFeatures = [
  {
    id: 1,
    title: 'RAG Chatbot',
    description: 'Retrieval-Augmented Generation chatbot for interactive learning with textbook content.'
  },
  {
    id: 2,
    title: 'Signup/Signin',
    description: 'User authentication system for personalized learning experiences.'
  },
  {
    id: 3,
    title: 'Personalization',
    description: 'Custom learning paths and progress tracking based on user preferences.'
  },
  {
    id: 4,
    title: 'Urdu Translation',
    description: 'Complete Urdu language support to make the content accessible to a wider audience.'
  }
];

function HomepageHeader() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <header className={clsx('hero hero--primary', styles.heroBanner)}>
      <HeroVideoBackground />
      <div className={clsx('container', styles.heroContent)}>
        <div className={styles.heroText}>
          <Heading as="h1" className="hero__title">{siteConfig.title}</Heading>
          <p className="hero__subtitle">{siteConfig.tagline}</p>
          <div className={styles.buttons}>
            <Link className="button button--primary button--lg margin-right--md" to="/docs/intro">Start Learning</Link>
            <Link className="button button--secondary button--lg" to="#modules">Explore Modules</Link>
          </div>
        </div>
      </div>
    </header>
  );
}

function ModuleCards() {
  return (
    <section className={clsx(styles.modulesSection, 'features-section fade-in')}>
      <div className="container">
        <div id="modules" className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('margin-bottom--lg', styles.sectionTitle, 'section-title fade-in')}>
              Course Modules
            </Heading>
          </div>
        </div>
        <div className="modules-grid">
          {modules.map((module, index) => (
            <div key={module.id} className={`module-card fade-in scale-hover ${index % 2 === 0 ? 'slide-in-left' : 'slide-in-right'}`}>
              <div className={styles.moduleIcon}>
                <span className={styles.icon}>{module.icon}</span>
                <Heading as="h3">{module.title}</Heading>
              </div>
              <p>{module.description}</p>
              <Link to={module.link} className="button button--primary button--sm pulse">
                Explore Module
              </Link>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function WeeklyBreakdown() {
  return (
    <section className={clsx(styles.weeklySection, 'features-section fade-in')}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('margin-bottom--lg', styles.sectionTitle, 'section-title fade-in')}>
              Your Learning Path: Course Breakdown
            </Heading>
          </div>
        </div>
        <div className="timeline">
          {weeklyBreakdown.map((week, index) => (
            <div key={index} className={`timeline-item fade-in ${index % 2 === 0 ? 'slide-in-left' : 'slide-in-right'} ${index % 2 === 0 ? '' : 'timeline-item--right'}`}>
              <div className="timeline-content scale-hover">
                <Heading as="h3" className={styles.weekTitle}>
                  {week.weeks}: {week.title}
                </Heading>
                <p>{week.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function HardwareRequirements() {
  return (
    <section className={clsx(styles.weeklySection, 'features-section fade-in')}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('margin-bottom--lg', styles.sectionTitle, 'section-title fade-in')}>
              Lab Infrastructure & Hardware Requirements
            </Heading>
          </div>
        </div>
        <div className="row">
          {hardwareRequirements.map((hardware, index) => (
            <div key={hardware.id} className={`col col--4 margin-bottom--lg fade-in fade-in-delay-${index + 1}`}>
              <div className="hardware-card scale-hover glow">
                <Heading as="h3" className={styles.hardwareTitle}>
                  {hardware.title}
                </Heading>
                <p>{hardware.description}</p>
              </div>
            </div>
          ))}
        </div>
      </div>
    </section>
  );
}

function HackathonFeatures() {
  return (
    <section className={clsx(styles.weeklySection, 'features-section fade-in')}>
      <div className="container">
        <div className="row">
          <div className="col col--12">
            <Heading as="h2" className={clsx('margin-bottom--lg', styles.sectionTitle, 'section-title fade-in')}>
              Hackathon Features & Scoring
            </Heading>
          </div>
        </div>
        <div className="row">
          {hackathonFeatures.map((feature, index) => (
            <div key={feature.id} className={`col col--3 margin-bottom--lg fade-in fade-in-delay-${index + 1}`}>
              <div className="hardware-card scale-hover glow">
                <Heading as="h3" className={styles.hardwareTitle}>
                  {feature.title}
                </Heading>
                <p>{feature.description}</p>
              </div>
            </div>
          ))}
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
      <main className="fade-in">
        <ModuleCards />
        <WeeklyBreakdown />
        <HardwareRequirements />
        <HackathonFeatures />
      </main>
    </Layout>
  );
}
