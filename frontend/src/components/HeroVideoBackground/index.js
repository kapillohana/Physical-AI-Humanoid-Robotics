import React from 'react';
import styles from './HeroVideoBackground.module.css';

const HeroVideoBackground = () => {
  return (
    <div className={styles.videoContainer}>
      {/* Vimeo embed as background */}
      <div className={styles.vimeoBackground}>
        <div style={{position:'relative', width: '100%', paddingTop: '56.25%'}}>
          <iframe
            src="https://player.vimeo.com/video/1147155298?background=1&muted=1&autoplay=1&loop=1&byline=0&title=0"
            frameBorder="0"
            allow="autoplay; fullscreen; picture-in-picture"
            allowFullScreen
            style={{position:'absolute', top:0, left:0, width:'100%', height:'100%', pointerEvents: 'none'}}
          ></iframe>
        </div>
      </div>

      {/* Video element as fallback (commented out for now)
      <video
        className={styles.backgroundVideo}
        autoPlay
        muted
        loop
        playsInline
        preload="metadata"
        poster="data:image/svg+xml,%3Csvg xmlns='http://www.w3.org/2000/svg' width='1920' height='1080' viewBox='0 0 1920 1080'%3E%3Crect width='1920' height='1080' fill='%23222222'/%3E%3Ctext x='50%25' y='50%25' dominant-baseline='middle' text-anchor='middle' font-size='24' fill='%23ffffff'%3EPhysical AI %26 Humanoid Robotics Background%3C/text%3E%3C/svg%3E"
      >
        <source
          src="/static/video/chess-robot.mp4"
          type="video/mp4"
        />
      </video>
      */}

      <div className={styles.backgroundOverlay}></div>
    </div>
  );
};

export default HeroVideoBackground;