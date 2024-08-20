# ROS2 Stereo Visual Odometry with Local Bundle Adjustment Using g2o
[![stereo-vo-local-BA.gif](https://i.postimg.cc/d18QFLJx/stereo-vo-local-BA.gif)](https://postimg.cc/tZ4GtRZ3)

An educational ROS2 stereo visual odometry system utilizing local bundle adjustment with the g2o library. This project is designed as a learning tool, offering insights into visual odometry and optimization techniques without aiming for high-performance enhancements. Based on concepts discussed in "SLAMBook2 Chapter 13".

## Features

- **Stereo Visual Odometry**: Utilizes stereo camera data to estimate the robot's motion.
- **Local Bundle Adjustment**: Employs the g2o library to optimize the positions of keypoints across multiple frames.
- **Educational Focus**: Designed to provide hands-on experience with visual odometry techniques without the complexity of performance-optimized code.

## Result

## Dependencies

- ROS2 (Humble or newer)
- g2o
- OpenCV
- Eigen
- Sophus

## Usage

Launch the stereo visual odometry node using the following ROS2 command:
```bash
ros2 launch stereo_visual_slam slam.launch.py
```