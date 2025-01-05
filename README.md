# ROS-Based Autonomous Mobile Robot with Manipulator

## Project Overview
This project develops two differential-drive mobile robot models: one equipped with a robotic manipulator and one without. Both models perform mapping, localization, and navigation, while the manipulator-enabled robot additionally supports object recognition, localization, navigation, and automatic grasping.

## Features

### Mapping & Localization
- Utilizes **GMapping** for real-time SLAM and **AMCL** for localization.
- Optimized particle filter and odometry settings for enhanced accuracy.

### Navigation
- Implements path planning with **move_base** and **DWA local planner**.
- Adjustable global and local costmap settings ensure effective obstacle avoidance.

### Object Recognition
- Employs a depth camera for color-based object detection and localization.
- Computes object position in the map frame using depth point clouds and static coordinate transformations.

### Manipulator Control
- Integrated with **MoveIt** for motion planning.
- Supports automatic grasping using the **ROS Link Attacher** plugin in Gazebo.

### Integration
- Combines manipulator control and mobile base navigation for coordinated task execution.
- Supports both keyboard teleoperation and autonomous navigation.
