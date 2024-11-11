# iRobot Create-3 Path Planning Project

## Overview
This project involves path planning for the iRobot Create-3 robot in a simulated environment. The goal is to program the robot to:

1. **Undock** from its docking station using the provided undocking service.
2. **Move from Point A to Point B**: 
   - **Start Position**: The robot's initial position after undocking.
   - **End Position**: A target point read from a YAML configuration file.
   - The robot's trajectory should be creative and **not follow a straight line or circular arc**.
3. **[Optional] Dock**: After reaching the target point, the robot may redock at its original docking station.

## Requirements
- **Operating System**: Ubuntu 22 with ROS2 Humble installed.
- **Simulator**: Create-3 Gazebo simulator.

## Setup Instructions
1. Clone and build the Create-3 simulator repository from iRobot.
2. Launch the Create-3 simulation in Gazebo using:
   ```bash
   ros2 launch irobot_create_gazebo_bringup create3_gazebo.launch.py
   ```

