# TurtleBot Trajectory, Controller, and ROS Interface in MATLAB

This repository contains code for trajectory planning, controller design, and interfacing with a TurtleBot using ROS (Robot Operating System) in MATLAB. The code allows the user to simulate and control a TurtleBot, design custom trajectories, and implement feedback controllers for navigation in a ROS-enabled environment.

## Table of Contents

- [Features](#features)
- [Requirements](#requirements)
- [Installation](#installation)
- [Usage](#usage)
- [Repository Structure](#repository-structure)
- [Troubleshooting](#troubleshooting)
- [Contributing](#contributing)
- [License](#license)

## Features

- **Trajectory Planning**: Implements cubic spline and other point-to-point trajectories for TurtleBot.
- **Control Design**: Provides kinematic controllers for differential-drive robots.
- **ROS Interface**: Includes functionality to communicate with the TurtleBot using ROS in MATLAB.
- **Simulation**: Test trajectories and controllers using MATLAB’s visualization tools before deploying on real hardware.

## Requirements

### Hardware
- TurtleBot with ROS-compatible hardware
- Wi-Fi connection to the TurtleBot

### Software
- MATLAB (R2021b or newer recommended)
- MATLAB ROS Toolbox
- MATLAB Robotics System Toolbox
- ROS (Noetic preferred)
- A working ROS installation on TurtleBot or a simulation environment

## Installation

1. Clone the repository:
    ```bash
    git clone https://github.com/username/turtlebot-trajectory-controller-ros.git
    ```
2. Open MATLAB and navigate to the cloned repository folder:
    ```matlab
    cd /path/to/turtlebot-trajectory-controller-ros
    ```
3. Ensure that the necessary toolboxes (ROS Toolbox, Robotics System Toolbox) are installed:
    ```matlab
    matlab.addons.installToolbox('Robotics System Toolbox');
    matlab.addons.installToolbox('ROS Toolbox');
    ```

4. Connect to the TurtleBot via ROS:
    ```matlab
    rosinit('http://192.168.50.112:11311'); % Replace with your TurtleBot's IP
    ```

5. Run the initialization script:
    ```matlab
    init_turtlebot_ros();
    ```

## Usage

1. **Trajectory Planning:**
   Define a point-to-point trajectory using cubic splines:
   ```matlab
   waypoints = [0, 0; 1, 1; 2, 0];
   trajectory = cubicSplineTrajectory(waypoints, time_vector);
