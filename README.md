# Robot GUI Interface 
This project implements a graphical user interface (GUI) for robot control using OpenCV, ROS (Robot Operating System), and the CVUI library. The system provides teleoperation controls, real-time odometry visualization, and distance tracking capabilities for robotic applications.
## Key Features
### Core Functionality
🎮 Interactive Robot Control GUI with OpenCV-based interface

🤖 ROS Integration for real-time robot communication

🔼🔽 Teleoperation Controls (forward, backward, left, right, stop)

📍 Odometry Visualization displaying real-time robot position (x, y, z)

📏 Distance Tracking Service that calculates and displays traveled distance

🧾 Robot Information Display (serial number, IP, firmware, payload, etc.)

### Technical Components
- distance_tracker_service.cpp – ROS service that tracks and reports distance traveled

- robot_gui.cpp – Main GUI application with control interface

- cvui.h – Lightweight GUI library for OpenCV (custom implementation)

## Technical Stack
- C++14/17

- ROS (Robot Operating System)

- OpenCV for computer vision and GUI rendering

- CVUI for lightweight UI components

- Multi-threading for concurrent ROS and GUI operations

## Installation & Usage
### Prerequisites
- ROS (Noetic or later)

- OpenCV 4.x

- CMake 3.10+

### Build Instructions
```bash 
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/adityaharidas/robot_gui.git
cd ..
catkin_make
source devel/setup.bash
```
## Key Code Components & Function Summaries
### Distance Tracker Node
This ROS node continuously tracks the distance the robot travels by subscribing to the /odom topic and offering a /get_distance service.
- odomCallback(...)

    - Subscribes to /odom topic

    - Updates robot's x, y position

    - Calculates Euclidean distance from the previous position

- getDistanceCallback(...)

    - Implements the /get_distance service

    - Returns the total distance traveled as a string response

### Main GUI Application
This file defines the GUI interface and its integration with ROS. It displays robot info, position, controls, and distance in real time using OpenCV and CVUI.

- robotInfoCallback(...)

    - Subscribes to /robot_info

    - Updates GUI fields like robot description, IP, firmware version, etc.

- odomCallback(...)

    - Subscribes to /odom

    - Updates live robot position (X, Y, Z)

- run()

    - Main GUI loop

    - Handles button rendering and interaction

    - Displays:

        - Robot general info

        - Teleoperation buttons

        - Live odometry position

        - Velocity feedback

        - Distance tracker output

- publishVelocities()

    - Publishes velocity commands to /cmd_vel

    - Triggered continuously when teleoperation buttons are pressed

- callGetDistanceService()

  - Calls /get_distance service

    - Updates the GUI with the distance traveled

- rosSpin()

  - Keeps ROS spinning in a separate thread so the GUI stays responsive

### GUI Framework
- Lightweight OpenCV-based UI header library
  
- Provides rendering for:
   
  - Text labels

  - Buttons

  - Interactive elements

- Integrated directly into the OpenCV display window
  
## Potential Improvements
- Add ROS parameter configuration

- Implement trajectory recording/playback

- Add sensor data visualization (LIDAR, camera)

- Support for multiple robot types

