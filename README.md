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

