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
distance_tracker_service.cpp – ROS service that tracks and reports distance traveled

robot_gui.cpp – Main GUI application with control interface

cvui.h – Lightweight GUI library for OpenCV (custom implementation)

### Technical Stack
- First item C++14/17

- Second item ROS (Robot Operating System)

OpenCV for computer vision and GUI rendering

CVUI for lightweight UI components

Multi-threading for concurrent ROS and GUI operations
