🚗 ROS-based Camera & LiDAR Self-Driving System
Overview
This project is a self-driving system that operates on the XYCar platform, based on the Robot Operating System (ROS).
It utilizes camera images and LiDAR sensor data to implement logic for multiple driving scenarios, including lane detection, obstacle avoidance, and stop line recognition.

Features
Real-time lane recognition using camera images

Steering control via PID control

Obstacle detection and S-curve avoidance algorithm using LiDAR

Stop line recognition and gradual deceleration/stopping process

Alternative control using LiDAR when lane detection fails inside a tunnel

System Architecture
sequenceDiagram
    participant Camera
    participant LiDAR
    participant ROS
    participant Logic
    participant Motor

    Camera->>ROS: Receive Image
    LiDAR-->>ROS: Receive Distance Data

    ROS->>Logic: Process Image + LiDAR Data

    alt Lane Detection Successful
        Logic->>Logic: Calculate Center Line
        Logic->>Motor: Send Steering Command via PID Control
    else Lane Not Detected
        Logic->>Logic: Determine Obstacle / Tunnel
        Logic->>Motor: Apply Offset Correction or Drive Straight
    end

    alt Stop Line Detected
        Logic->>Motor: Stop or Decelerate
    end

    Note over ROS, Logic: Looping Process

Dependencies
Python 2.x

ROS (Kinetic recommended)

OpenCV

NumPy

sensor_msgs (Image / LaserScan)

xycar_msgs (XYCar-specific messages)

Installation & Execution
1. Place in ROS Workspace
cd ~/catkin_ws/src
git clone <repository_url>
cd ~/catkin_ws
catkin_make
source devel/setup.bash

2. Launch Node
roscore
roslaunch xycar_pkg xycar_auto_drive.launch

Main Files
Filename

Description

main.py

The main script that controls the overall self-driving logic.

PID()

PID control function to calculate steering angle from an offset value.

img_callback()

Callback function for receiving camera images.

lidar_callback()

Callback function for receiving LiDAR data.

check_obstacles()

Measures the distance to obstacles in front, left, and right.

avoid_s_curve()

S-curve avoidance algorithm based on distance differences.

detect_stop_line()

Detects stop lines (judges by the ratio of white pixels in ROI).

start()

Manages all processes and executes the control loop.

Control Logic Overview
Detects lanes and calculates the center line using Canny + Hough Transform.

Outputs the deviation from the center line as a steering angle using PID control.

Identifies the presence and position of obstacles with LiDAR to make avoidance decisions.

When lane detection fails, it assumes a tunnel environment and uses only LiDAR for corrective control.

Performs gradual deceleration and stopping upon detecting a stop line.

Test Environment
XYCar ROS Platform

Ubuntu 16.04 / ROS Kinetic

Camera Resolution: 640x480

LiDAR: 504-point scan (LaserScan type)

To-do / Future Work
Improve the precision of LiDAR correction.

Replace with machine learning-based lane detection (e.g., CNN).

Separate nodes into modules and introduce testing.

Optimize the avoidance control algorithm.

Support migration to ROS2.

Shutdown
Upon user termination (q key press) or Ctrl+C, cv2.destroyAllWindows() and rospy.signal_shutdown() will be executed.

References
ROS Official Documentation: http://wiki.ros.org

OpenCV Python Official Documentation

XYCar Open Source Project
