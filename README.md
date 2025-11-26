# **🚗 Autonomous Four-Wheel Mobile Robot — Mathematical Motion Modeling & Simulation (ROS 2, Gazebo)**

This project implements purely mathematical motion planning for a four-wheel skid-steer ground robot in **ROS 2 Humble** and **Gazebo**.

Instead of relying on navigation frameworks like Nav2 or SLAM, the robot’s trajectory is driven by closed-form kinematic equations, enabling transparent understanding of motion dynamics. The goal is to design, simulate, and analyze different trajectories (Circle, Straight Line, Square, Figure-8) and ensure the robot performs these autonomously while remaining inside the simulated world boundaries.

## **🎯 Objectives**

* Apply skid-steer motion kinematics in real-time control.  
* Simulate robot motion in a confined virtual environment.  
* Enable switchable motion patterns via keyboard inputs.  
* Estimate pose using **Odometry feedback**.  
* Implement **Return-to-Home** functionality.  
* Ensure safe behavior by preventing boundary escape.  
* Demonstrate core ROS 2 concepts: topics, nodes, simulation, control loop.

## **🧩 Key Features**

| Feature | Description |
| :---- | :---- |
| **Fully autonomous trajectory execution** | Robot follows mathematically defined paths without external planners. |
| **Continuous Odometry feedback** | Real-time localization for Home Return logic. |
| **Motion bounded to world geometry** | Dynamic safety constraint (circular boundary). |
| **Real-time teleoperation switching** | Select path behavior on-the-fly via keyboard. |
| **Simulation-only system** | Ensures cost-free iterative testing and learning. |

## **🧠 Kinematic Motion Modeling**

We define motion in terms of parametric time-dependent functions. These ensure accurate angular velocity based on the curvature of the path.

Linear Velocity:

$$v \= \\sqrt{\\dot{x}^2 \+ \\dot{y}^2}$$  
Angular Velocity:

$$\\omega \= \\frac{\\dot{x}\\ddot{y} \- \\dot{y}\\ddot{x}}{v^2 \+ \\epsilon}$$

## **📌 Trajectories Implemented**

| Motion Mode | Mathematical Definition | Behavior Demonstrated |
| :---- | :---- | :---- |
| **Straight Line** | $v \= \\text{const}, \\omega \= 0$ | Constant translational motion. |
| **Circle Path** | $x \= R\\cos(\\omega t), y \= R\\sin(\\omega t)$ | Uniform circular locomotion. |
| **Square Path** | Time-segmented: Move \+ Rotate 90° | Polygonal path approximation. |
| **Figure-8** | $x \= A\\sin(\\omega t)$ $y \= A\\sin(\\omega t)\\cos(\\omega t)$ | Continuous loop crossover (Lemniscate of Gerono). |
| **Return-Home** | $\\omega \\propto (\\theta\_{target}-\\theta)$ $v \\propto \\text{distance}$ | PD Heading \+ Distance control. |

## **🚧 Boundary Confinement System (Safety Feature)**

The Gazebo world uses a **100m × 100m** ground plane. To maintain realistic simulation, robot motion is restricted to:

$$x^2 \+ y^2 \\le (45\\text{m})^2$$  
**If this limit is crossed:**

1. ✔ Linear velocity is cut.  
2. ✔ Robot rotates inward to re-enter boundary.  
3. ✔ Warning message shown in terminal.

*This simulates virtual fencing often used in real robots for geofencing.*

## **🧱 Robot and Sensor Modeling**

| Component | Status | Details |
| :---- | :---- | :---- |
| **URDF/XACRO model** | Fully integrated | Skid-steer drive controller. |
| **LiDAR model** | Present | Optional (not used for current navigation). |
| **Camera model** | Active | Not required for current logic. |
| **Dynamics \+ gravity** | Enabled | Uses Gazebo physics engine. |

## **🔁 Software Architecture**

┌────────────────────────────┐  
│  Motion Controller Node    │  (Publishes Twist)  
│  \- Mathematical Equations  │──────────────┐  
│  \- Mode Switch via Keyboard│              │  
└────────────────────────────┘              ▼  
                                      ┌───────────────┐  
                                      │ Robot in Gazebo│  
                                      │ Skid-Steer Model─────┐  
                                      └───────────────┘      │ publishes /odom  
                                                ▲            │  
                                                │            ▼  
                                   Return-to-Home Controller (Feedback Loop)

## **🔧 How to Run**

### **1️⃣ Build Workspace**

cd \~/ROS2\_4wheel\_autonomous\_robot-main  
colcon build  
source install/setup.bash

### **2️⃣ Launch Gazebo Simulation**

ros2 launch robot\_description launch\_sim.launch.py world:=./src/robot\_description/models/myWorld/boxes\_world.sdf use\_sim\_time:=true rviz\_config\_file:=./src/robot\_description/rviz/nav.config.rviz

### **3️⃣ Start Motion Controller**

ros2 run robot\_nav motion\_patterns.py

## **🎮 Keyboard Controls**

| Key | Action |
| :---- | :---- |
| **c** | Circle Motion (Default) |
| **l** | Straight Line |
| **s** | Square Path |
| **8** | Figure-8 Path |
| **h** | Return-to-Home position |
| **x** | Stop & hold |

## **🧪 Performance Observations**

* Figure-8 motion creates smooth curvature transitions, validating correct kinematic linking.  
* Boundary system prevents robot from falling off world edges.  
* Odometry-based Home return converges reliably within tolerance (\~25 cm).  
* Controller maintains stable performance even under varying simulation physics.

## **📌 Future Enhancements**

| Possible Upgrade | Benefit |
| :---- | :---- |
| **Model Predictive Control (MPC)** | Replace open-loop $v, \\omega$ control for better curvature tracking. |
| **SLAM Toolbox** | Enable unknown world exploration and mapping. |
| **LiDAR Integration** | Add collision avoidance for fully obstacle-aware autonomy. |
| **Waypoint Navigation** | Convert mathematical inputs into specific coordinate goals. |
| **Nav2 Integration** | Bridge custom logic toward the standard Navigation Stack. |

## **👥 Team Information**

Team Name: Invictus Dynamics  
Course: Autonomous Robotics & Simulation  
Department: Electronics & Communication Engineering  
Institute: Gati Shakti Vishwavidyalaya  
Faculty Mentor: Dr. Asifa Yesmin

### **👤 Contributors**

* Parth Sidhu  
* Shreya Mohanty  
* Srijan Gupta  
* Aashutosh Raj Verma  
* Abhinandan Singh