# B-Spline-Motion-Generation-for-6-DoF-Robot-Arm

## Overview
This project focuses on implementing B-Spline curves to control the motion of a serial manipulator’s end-effector. Utilizing the myCobot 280 with a Raspberry Pi, the system allows for extensive customizability, supporting the design of various types of 3D B-Spline Spline Motions.

## User Interface
A graphical user interface was developed with Tkinter to allow for the user to have significant control into data point selection, and motion design.


## Features
- **Input Flexibility:** Users can input positions through direct arm manipulation or by uploading a data file.
- **Curve Customization:**
  - Choose between control points or data points.
  - Select from closed or clamped B-Spline types.
  - Define the degree of the curve and parameterization method.
- **Dual-Quaternion Conversion:** Converts position data into dual-quaternions to maintain the system’s rigidity.
- **Inverse Kinematics:** Ensures all positions are within the robot’s workspace.
