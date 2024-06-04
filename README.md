# B-Spline-Motion-Generation-for-6-DoF-Robot-Arm

## Overview
This project focuses on implementing B-Spline curves to control the motion of a serial manipulator’s end-effector. Utilizing the myCobot 280 with a Raspberry Pi, the system allows for extensive customizability, supporting the design of various types of 3D B-Spline Spline Motions. Dual quaternions are used for the calculations of the motions. Find a more detailed explanation here:
- [Full Report](https://www.overleaf.com/read/rffbbpnjjppb#fc83f2)

## User Interface
A graphical user interface was developed with Tkinter to allow for the user to have significant control into data point selection, and motion design.

<img src="https://github.com/kpollackhinds/B-Spline-Motion-Generation-for-6-DoF-Robot-Arm/blob/main/image.png" alt="graphical user interface screenshot" width="600" height="300"/>

## Features
- **Input Flexibility:** Users can input positions through direct arm manipulation or by uploading a data file.
- **Curve Customization:**
  - Choose between control points or data points.
  - Select from closed or clamped B-Spline types.
  - Define the degree of the curve and parameterization method.
- **Inverse Kinematics:** Ensures all positions are within the robot’s workspace.
- **Real-Time Visualization:** Displays the path of the end-effector as well as control positions or interpolation data points.

## Usage
**Running the Program:**
   - Launch the program through the provided script: `python gui.py`. Ensure that the line `# mc = MyCobot(PI_PORT, PI_BAUD)` is uncommented if the MyCobot arm is not in use.
   - Input the desired positions either manually or by uploading a txt file appropriate format (x, y, z, roll, pitch, yaw). See [B-Spline.txt](https://github.com/kpollackhinds/B-Spline-Motion-Generation-for-6-DoF-Robot-Arm/blob/main/B_Spline.txt) for reference.
   - Configure the B-Spline settings and motion parameters as needed.
   - Click "Run Motion" to execute the desired motion, or "Go to selected positions" to have the arm go immediate to each input position (no specific motion between positions).
     
## Limitations

### System Communication and Control Challenges
The project faced significant challenges due to the unclear communication between the Raspberry Pi and the ESP32, which controls the robotic arm:
- **Lack of Control Insight:** The ESP32 operations are not transparent, complicating troubleshooting and limiting the ability to customize command responses effectively.
- **Jittering and Speed Issues:** The robot exhibited jittering and speed inconsistencies when moving between positions, largely due to the inability of the system to confirm the completion of one movement before starting another.

### Ineffective Solutions
Several strategies were attempted to mitigate jittering, none of which proved fully effective:
- **Movement Completion Checks:** Adding checks to confirm the cessation of movement before proceeding introduced unacceptable delays.
- **Time-Based Command Intervals:** Attempts to time the commands based on estimated travel times did not resolve the jittering.
- **Reduced Command Intervals:** Even rapid command sequences did not smooth the motion, nor did reducing the number of path points help in defining optimal intervals.

### Hardware Limitations
The project's underlying hardware limitations stem from its design as a **position-controlled system**, which inherently stops at each target position. Unknown acceleration/deceleration profiles and serial communication delays further complicated efforts to achieve smooth motion. These limitations indicate that the system may not be well-suited for tasks requiring high levels of motion continuity.


