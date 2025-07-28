# I. Setup
After setting up the URDF code and adding physical properties such as mass and inertia,  Gazebo is used to launch the vehicle model as follows:
<p align="center">
  <img src="https://github.com/user-attachments/assets/439273b4-7d11-4c4f-b4f8-a831c5aeb4df" width="450" height="300" alt="image" />
  <br />
  <em> Figure 1. 4WD mobile robot in simulation</em>
</p>
My simulated enviroment is the factory where there are a lot of obstacles and spaces for robot to 3D mapping and navigate 
<p align="center">
  <img src="https://github.com/user-attachments/assets/fdcd94ea-6cf3-4d26-808e-d76d2968b5da" width="805" height="450" alt="image" />
  <br />
  <em> Figure 2. Simulated factory in Gazebo </em>
</p>

# II. 3D Mapping
RTAB-Map (Real-Time Appearance-Based Mapping) is a graph-based SLAM approach being used in this project. Appearance-based SLAM means that the algorithm uses data collected from visual sensors to localize the robot and build a map of the environment. During the SLAM process using an RGB-D camera such as the Orbbec Astra, the system utilizes multiple ROS nodes to collect and process sensor data in real time in order to build a 3D map of the environment and localize the robot.

<div style="display: flex; justify-content: center; gap: 20px; margin: auto; width: fit-content;">
  <img src="https://github.com/user-attachments/assets/7d9a7789-15b1-4a67-be19-04d02838551a" width="389" height="220" alt="image" />
  <img src="https://github.com/user-attachments/assets/712a502c-0936-4876-81e7-2255b4a0fbd8" width="389" height="220" alt="image" />
</div>
<p align="center"><em>Figure 3. RTAB-Map S showing 3D Mapping with point clouds.</em></p>

