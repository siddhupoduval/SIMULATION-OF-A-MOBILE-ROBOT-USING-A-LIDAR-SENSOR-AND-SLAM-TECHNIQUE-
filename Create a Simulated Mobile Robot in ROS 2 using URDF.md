# How to Create a Simulated Mobile Robot in ROS 2 using URDF

This tutorial will guide you through the process of creating a simulated mobile robot in ROS 2 using URDF (Unified Robot Description Format). URDF is an XML-based file format commonly used in ROS to describe the structure and kinematics of robots.

## Prerequisites
Before getting started, ensure that you have the following:

- ROS 2 installed on your system. You can follow the official ROS 2 installation instructions provided by the ROS community.
- A working knowledge of ROS concepts and basic understanding of URDF.

## Steps

1. **Install Important ROS 2 Packages**
   
   Open a new terminal window, and type the following commands, one right after the other.

    `sudo apt install ros-<ros2-distro>-joint-state-publisher-gui`
  
    `sudo apt install ros-<ros2-distro>-xacro`


2. **Create a ROS 2 Package**
   In a new terminal window, move to the src

   `ros2 pkg create --build-type ament_cmake basic_mobile_robot`

   Move inside the package and create these folders.

   'mkdir config launch maps meshes models params rviz worlds'

   Now build the package by typing the following command

   `cd ~/ros_ws`
   
   `colcon build`
   
    
   
4. **Create a URDF file**

Design your robot's structure using the URDF format. The URDF file describes the robot's links, joints, sensors, and other physical properties. You can use a text editor or a URDF visual editor, such as URDF Editor, to create and edit the URDF file.

 Go to basic_moblie_robot folder and open a new terminal window

 `cd models`

 put your urdf file here



3. **Configure the robot's appearance**

Specify the visual appearance of the robot's links and joints by defining geometries and materials. You can use shapes like boxes, cylinders, or meshes to represent your robot's physical components. Additionally, define the materials to assign colors or textures to the robot's visual elements.

4. **Define the robot's kinematics**

Define the kinematic structure of the robot by specifying the hierarchical relationship between the robot's links and joints. Each joint connects two links and determines their relative movement. You need to define the joint types (e.g., fixed, revolute, continuous), their limits, and the transformation (origin) of each joint with respect to its parent link.

5. **Add sensors or plugins**

If your robot requires sensors or additional functionality, you can add sensor elements or plugins to the URDF file. This allows you to simulate and interact with the robot's sensors within ROS.

6. **Launch a simulation environment**

Set up a simulation environment, such as Gazebo or Webots, where you can test and visualize the behavior of your robot. Launch the simulation environment with appropriate configuration files and ensure it is running correctly.

7. **Integrate the robot in the environment**

Write launch files or scripts to load the robot's URDF model into the simulation environment. This step typically involves specifying the robot's control interfaces, sensor plugins, and other necessary components. Launch the simulation environment along with your robot's URDF file and observe how the robot behaves.

8. **Test and iterate**

Test your simulated robot's functionality and behavior within the environment. If needed, iterate on the URDF file, adjust parameters, or add more features until your robot behaves as desired.
