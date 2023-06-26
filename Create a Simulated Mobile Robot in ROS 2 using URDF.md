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


 Let’s add some packages that our project will depend on

 `cd ~/ros_ws/src/basic_mobile_robot`

 `gedit package.xml`

 After the <buildtool_depend> tag, add the following lines:

`<exec_depend>joint_state_publisher</exec_depend>
<exec_depend>robot_state_publisher</exec_depend>
<exec_depend>rviz</exec_depend>
<exec_depend>xacro</exec_depend>`

save and close the file



5. **Create the Launch File**
   open terminal from the basic_mobile_robot folder and type:

   `cd launch`

   `gedit basic_mobile_bot_v1.launch.py`


   Let’s add a configuration file that will initialize RViz with the proper settings so we can view the robot as soon as RViz launches


   `colcon_cd basic_mobile_robot`

   `cd rviz`

   `gedit urdf_config.rviz`

   this is where u add your config code for rviz

   
