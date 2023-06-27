# How to Create a Simulated Mobile Robot in ROS 2 using URDF

The process of creating a simulated mobile robot in ROS 2 using URDF (Unified Robot Description Format). URDF is an XML-based file format commonly used in ROS to describe the structure and kinematics of robots.

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


6. **Build the package**

   Build the project.

   `cd ~/ros_ws/`
   
   `colcon build`

7. **Launch the Robot in RViz**

   Open a new terminal, and launch the robot.

   `cd ~/ros_ws/`
   
   `ros2 launch basic_mobile_robot basic_mobile_bot_v1.launch.py`

8. **View the Coordinate Frames**

   `ros2 run tf2_tools view_frames.py`

   `evince frames.pdf`


# ODOMETERY


   **Prerequisites**

GAZEBO


1. **Create an SDF File for the Robot**

           `mkdir basic_mobile_bot_description`

  Move inside the folder.
           
        `cd basic_mobile_bot_description`

  Create a model.config file.

           `gedit model.config`

  let’s create an SDF (Simulation Description Format) file. This file will contain the tags that are needed to create an instance of the basic_mobile_bot model

           `gedit model.sdf`

  Here we put are sdf file


2. **Test the Robot**


   In gazebo on the left-hand side, click the “Insert” tab.

   On the left panel, find your robot.

   Then open a terminal and type

           `rqt_robot_steering`

3. **Creating the World**

   
   Open a new terminal window, and type:

           cd ~/ros_ws/src/basic_mobile_robot/worlds
   Let’s create a folder for the SDF model.

           mkdir basic_mobile_bot_world
   Move inside the folder.

           cd basic_mobile_bot_world

   Create a world file.


           gedit smalltown.world

   This is where we put our world file.


4. **Edit the Launch File**

   include the world in your launch file.

        cd launch

   
        gedit basic_mobile_bot_v2.launch.py

 5. **Launch the Robot**

   open terminal inside ros_ws

           ros2 launch basic_mobile_robot basic_mobile_bot_v2.launch.py




   ![image](https://github.com/siddhupoduval/SIMULATION-OF-A-MOBILE-ROBOT-USING-A-LIDAR-SENSOR-AND-SLAM-TECHNIQUE-/assets/117801516/bc8777a7-3e4a-4c75-b1a9-ac34b9ee3484)




# Setting Up LIDAR

1. Sensor Fusion
   
   We will configure the robot_localization package to use an Extended Kalman Filter (ekf_node) to fuse the data from sensor inputs. These sensor inputs come from the IMU Gazebo plugin and the differential drive Gazebo plugin that are defined in our SDF file.

**Install the Robot Localization Package**

      sudo apt install ros-humble-robot-localization

**Set the Configuration Parameters**

We now need to specify the configuration parameters of the ekf_node by creating a YAML file.

open the terminal from the basic_moblie_robot folder

      cd config

      gedit ekf.yaml

This is where we keep our ekf.yaml code


**edit the changes in the launch file**

**Launch the Robot**

      cd ~/ros_ws/

      ros2 launch basic_mobile_robot basic_mobile_bot_v3.launch.py



2. Modify the SDF File for the Robot

    Open terminal in basic_mobile

         cd models

   
         cd basic_mobile_bot_description


Type the following command:

      gedit model.sdf

Modify the sdf code and add lidar component to it and close the file.

Modify the URDF File

      gedit basic_mobile_bot_v2.urdf

      cd models

Edit the Launch File

      cd launch

      gedit basic_mobile_bot_v4.launch.py

**Launch the Robot**

      ros2 launch basic_mobile_robot basic_mobile_bot_v4.launch.py





![image](https://github.com/siddhupoduval/SIMULATION-OF-A-MOBILE-ROBOT-USING-A-LIDAR-SENSOR-AND-SLAM-TECHNIQUE-/assets/117801516/1c99bee7-73c9-4d72-b092-aacb0231d20b)






# Navigation and SLAM Using the ROS 2 Navigation Stack

1. **Modify the Launch File**

         cd launch


         gedit basic_mobile_bot_v5.launch.py


   Here you modify the code


2. **Add a Static Map**

   `~/ros_ws/src/basic_mobile_robot/maps`

   We add the  pgm file and the yaml file inside this folder

3. **Add Navigation Stack Parameters**

      The most important parameters are for the Costmap 2D package.A costmap is a map made up of numerous grid cells. Each grid cell has a “cost”. The cost              represents the difficulty a robot would have trying to move through that cell. 

   For example, a cell containing an obstacle would have a high cost. A cell that has no obstacle in it would have a low cost.


   We will use the AMCL (Adaptive Monte Carlo Localization) algorithm

   Go to `~/ros_ws/src/basic_mobile_robot/params folder`

   Create the nav2_params.yaml file here


4. **Create an RViz Configuration File**

   
         cd rviz

    
Create a new RViz file.

         gedit nav2_config.rviz


This is where you write the nav2_config.rviz file

5. **Update the Plugin Parameters**

      We have to updated the LIDAR plugin parameters inside model.sdf inside the basic_mobile_robot_description folder

   Update the LIDAR plugin parameters inside model.sdf inside the basic_mobile_robot_description folder

6. **Update the Robot Localization Parameters**

         cd ~/ros_ws/src/basic_mobile_robot/config


         gedit ekf.yaml

   Update the code here


7. **Launch the Robot Without SLAM**

         ros2 launch basic_mobile_robot basic_mobile_bot_v5.launch.py

    **Move the Robot From Point A to Point B**

   Set the initial pose of the robot by clicking the “2D Pose Estimate” on top of the rviz2 screen
   Then click on the map in the estimated position where the robot is in Gazebo.
   Set a goal for the robot to move to. Click “Navigation2 Goal” button in RViz, and click on a desired destination.

   The wheeled robot will move to the goal destination.

   To check the coordinate frames. Open a new terminal window, and type:



         ros2 run tf2_tools view_frames

         evince frames.pdf





   ![image](https://github.com/siddhupoduval/SIMULATION-OF-A-MOBILE-ROBOT-USING-A-LIDAR-SENSOR-AND-SLAM-TECHNIQUE-/assets/117801516/3bea1b09-3725-482e-bb20-2bea89db8bfd)





   8. **Launch the Robot With SLAM**
  
      **Prerequsists**

      SLAM toolbox



      To launch the robot with SLAM (simultaneous localization and mapping), open a terminal window, and run the following command:

            ros2 launch basic_mobile_robot basic_mobile_bot_v5.launch.py slam:=True


      Use the rqt_robot_steering tool to slowly drive the robot around the room. Open a terminal window, and type: 

            rqt_robot_steering


   9. **Save the Map**
  
             cd maps
      
       Type the following command to save the map:

            ros2 run nav2_map_server map_saver_cli -f my_map

   

   


























   
