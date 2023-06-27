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
   

   
