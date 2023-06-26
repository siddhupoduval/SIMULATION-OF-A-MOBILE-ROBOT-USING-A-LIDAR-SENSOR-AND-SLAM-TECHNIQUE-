# Creating a workspace in ros2

### Open a new terminal window, and create a new workspace. You can call it any name, but we will call it “ros_ws”
### Inside the workspace, we will create a source (i.e. src) directory. This is where your packages will go**.

`mkdir -p ~/ros_ws/src`


### Navigate to the workspace.

`cd ~/ros_ws/`


### Let’s build the packages. Go to the root of the workspace.

`sudo apt update`
`sudo apt install python3-colcon-common-extensions`


### Build the packages

`colcon build`

### Finally, we need to add the packages in this workspace to the current environment.

`sudo apt-get install gedit`

### Open a new terminal window, and open your .bashrc file:

`gedit ~/.bashrc`

### Add this line of code to the very bottom of the .bashrc file.

`source ~/dev_ws/install/setup.bash`

### Save the file, and close it.
