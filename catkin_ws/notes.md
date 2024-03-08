# Commands

Here are the commands you can use:

## Creating the Workspace

Create the folder `catkin_ws/src` to serve as the workspace directory.

Run `catkin_init_workspace` within the `/catkin_ws` directory to initialize the catkin workspace.

To build the catkin workspace, run `catkin_make` within the `/catkin_ws` directory.

Edit the bash configuration file by using `gedit ~/bashrc` and add the following line at the end:
```
source /home/ashfamt/programming/catkin_ws/devel/setup.bash
```

## Building and Installing Packages

- Use `catkin_make` to build the package and generate build files.

- Use `catkin_make install` to install the package and generate the `install` folder with target files.

To create a new package with dependencies, run the following command within the `/src` directory:
```
catkin_create_pkg ros_pkg_name pkg_dependencies
```
Replace `ros_pkg_name` with the desired package name and `pkg_dependencies` with the necessary dependencies. For example:
```
catkin_create_pkg hello_world roscpp rospy std_msgs
```

## Designing and Running in Gazebo

The recommended steps for designing and running in Gazebo are:

1. Create a URDF file with appropriate Gazebo meta tags.

2. Convert the URDF file into a launch file to facilitate execution.

3. Launch the converted world file.

## Controlling a Turtlebot in Gazebo from a Keyboard

To control a Turtlebot in Gazebo using a keyboard:

1. Install the `turtlebot3` package in ROS.

2. Add the following line to the `~/.bashrc` file:
```
export TURTLEBOT3_MODEL=burger
```
Remember to source the `.bashrc` file after making the update.

3. Launch the desired Gazebo world:
- For an empty world: `roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch`
- For a world with obstacles: `roslaunch turtlebot3_gazebo turtlebot3_world.launch`

4. In a new terminal, launch the teleoperation node:
    ```
    roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
    ```
    Alternatively, to enable autonomous control of the Turtlebot, use:
    ```
    roslaunch turtlebot3_gazebo turtlebot3_simulation.launch
    ```

5. Open RViz to visualize the Turtlebot's movements:
    ```
    roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
    ```


These are the commands and instructions you can follow for the respective tasks.