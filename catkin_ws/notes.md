# Commands

Create folder `catkin_ws/src`

run `catkin_init_workspace` to initialize a catkin workspace

in `/catkin_ws`, run `catkin_make` to create a catkin workspace

Edit bash using `gedit ~/bashrc` to add catkin_ws scripts into bash access and add the following command to the end of the file:
```
source /home/ashfamt/programming/catkin_ws/devel/setup.bash
```

`catkin_make`: creates build files 

`catkin_make install`: creates `install` folder to keep the install target files.


`catkin_create_pkg ros_pkg_name pkg_dependencies`: Creates new package with dependencies. Must run in the `/src` dir. For example:

`catkin_create_pkg hello_world roscpp rospy std_msgs`


Best way:
urdf file -> gazebo conversion -> world file -> launch file -> run