# packml
ROS packml <https://en.wikipedia.org/wiki/PackML> support package

# Build
## Dependencies
 - ROS kinetic
 - QT 5 
 - [catkin tools](https://catkin-tools.readthedocs.io/en/latest/)
## Build
```
mkdir -p ~/packml_workspace/src
cd ~/packml_workspace/src
git clone -b kinetic-devel https://github.com/ros-industrial-consortium/packml.git
catkin build
```

# Run
in terminal #1
```
roscore
```
in terminal #2
```
source devel/setup.bash
rosrun packml_ros packml_ros_node
```
in terminal #3
```
source devel/setup.bash
roslaunch packml_gui gui.launch
```
