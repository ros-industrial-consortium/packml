# packml
ROS packml <https://en.wikipedia.org/wiki/PackML> support package

# Build
## Dependencies
 - ROS kinetic
 - QT 5 
 - [catkin tools]()
## Build
```
mkdir -p ~/packml_workspace/src
cd ~/packml_workspace/src
git clone -b kinetic-devel https://github.com/shaun-edwards/packml.git
catkin build
```

# Run
```
source ~/packml_workspace/devel/setup.bash
rosrun packml_gui packml
```
![image](https://cloud.githubusercontent.com/assets/5349043/24665316/a54b1b8e-1922-11e7-9e9f-f03d3dfc3ce2.png)


