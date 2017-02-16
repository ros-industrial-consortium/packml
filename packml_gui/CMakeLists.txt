cmake_minimum_required(VERSION 2.8.3)
project(packml_gui)

catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS
    rospy
    rqt_gui
    rqt_gui_py
    packml_msgs
)

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY resource
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(
  PROGRAMS
    scripts/packml
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
