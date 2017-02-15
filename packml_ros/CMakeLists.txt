cmake_minimum_required(VERSION 2.8.3)
project(packml_ros)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  packml_msgs
  packml_sm
  roscpp
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

set(packml_ros_SRCS
  src/packml_ros.cpp
)

set(packml_ros_HDRS
  include/packml_ros/packml_ros.h
)

set(packml_ros_INCLUDE_DIRECTORIES
  include
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)



catkin_package(
  INCLUDE_DIRS ${packml_ros_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS packml_msgs packml_sm roscpp
  DEPENDS
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(packml_ros_MOCS ${packml_ros_HDRS})
  #qt5_wrap_ui(packml_ros__UIS_H ${packml_ros_UIS})
else()
  qt4_wrap_cpp(packml_ros_MOCS ${packml_ros_HDRS})
  #qt4_wrap_ui(packml_ros__UIS_H ${packml_ros_UIS})
endif()

include_directories(${packml_ros_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${packml_ros_SRCS} ${packml_ros_MOCS})
#add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)


add_executable(packml_ros_node src/packml_ros_node.cpp)
target_link_libraries(packml_ros_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(packml_ros_node PUBLIC -std=c++11)



## Specify libraries to link a library or executable target against
# target_link_libraries(packml_ros_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS packml_ros packml_ros_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  set(UTEST_SRC_FILES test/utest.cpp)

  catkin_add_gtest(${PROJECT_NAME}_utest ${UTEST_SRC_FILES})
  target_compile_options(${PROJECT_NAME}_utest PUBLIC -std=c++11)
  target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

