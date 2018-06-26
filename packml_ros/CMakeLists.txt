cmake_minimum_required(VERSION 2.8.3)
project(packml_ros)

find_package(catkin REQUIRED COMPONENTS
  packml_msgs
  packml_sm
  roscpp
)

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

include_directories(${packml_ros_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${packml_ros_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)

add_executable(packml_ros_node src/packml_ros_node.cpp)
target_link_libraries(packml_ros_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(packml_ros_node PUBLIC -std=c++11)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  set(UTEST_SRC_FILES test/utest.cpp)

  catkin_add_gtest(${PROJECT_NAME}_utest ${UTEST_SRC_FILES})
  target_compile_options(${PROJECT_NAME}_utest PUBLIC -std=c++11)
  target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
