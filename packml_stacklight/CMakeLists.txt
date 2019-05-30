cmake_minimum_required(VERSION 2.8.3)
project(packml_stacklight)

find_package(catkin REQUIRED COMPONENTS
  packml_msgs
  packml_sm
  std_msgs
  roscpp
)

set(${PROJECT_NAME}_SRCS
  src/utils.cpp
  src/${PROJECT_NAME}.cpp
)

set(${PROJECT_NAME}_HDRS
  include/${PROJECT_NAME}/utils.h
  include/${PROJECT_NAME}/${PROJECT_NAME}.h
)

set(${PROJECT_NAME}_INCLUDE_DIRECTORIES
  include
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
  INCLUDE_DIRS ${${PROJECT_NAME}_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS packml_msgs roscpp std_msgs
  DEPENDS
)

include_directories(${${PROJECT_NAME}_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})

add_library(${PROJECT_NAME} ${${PROJECT_NAME}_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_options(${PROJECT_NAME}_node PUBLIC -std=c++11)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)


#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  set(UTEST_SRC_FILES test/test_stacklight.cpp)

  catkin_add_gtest(${PROJECT_NAME}_utest ${UTEST_SRC_FILES})
  target_compile_options(${PROJECT_NAME}_utest PUBLIC -std=c++11)
  target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME})
endif()


#if(CATKIN_ENABLE_TESTING)
#	find_package(rostest)
#	#catkin_add_gtest(${PROJECT_NAME}_test test/launch/test_stacklight.launch test/test_stacklight.cpp)
#	target_link_libraries(${PROJECT_NAME}_test ${PROJECT_NAME} ${catkin_LIBRARIES})
#endif()
