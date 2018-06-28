cmake_minimum_required(VERSION 2.8.3)
project(packml_sm)

find_package(catkin REQUIRED roscpp)
find_package(Boost REQUIRED COMPONENTS system mpi)

set(packml_sm_SRCS
  src/abstract_state_machine.cpp
  src/boost/packml_state_machine.cpp
  src/ros/dlog.cpp
)

set(packml_sm_HDRS  
  include/packml_sm/abstract_state_machine.h
  include/packml_sm/common.h
  include/packml_sm/dlog.h
  include/packml_sm/event.h
  include/packml_sm/make_unique.h
  include/packml_sm/state_changed_event_args.h
  include/packml_sm/boost/packml_events.h
  include/packml_sm/boost/packml_state_machine.h
  include/packml_sm/boost/packml_state_machine_continuous.h
  include/packml_sm/boost/packml_state_machine_single_cycle.h
  include/packml_sm/boost/packml_states.h
  include/packml_sm/boost/packml_transitions_continuous.h
  include/packml_sm/boost/packml_transitions_single_cycle.h
  include/packml_sm/boost/state_change_notifier.h
)

set(packml_sm_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

catkin_package(
  INCLUDE_DIRS ${packml_sm_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  DEPENDS
)

include_directories(${packml_sm_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${packml_sm_SRCS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
target_compile_definitions(${PROJECT_NAME} PUBLIC -DBOOST_MPL_CFG_NO_PREPROCESSED_HEADERS PUBLIC -DBOOST_MPL_LIMIT_VECTOR_SIZE=60 PUBLIC -DBOOST_MPL_LIMIT_MAP_SIZE=60 PUBLIC -DFUSION_MAX_VECTOR_SIZE=50)
target_compile_options(${PROJECT_NAME} PUBLIC -std=c++11)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

#############
## Testing ##
#############

if(CATKIN_ENABLE_TESTING)
  find_package(rostest)
  set(UTEST_SRC_FILES test/utest.cpp
      test/state_machine.cpp)

  catkin_add_gtest(${PROJECT_NAME}_utest ${UTEST_SRC_FILES})
  target_compile_options(${PROJECT_NAME}_utest PUBLIC -std=c++11)
  target_link_libraries(${PROJECT_NAME}_utest ${PROJECT_NAME})
endif()
