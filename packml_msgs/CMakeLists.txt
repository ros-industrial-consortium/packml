cmake_minimum_required(VERSION 2.8.3)
project(packml_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(
  DIRECTORY msg
  FILES
    ItemizedStats.msg
    Mode.msg
    State.msg
    Stats.msg
    Status.msg
)

add_service_files(
  DIRECTORY srv
  FILES
    GetStats.srv
    ModeChange.srv
    Transition.srv
    ResetStats.srv
)


generate_messages(
  DEPENDENCIES
    std_msgs
)


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime std_msgs
)

#############
## Install ##
#############

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)



