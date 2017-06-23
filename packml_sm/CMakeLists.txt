cmake_minimum_required(VERSION 2.8.3)
project(packml_sm)
find_package(catkin REQUIRED rqt_gui rqt_gui_cpp)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()


set(packml_sm_SRCS
  src/state_machine.cpp
  src/state.cpp
  src/transitions.cpp
)

set(packml_sm_HDRS
  include/packml_sm/state_machine.h
  include/packml_sm/state.h
  include/packml_sm/transitions.h
  include/packml_sm/events.h
  include/packml_sm/common.h
)

set(packml_sm_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)


catkin_package(
  INCLUDE_DIRS ${packml_sm_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp
  DEPENDS
)


if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(packml_sm_MOCS ${packml_sm_HDRS})
  #qt5_wrap_ui(packml_sm__UIS_H ${packml_sm_UIS})
else()
  qt4_wrap_cpp(packml_sm_MOCS ${packml_sm_HDRS})
  #qt4_wrap_ui(packml_sm__UIS_H ${packml_sm_UIS})
endif()

include_directories(${packml_sm_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS})
add_library(${PROJECT_NAME} ${packml_sm_SRCS} ${packml_sm_MOCS})

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

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
