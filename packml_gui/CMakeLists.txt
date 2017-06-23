cmake_minimum_required(VERSION 2.8.3)
project(packml_gui)
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz
)

find_package(Qt5Widgets REQUIRED)
find_package(Qt5Core REQUIRED)
find_package(Qt5Gui REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp rviz
)

set(packml_gui_SRCS
 src/dashboard_panel.cpp
 src/dashboard_widget.cpp
 src/navigation_panel.cpp
 src/navigation_widget.cpp
)

set(packml_gui_HDRS
  include/packml_gui/dashboard_panel.h
  include/packml_gui/dashboard_widget.h
  include/packml_gui/navigation_panel.h
  include/packml_gui/navigation_widget.h
)

set(packml_gui_UIS
 resource/packml_dashboard.ui
 resource/packml_navigation.ui
)

set(packml_gui_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
)

add_definitions(-DQT_NO_KEYWORDS)

qt5_wrap_cpp(packml_gui_MOCS ${packml_gui_HDRS})
qt5_wrap_ui(packml_gui_UIS_H ${packml_gui_UIS})

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${packml_gui_INCLUDE_DIRECTORIES}
)

add_library(${PROJECT_NAME}
  ${packml_gui_UIS_H}
  ${packml_gui_MOCS}
  ${packml_gui_SRCS}
)

target_link_libraries(${PROJECT_NAME}
                      ${catkin_LIBRARIES}
                      Qt5::Widgets
                      Qt5::Gui
                      )


find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
