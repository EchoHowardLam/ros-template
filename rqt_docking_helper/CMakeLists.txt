cmake_minimum_required(VERSION 2.8.3)
project(rqt_docking_helper)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS 
    roscpp
    rqt_gui
    rqt_gui_cpp
    std_msgs
    geometry_msgs
    gazebo_msgs
    std_srvs
    actionlib_msgs

    april_docking
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  find_package(Qt5Widgets REQUIRED)
else()
  find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)
  include(${QT_USE_FILE})
endif()

catkin_python_setup()

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES ${PROJECT_NAME} 
  CATKIN_DEPENDS roscpp rqt_gui rqt_gui_cpp std_msgs geometry_msgs gazebo_msgs std_srvs actionlib_msgs april_docking
)

set(rqt_docking_helper_SRCS
  src/helper_gui.cpp
)

set(rqt_docking_helper_HDRS
  include/helper_gui.h
)

set(rqt_docking_helper_UIS
  resource/helper_gui.ui
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  qt5_wrap_cpp(rqt_docking_helper_MOCS ${rqt_docking_helper_HDRS})
  qt5_wrap_ui(rqt_docking_helper_UIS_H ${rqt_docking_helper_UIS})
else()
  qt4_wrap_cpp(rqt_docking_helper_MOCS ${rqt_docking_helper_HDRS})
  qt4_wrap_ui(rqt_docking_helper_UIS_H ${rqt_docking_helper_UIS})
endif()

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  ${catkin_INCLUDE_DIRS}
)
add_library(${PROJECT_NAME}
  ${rqt_docking_helper_SRCS}
  ${rqt_docking_helper_MOCS}
  ${rqt_docking_helper_UIS_H}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

if("${qt_gui_cpp_USE_QT_MAJOR_VERSION} " STREQUAL "5 ")
  target_link_libraries(${PROJECT_NAME} Qt5::Widgets)
else()
  target_link_libraries(${PROJECT_NAME} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
endif()

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

add_dependencies(rqt_docking_helper ${catkin_EXPORTED_TARGETS})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(PROGRAMS scripts/rqt_docking_helper
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  resource
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
