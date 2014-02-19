cmake_minimum_required(VERSION 2.8.3)
project(ros_sot_robot_model)

find_package(catkin REQUIRED COMPONENTS dynamic_graph_bridge jrl_dynamics_urdf roscpp urdf) 

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ros_sot_robot_model
  CATKIN_DEPENDS dynamic_graph_bridge jrl_dynamics_urdf roscpp urdf
  DEPENDS dynamic-graph
)

include(cmake/base.cmake)
include(cmake/python.cmake)


#add_required_dependency(dynamic-graph-python)

set(LIBRARY_OUTPUT_PATH ${CATKIN_DEVEL_PREFIX}/lib)
set(CMAKE_INSTALL_RPATH "${LIBRARY_OUTPUT_PATH}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${LIBRARY_OUTPUT_PATH}")
set(PKG_CONFIG_ADDITIONAL_VARIABLES
  ${PKG_CONFIG_ADDITIONAL_VARIABLES}
  plugindirname
  plugindir
  )

add_required_dependency(dynamic-graph)

macro(compile_plugin NAME)
  message(lib path ${LIBRARY_OUTPUT_PATH})
  file(MAKE_DIRECTORY "${LIBRARY_OUTPUT_PATH}/dynamic_graph/ros/${NAME}")
  add_library(${NAME} src/${NAME}.cpp include/${PROJECT_NAME}/${NAME}.hh)
#  pkg_config_use_dependency(${NAME} jrl-mal)
  pkg_config_use_dependency(${NAME} dynamic-graph)
#  pkg_config_use_dependency(${NAME} sot-core)
#  pkg_config_use_dependency(${NAME} jrl-dynamics-urdf)
  target_link_libraries(${NAME} ${catkin_LIBRARIES})
#  set_target_properties(${NAME} PROPERTIES BUILD_WITH_INSTALL_RPATH True)
  install(TARGETS ${NAME} DESTINATION lib)


  dynamic_graph_python_module("ros/${NAME}"
    ${NAME}
    ros/${NAME}/wrap
    )

#  PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap realtime_tools)
#  PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap jrl-mal)
  PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap dynamic_graph)
#  PKG_CONFIG_USE_DEPENDENCY(ros/${NAME}/wrap sot-core)
endmacro()

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})

compile_plugin(ros_sot_robot_model)

add_subdirectory(src)
