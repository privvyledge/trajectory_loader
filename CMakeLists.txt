cmake_minimum_required(VERSION 3.14)
project(trajectory_loader)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(TRAJECTORY_LOADER_LIB_SRC
  src/trajectory_loader.cpp
)

set(TRAJECTORY_LOADER_LIB_HEADERS
  include/trajectory_loader/trajectory_loader.hpp
  include/trajectory_loader/visibility_control.hpp
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${TRAJECTORY_LOADER_LIB_SRC}
  ${TRAJECTORY_LOADER_LIB_HEADERS}
)

set(TRAJECTORY_LOADER_NODE_SRC
  src/trajectory_loader_node.cpp
)

set(TRAJECTORY_LOADER_NODE_HEADERS
  include/trajectory_loader/trajectory_loader_node.hpp
)

ament_auto_add_library(${PROJECT_NAME}_node SHARED
  ${TRAJECTORY_LOADER_NODE_SRC}
  ${TRAJECTORY_LOADER_NODE_HEADERS}
)

rclcpp_components_register_node(${PROJECT_NAME}_node
  PLUGIN "trajectory_loader::TrajectoryLoaderNode"
  EXECUTABLE ${PROJECT_NAME}_node_exe
)

ament_auto_package(INSTALL_TO_SHARE
  launch
  config
  data
)
