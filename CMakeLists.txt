cmake_minimum_required(VERSION 3.5)

project(gscam)

# Default to C99
if (NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++14
if (NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif ()

# Gstreamer doesn't provide CMake files
find_package(PkgConfig)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0)

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)

include_directories(include)

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

#=============
# GSCam node
#=============

set(GSCAM_NODE_SOURCES
  src/gscam_node.cpp)

set(GSCAM_NODE_DEPS
  camera_calibration_parsers
  camera_info_manager
  class_loader
  GST_APP
  rclcpp
  sensor_msgs)

add_library(gscam_node SHARED
  ${GSCAM_NODE_SOURCES})

target_compile_definitions(gscam_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(gscam_node
  ${GSCAM_NODE_DEPS})

rclcpp_components_register_nodes(gscam_node "gscam::GSCamNode")
set(node_plugins "${node_plugins}gscam::GSCamNode;$<TARGET_FILE:gscam_node>\n")

#=============
# Export incantations, see https://github.com/ros2/demos/blob/master/composition/CMakeLists.txt
#=============

if (NOT WIN32)
  ament_environment_hooks("${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}")
endif ()

#=============
# GSCam main, statically linked to make IDE debugging easier
#=============

add_executable(gscam_main
  src/gscam_main.cpp
  ${GSCAM_NODE_SOURCES})

ament_target_dependencies(gscam_main
  ${GSCAM_NODE_DEPS})

#=============
# Install
#=============

# Install nodes
install(
  TARGETS gscam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install executables
install(
  TARGETS gscam_main
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()