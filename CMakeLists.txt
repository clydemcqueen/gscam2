cmake_minimum_required(VERSION 3.6)

project(gscam2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Emulate Colcon in CLion
if($ENV{CLION_IDE})
  message(STATUS "Running inside CLion")
  find_package(fastrtps_cmake_module REQUIRED)
  set(FastRTPS_INCLUDE_DIR "/opt/ros/foxy/include")
  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/foxy/lib/libfastrtps.so")
  set(ros2_shared_DIR "${PROJECT_SOURCE_DIR}/../../../install/ros2_shared/share/ros2_shared/cmake")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DRUN_INSIDE_CLION")
endif()

# Gstreamer doesn't provide CMake files
find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0 IMPORTED_TARGET)
pkg_check_modules(GST_APP REQUIRED gstreamer-app-1.0 IMPORTED_TARGET)

find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclpy REQUIRED)
find_package(ros2_shared REQUIRED)
find_package(sensor_msgs REQUIRED)

include_directories(include)

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

#=============
# GSCam node
#=============

set(GSCAM_NODE_DEPS
  camera_calibration_parsers
  camera_info_manager
  class_loader
  rclcpp
  rclcpp_components
  ros2_shared
  sensor_msgs)

add_library(gscam_node SHARED)
target_sources(gscam_node PRIVATE src/gscam_node.cpp)
add_library(gscam2::gscam_node ALIAS gscam_node)

target_compile_definitions(gscam_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(gscam_node
  ${GSCAM_NODE_DEPS})

target_link_libraries(gscam_node PkgConfig::GSTREAMER PkgConfig::GST_APP)
target_include_directories(gscam_node PUBLIC
  "$<BUILD_INTERFACE:${GSTREAMER_INCLUDE_DIRS}>")
rclcpp_components_register_nodes(gscam_node "gscam2::GSCamNode")
set(node_plugins "${node_plugins}gscam2::GSCamNode;$<TARGET_FILE:gscam_node>\n")

#=============
# Test subscriber node
#=============

set(SUBSCRIBER_NODE_SOURCES
  src/subscriber_node.cpp)

set(SUBSCRIBER_NODE_DEPS
  class_loader
  rclcpp
  rclcpp_components
  sensor_msgs)

add_library(subscriber_node SHARED
  ${SUBSCRIBER_NODE_SOURCES})

target_compile_definitions(subscriber_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(subscriber_node
  ${SUBSCRIBER_NODE_DEPS})

rclcpp_components_register_nodes(subscriber_node "gscam2::ImageSubscriberNode")
set(node_plugins "${node_plugins}gscam2::ImageSubscriberNode;$<TARGET_FILE:subscriber_node>\n")

#=============
# GSCam main, statically linked to make IDE debugging easier
#=============

add_executable(gscam_main
  src/gscam_main.cpp)

ament_target_dependencies(gscam_main
  ${GSCAM_NODE_DEPS})

target_link_libraries(gscam_main gscam2::gscam_node)

#=============
# Manual composition of camera and subscriber nodes, IPC=true
#=============

add_executable(
  ipc_test_main
  src/ipc_test_main.cpp
)
target_link_libraries(
  ipc_test_main
  gscam_node
  subscriber_node
)
ament_target_dependencies(
  ipc_test_main
  rclcpp
)

#=============
# Test
#=============

# Load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest)
  ament_add_gtest(
    smoke_test
    test/smoke_test.cpp
    ENV GSCAM_CONFIG="videotestsrc pattern=snow ! capsfilter caps=video/x-raw,width=800,height=600 ! videoconvert"
  )
  if(TARGET smoke_test)
    target_link_libraries(smoke_test gscam_node)
  endif()
endif()

#=============
# Export
# Best practice, see https://discourse.ros.org/t/ament-best-practice-for-sharing-libraries/3602
#=============

ament_export_dependencies(class_loader)

ament_export_include_directories(include)

ament_export_targets(export_gscam_node export_subscriber_node)

ament_export_libraries(gscam_node subscriber_node)

ament_package()

#=============
# Install
#=============

install(DIRECTORY include/${PROJECT_NAME} DESTINATION include)

install(
  TARGETS gscam_node
  EXPORT export_gscam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS subscriber_node
  EXPORT export_subscriber_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS gscam_main ipc_test_main
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY cfg launch DESTINATION share/${PROJECT_NAME})