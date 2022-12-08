cmake_minimum_required(VERSION 3.12)

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

# Default to SHARED, but user can turn this off to get STATIC
option(BUILD_SHARED_LIBS "Build using shared libraries" ON)

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

# Create ament index resource which references the libraries in the binary dir
set(node_plugins "")

# How to build a library:
# https://docs.ros.org/en/humble/How-To-Guides/Ament-CMake-Documentation.html#building-a-library

#=============
# GSCam node
#=============

add_library(gscam_node src/gscam_node.cpp)
add_library(gscam2::gscam_node ALIAS gscam_node)

target_compile_definitions(gscam_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

target_include_directories(gscam_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(gscam_node
  PRIVATE
    camera_calibration_parsers::camera_calibration_parsers
    camera_info_manager::camera_info_manager
    class_loader::class_loader
    rclcpp_components::component
    ros2_shared::ros2_shared
    sensor_msgs::sensor_msgs_library
  PUBLIC
    rclcpp::rclcpp
    PkgConfig::GSTREAMER
    PkgConfig::GST_APP)

rclcpp_components_register_nodes(gscam_node "gscam2::GSCamNode")
set(node_plugins "${node_plugins}gscam2::GSCamNode;$<TARGET_FILE:gscam_node>\n")

#=============
# Test subscriber node
#=============

add_library(subscriber_node src/subscriber_node.cpp)
add_library(gscam2::subscriber_node ALIAS subscriber_node)

target_compile_definitions(subscriber_node
  PRIVATE "COMPOSITION_BUILDING_DLL")

target_include_directories(subscriber_node
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>)

target_link_libraries(subscriber_node
  PRIVATE
    class_loader::class_loader
    rclcpp_components::component
    sensor_msgs::sensor_msgs_library
  PUBLIC
    rclcpp::rclcpp)

rclcpp_components_register_nodes(subscriber_node "gscam2::ImageSubscriberNode")
set(node_plugins "${node_plugins}gscam2::ImageSubscriberNode;$<TARGET_FILE:subscriber_node>\n")

#=============
# GSCam main, statically linked to make IDE debugging easier
#=============

add_executable(gscam_main
  src/gscam_main.cpp)

target_link_libraries(gscam_main
  PRIVATE
    gscam2::gscam_node)

#=============
# Manual composition of camera and subscriber nodes, IPC=true
#=============

add_executable(ipc_test_main src/ipc_test_main.cpp)

target_link_libraries(ipc_test_main
  PRIVATE
    gscam2::gscam_node
    gscam2::subscriber_node
    sensor_msgs::sensor_msgs_library)

#=============
# Test
#=============

if(BUILD_TESTING)
  # Load & run linters listed in package.xml
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  # Smoke test
  find_package(ament_cmake_gtest)
  ament_add_gtest(smoke_test
    test/smoke_test.cpp
    ENV GSCAM_CONFIG="videotestsrc pattern=snow ! capsfilter caps=video/x-raw,width=800,height=600 ! videoconvert")
  if(TARGET smoke_test)
    target_link_libraries(smoke_test
      gscam2::gscam_node
      sensor_msgs::sensor_msgs_library)
  endif()
endif()

#=============
# Install
#=============

install(DIRECTORY include/${PROJECT_NAME}
  DESTINATION include)

install(TARGETS gscam_node subscriber_node
  EXPORT gscam2_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS gscam_main ipc_test_main
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY cfg launch
  DESTINATION share/${PROJECT_NAME})

#=============
# Export targets, ament_export_targets will call "install(EXPORT ...)"
#=============

ament_export_targets(gscam2_targets HAS_LIBRARY_TARGET)

ament_export_dependencies(rclcpp sensor_msgs)

ament_package()
