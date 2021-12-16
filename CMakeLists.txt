cmake_minimum_required(VERSION 3.5)
project(obstacles_rviz_plugin)

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

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(rviz_ogre_vendor REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(obstacle_detector_interfaces REQUIRED)
# find_package(obstacle_detector REQUIRED)

include_directories(include src ${OGRE_INCLUDE_DIRS} ${OGRE_INCLUDE_DIRS}/Paging)

set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  src/obstacles_display.cpp
  src/circle_visual.cpp
  src/segment_visual.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rviz_common
  rviz_rendering

  obstacle_detector_interfaces
  # obstacle_detector
)

target_link_libraries(${PROJECT_NAME}
  # obstacle_detector::obstacle_detector_core
  Qt5::Widgets
)

pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

install(TARGETS
  ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_package()
