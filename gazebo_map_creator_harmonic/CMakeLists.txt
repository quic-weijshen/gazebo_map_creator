cmake_minimum_required(VERSION 3.5)
project(gazebo_map_creator)

# Default to C11
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif()

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PCL_LIBRARIES)
set(GZ_PLUGIN)
set(GZ_SIM)
set(GZ_RENDERING)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PCL REQUIRED)
find_package(gazebo_map_creator_interface REQUIRED)

find_package(sdformat15 REQUIRED)
find_package(gz-common6-all REQUIRED)
find_package(gz-sim9-all REQUIRED)
find_package(gz-math8-all REQUIRED)
find_package(gz-msgs11-all REQUIRED)
find_package(gz-physics8-all REQUIRED)
find_package(gz-sensors9-all REQUIRED)
find_package(gz-transport14-all REQUIRED)

include_directories(
  ${PCL_INCLUDE_DIRS}
  ${SDFORMAT-INCLUDE_DIRS}
  ${GZ-COMMON_INCLUDE_DIRS}
  ${GZ-SIM_INCLUDE_DIRS}
  ${GZ-MATH_INCLUDE_DIRS}
  ${GZ-MSGS_INCLUDE_DIRS}
  ${GZ-TRANSPORT_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

link_libraries(
  ${SDFORMAT-LIBRARIES}
  ${GZ-COMMON_LIBRARIES}
  ${GZ-SIM_LIBRARIES}
  ${GZ-MATH_LIBRARIES}
  ${GZ-MSGS_LIBRARIES}
  ${GZ-TRANSPORT_LIBRARIES}
)

add_definitions(${PCL_DEFINITIONS})

add_library(${PROJECT_NAME} SHARED
  src/gazebo_map_creator.cpp
)

ament_target_dependencies(${PROJECT_NAME}
  "gazebo_map_creator_interface"
  "geometry_msgs"
  "rclcpp"
  pluginlib
)

ament_export_libraries(${PROJECT_NAME})

target_link_libraries(${PROJECT_NAME}
  gz-sim9::gz-sim9
  ${GZ_PLUGIN}
  ${GZ_RENDERING}
  ${PCL_LIBRARIES}
)

install(PROGRAMS DESTINATION lib/${PROJECT_NAME}/ )
install(DIRECTORY scripts DESTINATION share/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib )
# Install Python executables
install(PROGRAMS
  scripts/request_map.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()