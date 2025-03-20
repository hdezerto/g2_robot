cmake_minimum_required(VERSION 3.8)
project(icp)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
#endif()

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io registration)

# Include directories
include_directories(
  ${PCL_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

# Add executable for C++ node
add_executable(icp_processor src/icp_processor.cpp)

# Link libraries
ament_target_dependencies(icp_processor rclcpp sensor_msgs pcl_conversions pcl_ros PCL)
target_link_libraries(icp_processor ${PCL_LIBRARIES})

# Install C++ executables
install(TARGETS
  icp_processor
  DESTINATION lib/${PROJECT_NAME}
)

# Ament package
ament_package()