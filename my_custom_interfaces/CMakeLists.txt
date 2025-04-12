cmake_minimum_required(VERSION 3.5)
project(my_custom_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)


# Declare your service interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Pickup.srv"
  "srv/GetPosition.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs
)

ament_package()
