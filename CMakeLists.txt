cmake_minimum_required(VERSION 3.5)
project(psutil_ros)

set(CMAKE_CXX_STANDARD 17)

# find_package(ament_python REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Cpu.msg"
  "msg/VirtualMemory.msg"
  "msg/Network.msg"
  "msg/Temperatures.msg"
  "msg/SensorTemp.msg"
  DEPENDENCIES std_msgs
  # ADD_PYTHON_BINDINGS  # Ensure this is added to generate Python bindings
)

ament_export_dependencies(rosidl_default_runtime)

# Install Python executables
install(PROGRAMS
  scripts/psutil_ros_node.py  # Install the node script from psutil_ros folder
  DESTINATION lib/${PROJECT_NAME}  # Destination directory
)

ament_package()
