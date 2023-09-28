cmake_minimum_required(VERSION 3.5)
project(tt_pkg)

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
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tt_pkg REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/PositionInfo.msg"
  "msg/MoveCmd.msg"
  "msg/MoveGoal.msg"
  "msg/ArmCmd.msg"
  "msg/StuffInfo.msg"
  "msg/TargetInfo.msg"
  DEPENDENCIES std_msgs
)
ament_export_dependencies(rosidl_default_runtime)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
add_executable(communication src/communication.cpp src/uart.cpp src/protocol.cpp)
ament_target_dependencies(communication rclcpp tt_pkg)
target_link_libraries(communication pthread)

# add_executable(uart_test src/uart_test.cpp src/uart.cpp src/protocol.cpp)
# target_link_libraries(uart_test pthread)

# Install Cpp executables
install(TARGETS
  communication
  # uart_test
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Glob the Python scripts in the scripts directory
file(GLOB SCRIPTS_FILES scripts/*.py)

# Install Python executables
install(
  PROGRAMS ${SCRIPTS_FILES} 
  DESTINATION lib/${PROJECT_NAME}
)
# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
