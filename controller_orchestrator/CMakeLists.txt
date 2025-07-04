cmake_minimum_required(VERSION 3.8)
project(controller_orchestrator)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# -------------------------------------------------------------------
# 1) Build the controller_orchestrator library
# -------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED src/controller_orchestrator.cpp)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} rclcpp std_msgs
                          controller_manager_msgs)

# -------------------------------------------------------------------
# 2) Create the 'controller_orchestrator_node' executable
# -------------------------------------------------------------------
add_executable(controller_orchestrator_node
               src/controller_orchestrator_node.cpp)

# ament_target_dependencies adds include paths & links in ROS 2 dependencies
ament_target_dependencies(controller_orchestrator_node rclcpp std_msgs
                          controller_manager_msgs)

# Link against our library
target_link_libraries(controller_orchestrator_node ${PROJECT_NAME})

# -------------------------------------------------------------------
# 3) Install rules
# -------------------------------------------------------------------
# Export the library
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # libraries themselves go into lib;
                                          # executables into lib/<pkg>
)

# Install the node executable
install(TARGETS controller_orchestrator_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})

# Install headers
install(DIRECTORY include/ DESTINATION include)

# Optionally install config / launch directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL)

# Export dependencies, include dirs, and targets
ament_export_include_directories(include)
ament_export_dependencies(rclcpp std_msgs controller_manager_msgs)
ament_export_targets(export_${PROJECT_NAME})

ament_package()
