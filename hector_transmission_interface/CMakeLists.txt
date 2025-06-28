cmake_minimum_required(VERSION 3.16)
project(hector_transmission_interface LANGUAGES CXX)

find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    pluginlib rclcpp transmission_interface controller_manager_msgs
    controller_orchestrator hector_transmission_interface_msgs)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

add_library(hector_transmission_interface SHARED
            src/dynamic_offset_transmission_loader.cpp)
target_include_directories(
  hector_transmission_interface
  PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
         $<INSTALL_INTERFACE:include/hector_transmission_interface>)
ament_target_dependencies(hector_transmission_interface
                          ${THIS_PACKAGE_INCLUDE_DEPENDS})
#[[target_link_libraries(
        hector_transmission_interface
        PUBLIC transmission_interface pluginlib
        rclcpp)]]

pluginlib_export_plugin_description_file(hector_transmission_interface
                                         ros2_control_plugins.xml)

install(DIRECTORY include/ DESTINATION include/hector_transmission_interface)
install(
  TARGETS hector_transmission_interface
  EXPORT export_hector_transmission_interface
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_targets(export_hector_transmission_interface HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
ament_generate_version_header(${PROJECT_NAME})
