cmake_minimum_required(VERSION 3.16)
project(hector_transmission_interface LANGUAGES CXX)

# Standard Compiler and Export Settings
find_package(ros2_control_cmake REQUIRED)
set_compiler_options()
export_windows_symbols()

set(THIS_PACKAGE_INCLUDE_DEPENDS
    pluginlib rclcpp transmission_interface controller_manager_msgs
    hector_transmission_interface_msgs sensor_msgs)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gen_version_h REQUIRED)
foreach(dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${dependency} REQUIRED)
endforeach()

# --- Main Library ---
add_library(${PROJECT_NAME} SHARED src/adjustable_offset_transmission_loader.cpp
                                   src/adjustable_offset_manager.cpp)

target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
                         $<INSTALL_INTERFACE:include>)

ament_target_dependencies(${PROJECT_NAME} ${THIS_PACKAGE_INCLUDE_DEPENDS})

# Pluginlib Export
pluginlib_export_plugin_description_file(${PROJECT_NAME}
                                         ros2_control_plugins.xml)

# --- Testing Section ---
if(BUILD_TESTING)
  find_package(ament_cmake_gmock REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(rtest REQUIRED)
  find_package(GTest CONFIG REQUIRED COMPONENTS GTest GMock)

  # 1. Add the original transmission test (GTest)
  ament_add_gtest(test_adjustable_offset_transmission
                  test/test_adjustable_offset_transmission.cpp)
  target_link_libraries(test_adjustable_offset_transmission ${PROJECT_NAME}
                        ${rclcpp_LIBRARIES})
  ament_target_dependencies(test_adjustable_offset_transmission rclcpp
                            transmission_interface)

  # 1. Add the Manager test (GMock + rtest) NOTE: Added test/main.cpp which is
  #   required for initializing ROS2/GTest context
  ament_add_gmock(
    test_adjustable_offset_manager src/adjustable_offset_manager.cpp
    src/adjustable_offset_transmission_loader.cpp
    test/test_adjustable_offset_manager.cpp)

  # 1. Link Manager Test against the Test Doubles We use the namespaced target
  #   for our doubles, but use plain rtest for the library
  target_link_libraries(
    test_adjustable_offset_manager
    rtest::publisher_mock
    rtest::subscription_mock
    rtest::timer_mock
    rtest::service_mock
    rtest::service_client_mock
    rtest::action_server_mock
    rtest::action_client_mock)

  # ament_target_dependencies handles the "rtest" target/include resolution
  # correctly
  ament_target_dependencies(test_adjustable_offset_manager
                            ${THIS_PACKAGE_INCLUDE_DEPENDS})

  target_include_directories(test_adjustable_offset_manager
                             PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

  find_package(hector_testing_utils REQUIRED)
  ament_add_gtest(test_offset_manager_integration
                  test/test_offset_manager_integration.cpp)
  target_link_libraries(
    test_offset_manager_integration ${PROJECT_NAME} ${rclcpp_LIBRARIES}
    hector_testing_utils::hector_testing_utils)
  ament_target_dependencies(test_offset_manager_integration rclcpp
                            hardware_interface transmission_interface)
  target_include_directories(test_offset_manager_integration
                             PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endif()

# --- Installation & Export ---
install(DIRECTORY include/ DESTINATION include/)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
ament_generate_version_header(${PROJECT_NAME})
