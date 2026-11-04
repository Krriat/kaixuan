cmake_minimum_required(VERSION 3.8)
project(kaixuan_can_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ethercat CONFIG REQUIRED)
find_package(Threads REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_interface REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(pluginlib REQUIRED)
find_package(control_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(diagnostic_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/kaixuan_can_system.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
  std_msgs
  std_srvs
  diagnostic_msgs
)

target_link_libraries(${PROJECT_NAME}
  EtherLab::ethercat
  Threads::Threads
)

add_executable(kaixuan_home_server
  src/kaixuan_home_server.cpp
)

add_executable(ethercat_direct_tool
  src/ethercat_direct_tool.cpp
)

target_compile_features(kaixuan_home_server PUBLIC cxx_std_17)
target_compile_features(ethercat_direct_tool PUBLIC cxx_std_17)

ament_target_dependencies(kaixuan_home_server
  rclcpp
  rclcpp_action
  control_msgs
  trajectory_msgs
  std_srvs
)

target_link_libraries(ethercat_direct_tool
  EtherLab::ethercat
  Threads::Threads
)

pluginlib_export_plugin_description_file(hardware_interface kaixuan_can_hardware_plugins.xml)

add_library(kaixuan_torque_controller SHARED src/torque_controller.cpp)
target_include_directories(kaixuan_torque_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(kaixuan_torque_controller PUBLIC cxx_std_17)
ament_target_dependencies(kaixuan_torque_controller controller_interface hardware_interface
  rclcpp rclcpp_lifecycle pluginlib realtime_tools std_msgs)
pluginlib_export_plugin_description_file(controller_interface torque_controller_plugins.xml)

# Prefer a normal ROS installation; also support the supplied rootless installer.
set(KAIXUAN_PINOCCHIO_PREFIX "$ENV{HOME}/.local/share/kaixuan/pinocchio/opt/ros/humble"
  CACHE PATH "Optional private Pinocchio prefix")
if(EXISTS "${KAIXUAN_PINOCCHIO_PREFIX}")
  list(PREPEND CMAKE_PREFIX_PATH "${KAIXUAN_PINOCCHIO_PREFIX}")
endif()
find_package(pinocchio QUIET)
if(pinocchio_FOUND)
  add_library(kaixuan_gravity_model SHARED src/gravity_model.cpp)
  target_include_directories(kaixuan_gravity_model PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
  target_compile_features(kaixuan_gravity_model PUBLIC cxx_std_17)
  target_link_libraries(kaixuan_gravity_model PRIVATE pinocchio::pinocchio)
  set_target_properties(kaixuan_gravity_model PROPERTIES INSTALL_RPATH_USE_LINK_PATH TRUE)
  add_library(kaixuan_gravity_controller SHARED src/gravity_controller.cpp)
  target_include_directories(kaixuan_gravity_controller PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
  target_compile_features(kaixuan_gravity_controller PUBLIC cxx_std_17)
  ament_target_dependencies(kaixuan_gravity_controller controller_interface hardware_interface
    rclcpp rclcpp_lifecycle pluginlib realtime_tools std_msgs)
  target_link_libraries(kaixuan_gravity_controller kaixuan_gravity_model)
  set_target_properties(kaixuan_gravity_controller PROPERTIES INSTALL_RPATH "$ORIGIN")
  pluginlib_export_plugin_description_file(controller_interface gravity_controller_plugins.xml)
  install(TARGETS kaixuan_gravity_model kaixuan_gravity_controller LIBRARY DESTINATION lib)
else()
  message(WARNING "Pinocchio unavailable: gravity controller omitted. Run scripts/install_pinocchio.sh and rebuild.")
endif()

install(TARGETS ${PROJECT_NAME} kaixuan_torque_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS kaixuan_home_server ethercat_direct_tool
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES TORQUE_CONTROL.md GRAVITY_COMPENSATION.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/install_pinocchio.sh DESTINATION share/${PROJECT_NAME}/scripts)

if(BUILD_TESTING)
  enable_testing()
  if(pinocchio_FOUND)
    add_executable(test_gravity_model test/test_gravity_model.cpp)
    target_link_libraries(test_gravity_model kaixuan_gravity_model)
    add_test(NAME gravity_model COMMAND test_gravity_model)
    add_executable(test_gravity_controller test/test_gravity_controller.cpp)
    target_link_libraries(test_gravity_controller kaixuan_gravity_controller)
    ament_target_dependencies(test_gravity_controller controller_interface hardware_interface
      rclcpp rclcpp_lifecycle realtime_tools std_msgs)
    add_test(NAME gravity_controller COMMAND test_gravity_controller)
    set_tests_properties(gravity_controller PROPERTIES ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=216")
    add_test(NAME gravity_dynamics COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_gravity_dynamics.py)
  endif()
  add_executable(test_joint_angle_mapping test/test_joint_angle_mapping.cpp)
  target_include_directories(test_joint_angle_mapping PRIVATE include)
  target_compile_features(test_joint_angle_mapping PRIVATE cxx_std_17)
  add_test(NAME joint_angle_mapping COMMAND test_joint_angle_mapping)
  add_executable(test_torque_safety test/test_torque_safety.cpp)
  target_include_directories(test_torque_safety PRIVATE include)
  target_compile_features(test_torque_safety PRIVATE cxx_std_17)
  add_test(NAME torque_safety COMMAND test_torque_safety)
  add_executable(test_torque_controller test/test_torque_controller.cpp)
  target_link_libraries(test_torque_controller kaixuan_torque_controller)
  ament_target_dependencies(test_torque_controller controller_interface hardware_interface
    rclcpp rclcpp_lifecycle realtime_tools std_msgs)
  add_test(NAME torque_controller COMMAND test_torque_controller)
  set_tests_properties(torque_controller PROPERTIES ENVIRONMENT "ROS_LOCALHOST_ONLY=1;ROS_DOMAIN_ID=217")
  add_executable(test_torque_interfaces test/test_torque_interfaces.cpp)
  target_link_libraries(test_torque_interfaces ${PROJECT_NAME})
  ament_target_dependencies(test_torque_interfaces hardware_interface rclcpp rclcpp_lifecycle std_msgs
    std_srvs diagnostic_msgs)
  add_test(NAME torque_interfaces COMMAND test_torque_interfaces)
  add_executable(test_dc_reference_trace test/test_dc_reference_trace.cpp)
  target_include_directories(test_dc_reference_trace PRIVATE include)
  target_compile_features(test_dc_reference_trace PRIVATE cxx_std_17)
  add_test(NAME dc_reference_trace COMMAND test_dc_reference_trace)
  add_executable(test_dc_reference_fast_profile test/test_dc_reference_fast_profile.cpp)
  target_include_directories(test_dc_reference_fast_profile PRIVATE include)
  target_compile_features(test_dc_reference_fast_profile PRIVATE cxx_std_17)
  add_test(NAME dc_reference_fast_profile COMMAND test_dc_reference_fast_profile)
  add_executable(test_dc_reference_follower test/test_dc_reference_follower.cpp)
  target_include_directories(test_dc_reference_follower PRIVATE include)
  target_compile_features(test_dc_reference_follower PRIVATE cxx_std_17)
  add_test(NAME dc_reference_follower COMMAND test_dc_reference_follower)
  add_executable(test_dc_startup_reset test/test_dc_startup_reset.cpp)
  target_include_directories(test_dc_startup_reset PRIVATE include)
  target_compile_features(test_dc_startup_reset PRIVATE cxx_std_17)
  add_test(NAME dc_startup_reset COMMAND test_dc_startup_reset)
  add_executable(test_startup_guard test/test_startup_guard.cpp)
  target_include_directories(test_startup_guard PRIVATE include)
  target_compile_features(test_startup_guard PRIVATE cxx_std_17)
  add_test(NAME startup_guard COMMAND test_startup_guard)
  add_executable(test_cycle_capture test/test_cycle_capture.cpp)
  target_include_directories(test_cycle_capture PRIVATE include)
  target_compile_features(test_cycle_capture PRIVATE cxx_std_17)
  target_link_libraries(test_cycle_capture Threads::Threads)
  add_test(NAME cycle_capture COMMAND test_cycle_capture)
  add_executable(test_bus_probe test/test_bus_probe.cpp)
  target_include_directories(test_bus_probe PRIVATE include)
  target_compile_features(test_bus_probe PRIVATE cxx_std_17)
  add_test(NAME bus_probe COMMAND test_bus_probe)
endif()

ament_package()
