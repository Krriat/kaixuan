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

install(TARGETS ${PROJECT_NAME}
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

if(BUILD_TESTING)
  enable_testing()
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
