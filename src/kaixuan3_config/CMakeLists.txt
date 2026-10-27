cmake_minimum_required(VERSION 3.22)
project(kaixuan3_config)

find_package(ament_cmake REQUIRED)

ament_package()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
    PATTERN "setup_assistant.launch" EXCLUDE)
endif()

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/flight_recorder.py DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/cycle_capture.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/bus_diagnostics.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/monitor64.py DESTINATION lib/${PROJECT_NAME})
