cmake_minimum_required(VERSION 3.8)
project(tilt_kaixuan)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes scripts textures urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
