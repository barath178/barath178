cmake_minimum_required(VERSION 3.8)
project(tb3_omx_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch meshes urdf textures worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
