cmake_minimum_required(VERSION 3.5)
project(compal_amr_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf_file/
  DESTINATION share/${PROJECT_NAME}/urdf_file
)

ament_package()
