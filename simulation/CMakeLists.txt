cmake_minimum_required(VERSION 3.8)
project(simulation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch worlds models config urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
