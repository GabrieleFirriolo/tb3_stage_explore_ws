cmake_minimum_required(VERSION 3.5)
project(tb3_stage_explore)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
