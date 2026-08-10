cmake_minimum_required(VERSION 3.5)
project(tractor_sensor_kit_launch)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_auto_package()
