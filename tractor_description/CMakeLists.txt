cmake_minimum_required(VERSION 3.8)
project(tractor_description)

find_package(ament_cmake REQUIRED)

# Install mesh, urdf, และ config เข้าสู่ share/tractor_description
install(DIRECTORY
  config
  mesh
  urdf
  launch
  DESTINATION share/${PROJECT_NAME}
)
# Install Python scripts
install(PROGRAMS
  scripts/nmb_odometer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nmb_odometer_node
)

install(PROGRAMS
  scripts/tractor_odometer_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME tractor_odometer_node
)

install(PROGRAMS
  scripts/agv_pose_display_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME agv_pose_display_node
)

# Install kinematic_model.py as a module alongside the node
install(PROGRAMS
  scripts/kinematic_model.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
