cmake_minimum_required(VERSION 3.8)
project(biped_bike_robot)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  meshes
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/patch_urdf.py
  scripts/ready_posture.py
  scripts/op3_walker.py
  scripts/transform_bike.py
  scripts/revert_bike.py
  scripts/bike_teleop.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
