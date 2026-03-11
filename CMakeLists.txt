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

# patch 스크립트도 설치 (필요 시 사용)
install(PROGRAMS
  scripts/patch_urdf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
