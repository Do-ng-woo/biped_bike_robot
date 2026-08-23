cmake_minimum_required(VERSION 3.8)
project(biped_bike_robot)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(trajectory_interpolation_test
    test/test_trajectory_interpolation.py
    PYTHON_EXECUTABLE /usr/bin/python3
  )
endif()

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
  scripts/ik_walker.py
  scripts/transform_bike.py
  scripts/bike_transform_sequence.py
  scripts/revert_bike.py
  scripts/bike_teleop.py
  scripts/transform_bike_reverse.py
  scripts/dxl_joint_state_bridge.py
  scripts/imu_base_tf.py
  scripts/opencr_imu_reader.py
  scripts/web_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
