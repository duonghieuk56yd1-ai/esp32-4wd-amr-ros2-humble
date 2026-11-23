cmake_minimum_required(VERSION 3.10)
project(amr_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

# ros2 run amr_bringup calibrate_odom  (hiệu chỉnh bán kính / khoảng cách bánh)
install(
  PROGRAMS scripts/calibrate_odom.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME calibrate_odom
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_bringup_config test/test_bringup_config.py)
  ament_add_pytest_test(test_naming_consistency test/test_naming_consistency.py)
  ament_add_pytest_test(test_calibrate_odom test/test_calibrate_odom.py)
endif()

ament_package()
