cmake_minimum_required(VERSION 3.8)
project(bswarm)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)

find_package(px4_msgs REQUIRED)
find_package(px4_ros_com REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  bswarm/team_controller.py
  bswarm/boids.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()