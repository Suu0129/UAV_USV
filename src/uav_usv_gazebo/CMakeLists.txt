cmake_minimum_required(VERSION 3.16)
project(uav_usv_gazebo)

find_package(ament_cmake REQUIRED)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
