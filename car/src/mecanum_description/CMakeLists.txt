cmake_minimum_required(VERSION 3.8)
project(mecanum_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
