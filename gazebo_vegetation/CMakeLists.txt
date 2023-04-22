cmake_minimum_required(VERSION 2.8.3)
project(gazebo_vegetation)

## Compile as C++17
add_compile_options(-std=c++17)
set(CMAKE_CXX_STANDARD 17)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  
)

find_package(gazebo REQUIRED)

catkin_package(
 CATKIN_DEPENDS gazebo_ros
)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} )


install(DIRECTORY models DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
