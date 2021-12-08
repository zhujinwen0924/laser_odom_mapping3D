cmake_minimum_required(VERSION 2.8.3)
project(mapping3D)


add_compile_options(-std=c++11)
#SET(CMAKE_BUILD_TYPE Release)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf

)

find_package(Eigen3  REQUIRED)
find_package(PCL  REQUIRED)


catkin_package(
)

find_package(OpenCV 3.0 QUIET)
find_package(Eigen3  REQUIRED)

include_directories(
${catkin_INCLUDE_DIRS}
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/include
${EIGEN3_INCLUDE_DIR}
${PCL_INCLUDE_DIRS}
)



add_executable(mapping3D
src/mapping3D.cpp)
target_link_libraries(mapping3D
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

