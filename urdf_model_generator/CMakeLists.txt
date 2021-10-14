cmake_minimum_required(VERSION 3.0.2)
project(urdf_model_generator)

find_package(catkin REQUIRED COMPONENTS
  urdf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(urdf_model_generator src/urdf_model_generator.cpp)

target_link_libraries(urdf_model_generator
  ${catkin_LIBRARIES}
)
