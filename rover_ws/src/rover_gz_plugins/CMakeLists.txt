cmake_minimum_required(VERSION 3.8)
project(rover_gz_plugins)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)

# Gazebo system plugin: enforces the rocker differential constraint.
# Loaded by name from the URDF (<plugin filename="RockerDifferential" ...>);
# sim.launch.py puts this install dir on GZ_SIM_SYSTEM_PLUGIN_PATH.
add_library(RockerDifferential SHARED src/rocker_differential.cc)
target_link_libraries(RockerDifferential PRIVATE gz-sim8::gz-sim8)

install(TARGETS RockerDifferential
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
