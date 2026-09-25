cmake_minimum_required(VERSION 3.20)
project(aura_gazebo_controller LANGUAGES CXX)

find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

add_executable(aura_controller controller.cpp)
target_compile_features(aura_controller PRIVATE cxx_std_17)
target_link_libraries(aura_controller PRIVATE
  gz-transport13::core
  gz-msgs10::core
)
