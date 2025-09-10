cmake_minimum_required(VERSION 3.9...3.20)
project(polyai LANGUAGES CXX)

set(USE_GRAPHICAL_BENCHMARK OFF CACHE BOOL "" FORCE)
set(USE_GLUT OFF CACHE BOOL "" FORCE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_subdirectory(ammo.js/bullet EXCLUDE_FROM_ALL)

add_executable(polyai src/main.cpp)

target_link_libraries(polyai PRIVATE
    BulletDynamics
    BulletCollision
    LinearMath
)

target_include_directories(polyai PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/ammo.js/bullet/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
