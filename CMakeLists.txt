cmake_minimum_required(VERSION 3.9...3.20)
project(polyai LANGUAGES CXX)

include(CheckIPOSupported)
check_ipo_supported(RESULT ipo_supported OUTPUT ipo_error)
if(ipo_supported)
    message(STATUS "LTO supported — enabling IPO/LTO")
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)
else()
    message(WARNING "LTO not supported: ${ipo_error}")
endif()

set(USE_GRAPHICAL_BENCHMARK OFF CACHE BOOL "" FORCE)
set(USE_GLUT OFF CACHE BOOL "" FORCE)

add_subdirectory(ammo.js/bullet)

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
