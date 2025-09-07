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

set(BUILD_DEMOS OFF CACHE BOOL "" FORCE)
set(BUILD_MULTITHREADING OFF CACHE BOOL "" FORCE)
add_subdirectory(ammo.js/bullet EXCLUDE_FROM_ALL)

add_executable(polyai src/main.cpp)

target_link_libraries(polyai PRIVATE
    BulletCollision
    BulletDynamics
    LinearMath
)

target_include_directories(polyai PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/ammo.js/bullet/src
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

set(CMAKE_CXX_COMPILER clang++)
