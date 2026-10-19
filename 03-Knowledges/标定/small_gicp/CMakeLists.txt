cmake_minimum_required(VERSION 3.16)
project(small_gicp_demo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(SMALL_GICP_DIR "${CMAKE_CURRENT_SOURCE_DIR}/small_gicp" CACHE PATH
    "Path to a small_gicp checkout")

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenMP QUIET)

if(NOT EXISTS "${SMALL_GICP_DIR}/include/small_gicp/registration/registration_helper.hpp")
  message(FATAL_ERROR
    "small_gicp headers not found. Set -DSMALL_GICP_DIR=/path/to/small_gicp")
endif()

add_library(small_gicp STATIC
  "${SMALL_GICP_DIR}/src/small_gicp/registration/registration.cpp"
  "${SMALL_GICP_DIR}/src/small_gicp/registration/registration_helper.cpp")
target_include_directories(small_gicp PUBLIC "${SMALL_GICP_DIR}/include")
target_link_libraries(small_gicp PUBLIC Eigen3::Eigen)

add_executable(small_gicp_demo demo.cpp)
target_link_libraries(small_gicp_demo PRIVATE small_gicp)

if(OpenMP_CXX_FOUND)
  target_link_libraries(small_gicp PUBLIC OpenMP::OpenMP_CXX)
endif()
