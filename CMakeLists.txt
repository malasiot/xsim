project(xsim)

cmake_minimum_required(VERSION 3.0.0)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/CMake/ ${CMAKE_ROOT}/Modules/)
set(LIB_PREFIX xsim)

include(Setup)
include(FindThirdParty)

include_directories(src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_BINARY_DIR}/
    ${EIGEN3_INCLUDE_DIR}
    )

add_subdirectory(src)
add_subdirectory(test)

include(${CMAKE_SOURCE_DIR}/CMake/Install.cmake)

