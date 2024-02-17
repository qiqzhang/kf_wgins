cmake_minimum_required(VERSION 3.10)
project(KF-GINS)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-w")
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -g -ggdb ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-g ${CMAKE_CXX_FLAGS}")

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

if ("${CMAKE_CXX_COMPILER_ID}" STREQUAL "MSVC")
  add_definitions( -D_USE_MATH_DEFINES=True)
  add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
else()
  set(CMAKE_CXX_FLAGS "-Wno-unused-result")
endif()

file(GLOB_RECURSE SOURCE
    src/kf_gins.cpp
    src/fileio/fileloader.cc
    src/fileio/filesaver.cc
    src/kf-gins/gi_engine.cpp
    src/kf-gins/insmech.cpp)

include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/src/wgins
  ${PROJECT_SOURCE_DIR}/src/common
)
add_executable(${PROJECT_NAME} ${SOURCE})

include_directories(${PROJECT_SOURCE_DIR}/ThirdParty)

# Eigen3
include_directories(ThirdParty/eigen-3.3.9)

# sophus
include_directories(${PROJECT_SOURCE_DIR}/ThirdParty/sophus)

# yaml-cpp-0.7.0
add_subdirectory(ThirdParty/yaml-cpp-0.7.0)
target_link_libraries(${PROJECT_NAME} yaml-cpp)

# abseil
set(ABSL_PROPAGATE_CXX_STD true)
add_subdirectory(ThirdParty/abseil-cpp-20220623.1)
target_link_libraries(${PROJECT_NAME} absl::strings absl::str_format absl::time)

add_executable(kf_wgins 
  src/kf_wgins.cc
  src/wgins/wgi_engine.cc
  src/wgins/utm_convert.cc
  src/wgins/StaticIMUInit.cc
  ThirdParty/utm_convert/utm.cc
  ThirdParty/utm_convert/tranmerc.cc
)
target_link_libraries(kf_wgins absl::strings absl::str_format absl::time)


