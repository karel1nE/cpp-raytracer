cmake_minimum_required(VERSION 3.14)

project(shad-cpp0)

set(CMAKE_CXX_STANDARD             20)
set(CMAKE_MODULE_PATH              "${CMAKE_SOURCE_DIR}/tools/cmake")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}")
set(CMAKE_EXPORT_COMPILE_COMMANDS  ON)

find_package(Catch REQUIRED)

include(tools/cmake/TestSolution.cmake)
include(tools/cmake/BuildFlags.cmake)
include_directories(tools/util)

function(add_if_exists name)
  if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${name})
    add_subdirectory(${name})
  endif()
endfunction()

find_package(PNG)
if (${PNG_FOUND})
  add_if_exists(raytracer-debug)
  add_if_exists(raytracer-geom)
  add_if_exists(raytracer-reader)
  add_if_exists(raytracer)
  add_if_exists(raytracer-b2)
elseif()
  message(STATUS "raytracer disabled. PNG:${PNG_FOUND}")
endif()
