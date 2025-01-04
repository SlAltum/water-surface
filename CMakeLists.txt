cmake_minimum_required(VERSION 3.10.0)
project(WaterSurface VERSION 0.1.0 LANGUAGES C CXX)

if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_definitions(-DDEBUG)
endif()

add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/source-charset:utf-8>")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SDL2 CONFIG REQUIRED)
find_package(GTest REQUIRED)
find_package(OpenCLHeaders REQUIRED)
find_package(OpenCLICDLoader REQUIRED)
find_package(OpenCLHeadersCpp REQUIRED)

include_directories("include")
include_directories(3rdparty/imgui)
include_directories(3rdparty/stb)
include_directories(3rdparty/delaunator-cpp)
file(GLOB SOURCES src/*.cpp)
file(GLOB IMGUI_SOURCES 3rdparty/imgui/*.cpp)

add_library(Imgui ${IMGUI_SOURCES})
target_link_libraries(Imgui PRIVATE SDL2::SDL2-static SDL2::SDL2main)

add_library(WaterSurface STATIC ${SOURCES})
target_link_libraries(WaterSurface PRIVATE OpenCL::Headers OpenCL::OpenCL OpenCL::HeadersCpp)
target_compile_definitions(WaterSurface PRIVATE CL_HPP_TARGET_OPENCL_VERSION=300)

if (WIN32)
    if ("${CMAKE_MAKE_PROGRAM}" MATCHES "MsBuild")
        add_custom_command(TARGET WaterSurface
                POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/assets ${CMAKE_BINARY_DIR}/$(ConfigurationName)/assets
                COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/kernel ${CMAKE_BINARY_DIR}/$(ConfigurationName)/kernel
                )
    elseif(MINGW)
        add_custom_command(TARGET WaterSurface
                POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/assets ${CMAKE_BINARY_DIR}/assets
                COMMAND ${CMAKE_COMMAND} -E copy_directory ${CMAKE_CURRENT_SOURCE_DIR}/kernel ${CMAKE_BINARY_DIR}/kernel
                )
    endif ()
endif ()

install(TARGETS WaterSurface RUNTIME DESTINATION bin)

add_executable(WaterSurfaceExample example/water_surface_example.cpp)
target_link_libraries(WaterSurfaceExample PRIVATE Imgui)
target_link_libraries(WaterSurfaceExample PRIVATE SDL2::SDL2-static SDL2::SDL2main)
target_link_libraries(WaterSurfaceExample PRIVATE WaterSurface)

add_executable(WaterSurfaceTest test/water_surface_test.cpp)
target_link_libraries(WaterSurfaceTest PRIVATE ${GTEST_BOTH_LIBRARIES} pthread)
target_link_libraries(WaterSurfaceTest PRIVATE WaterSurface)

gtest_discover_tests(WaterSurfaceTest)