cmake_minimum_required(VERSION 3.10)

project(RayMarching LANGUAGES CXX)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

add_compile_options(
    -Werror

    -Wall
    -Wextra
    -Wpedantic

    -Wcast-align
    -Wcast-qual
    -Wconversion
    -Wctor-dtor-privacy
    -Wenum-compare
    -Wfloat-equal
    -Wnon-virtual-dtor
    #-Wold-style-cast
    -Woverloaded-virtual
    -Wredundant-decls
    #-Wsign-conversion
    -Wsign-promo
)

add_subdirectory(misc)

configure_file(${CMAKE_SOURCE_DIR}/vertex.glsl ${CMAKE_BINARY_DIR}/vertex.glsl COPYONLY)
configure_file(${CMAKE_SOURCE_DIR}/fragment.glsl ${CMAKE_BINARY_DIR}/fragment.glsl COPYONLY)

add_executable(RayMarching main.cpp)
target_link_libraries(RayMarching mylibs)
target_link_libraries(RayMarching GL glut GLEW glfw)