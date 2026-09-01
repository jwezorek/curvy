cmake_minimum_required(VERSION 3.25)

project(curvy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)

add_executable(curvy WIN32
    src/circle.cpp
    src/circle.h
    src/colors.h
    src/curvy_arithmetic_viewer.cpp
    src/curvy_arithmetic_viewer.h
    src/curvy_vector.cpp
    src/curvy_vector.h
    src/curvy_vector_viewer.cpp
    src/curvy_vector_viewer.h
    src/curvy_world_simulation.cpp
    src/curvy_world_simulation.h
    src/game.cpp
    src/game.h
    src/gdi_util.cpp
    src/gdi_util.h
    src/main.cpp
    src/puck.cpp
    src/puck.h
    src/state.h
    src/util.cpp
    src/util.h
)

target_compile_definitions(curvy PRIVATE
    UNICODE
    _UNICODE
)

target_link_libraries(curvy PRIVATE
    Eigen3::Eigen
    Boost::headers
    Gdiplus
)

if(MSVC)
    target_compile_options(curvy PRIVATE /W3 /permissive-)
endif()
