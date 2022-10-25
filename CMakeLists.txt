cmake_minimum_required(VERSION 3.22)
project(engine)

set(CMAKE_CXX_STANDARD 14)
add_executable(engine main.cpp Body.cpp DrawBody.cpp Geometry.cpp Manifold.cpp State.cpp)
find_package(SFML 2.5 COMPONENTS graphics audio REQUIRED)

target_link_libraries(engine sfml-graphics sfml-audio)