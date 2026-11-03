cmake_minimum_required(VERSION 3.21)

project(MassMotion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(MassMotion STATIC
    src/Grid.cpp
    include/Grid.h
)

target_include_directories(MassMotion PUBLIC include)
