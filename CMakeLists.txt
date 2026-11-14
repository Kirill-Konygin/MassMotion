cmake_minimum_required(VERSION 3.21)

project(MassMotion LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(Threads REQUIRED)

add_library(MassMotion STATIC
    src/Agents.cpp
    src/Grid.cpp
    src/ProximitySearch.cpp
    src/ThreadPool.cpp
    include/Agents.h
    include/Grid.h
    include/ProximitySearch.h
    include/ScopeTimer.h
    include/ThreadPool.h
    include/Vector2.h
)

target_include_directories(MassMotion PUBLIC include)
target_link_libraries(MassMotion PUBLIC Threads::Threads)

option(MASSMOTION_BUILD_BENCHMARK "Build the simulation benchmark" OFF)
option(MASSMOTION_ENABLE_PROFILING "Enable scope timing output" OFF)
if(MASSMOTION_ENABLE_PROFILING)
    target_compile_definitions(MassMotion PUBLIC MASSMOTION_ENABLE_PROFILING=1)
endif()

if(MASSMOTION_BUILD_BENCHMARK)
    add_executable(MassMotionBenchmark benchmarks/SimulationBenchmark.cpp)
    target_link_libraries(MassMotionBenchmark PRIVATE MassMotion)
endif()

option(MASSMOTION_BUILD_VIEWER "Build the SDL3 grid viewer" ON)

if(MASSMOTION_BUILD_VIEWER)
    include(FetchContent)

    if(POLICY CMP0135)
        cmake_policy(SET CMP0135 NEW)
    endif()

    set(SDL_SHARED OFF)
    set(SDL_STATIC ON)
    set(SDL_TEST_LIBRARY OFF)

    FetchContent_Declare(SDL3
        URL https://github.com/libsdl-org/SDL/releases/download/release-3.4.18/SDL3-3.4.18.tar.gz
        URL_HASH SHA256=9c75cf16330322c217dedd2e0609f1124f1b54b8633e763467b4684d0f4334a3
    )
    FetchContent_MakeAvailable(SDL3)

    add_executable(MassMotionViewer viewer/Viewer.cpp)
    target_link_libraries(MassMotionViewer PRIVATE MassMotion SDL3::SDL3-static)
endif()
