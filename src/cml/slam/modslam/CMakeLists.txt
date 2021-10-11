cmake_minimum_required(VERSION 3.9)
project(DSO
        VERSION 1.0
        LANGUAGES C CXX
        )

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_EXTENSIONS ON)

add_library(Hybrid SHARED Hybrid.cpp Hybrid.h Research.cpp Make.cpp indirect/Tracking.cpp indirect/Mapping.cpp direct/Mapping.cpp direct/Tracking.cpp)
target_link_libraries(Hybrid ${CML_LIBRARIES})

file(COPY modslam.yaml DESTINATION ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})