cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(Constraints-in-3D-Pose-Estimation)

find_package(COVIS REQUIRED)
add_definitions(${COVIS_DEFINITIONS})
include_directories(${COVIS_INCLUDE_DIRS})

find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(aout src/main.cpp)
target_link_libraries(aout ${COVIS_LIBRARIES})
