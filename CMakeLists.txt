cmake_minimum_required( VERSION 3.2 FATAL_ERROR )
project( Constraints-in-3D-Pose-Estimation )

set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /usr/lib/vtk-6.2)

find_package( COVIS HINTS /home/ztaal/covis/build/lib/cmake/COVIS )
find_package(yaml-cpp HINTS /home/ztaal/yaml-cpp/build/)
# find_package( COVIS HINTS /home/msteenberg/covis/build/lib/cmake/COVIS )
# find_package(yaml-cpp HINTS /home/msteenberg/yaml-cpp/build/)

add_definitions( ${COVIS_DEFINITIONS} )
include_directories( ${COVIS_INCLUDE_DIRS} )

find_package( VTK 6.2 REQUIRED NO_MODULE )
include( ${VTK_USE_FILE} )

add_executable( aout src/main.cpp src/Ransac.cpp src/Timer.cpp src/Benchmark.cpp
                src/Correspondence.cpp src/PosePrior.cpp src/yml_loader.cpp
                src/Benchmark_Sixd.cpp )
target_link_libraries( aout ${COVIS_LIBRARIES} yaml-cpp )

#find_package(Eigen3 3.3.2 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

target_include_directories(aout PRIVATE "/home/ztaal/yaml-cpp/include/")
# target_include_directories(aout PRIVATE "/home/msteenberg/yaml-cpp/include/")
#link_directories(/home/msteenberg)
#target_link_libraries(aout yaml-cpp)


add_definitions(
    -Wno-deprecated-declarations
)
