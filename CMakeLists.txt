cmake_minimum_required( VERSION 3.2 FATAL_ERROR )
project( Constraints-in-3D-Pose-Estimation )

set(CMAKE_LIBRARY_PATH ${CMAKE_LIBRARY_PATH} /usr/lib/vtk-6.2)

find_package( COVIS HINTS /home/ztaal/covis/build/lib/cmake/COVIS )
add_definitions( ${COVIS_DEFINITIONS} )
include_directories( ${COVIS_INCLUDE_DIRS} )

find_package( VTK 6.2 REQUIRED NO_MODULE )
include( ${VTK_USE_FILE} )

add_executable( aout src/main.cpp src/ransac.cpp )
target_link_libraries( aout ${COVIS_LIBRARIES} )

add_definitions(
    -Wno-deprecated-declarations
)
