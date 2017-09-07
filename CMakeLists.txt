cmake_minimum_required(VERSION 3.2 FATAL_ERROR)
project(Constraints-in-3D-Pose-Estimation)

# link_directories(/home/ztaal/Documents/covis/cmake)
find_package(COVIS REQUIRED)
add_definitions(${COVIS_DEFINITIONS})
include_directories(${COVIS_INCLUDE_DIRS})
#include_directories(/home/ztaal/Documents/covis/cmake/)

#set( VTK_FOUND 1 )
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_executable(aout src/main.cpp)
target_link_libraries(aout ${COVIS_LIBRARIES})




# cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
# project(Constraints-in-3D-Pose-Estimation)
# find_package(PCL REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS} /home/ztaal/Documents/covis/src/)
# link_directories(${PCL_LIBRARY_DIRS} /home/ztaal/Documents/covis/src/)
# add_definitions(${PCL_DEFINITIONS})
# add_executable(aout src/main.cpp)
# target_link_libraries(aout ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
