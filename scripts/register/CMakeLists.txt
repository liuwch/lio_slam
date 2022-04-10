cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(pcd_ndt)

find_package(PCL REQUIRED)
find_package(Eigen3 3.3.4 REQUIRED)

include_directories(include
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (bin2pcd bin2pcd.cpp)
#target_link_libraries (bin2pcd ${PCL_LIBRARIES})

#add_executable (pcd_viewer pcd_viewer.cpp)
#target_link_libraries (pcd_viewer ${PCL_LIBRARIES})

add_executable (pcd_ndt pcd_ndt.cpp)
target_link_libraries (pcd_ndt ${PCL_LIBRARIES} pthread )

add_executable (pcd_icp pcd_icp.cpp)
target_link_libraries (pcd_icp ${PCL_LIBRARIES} pthread )
