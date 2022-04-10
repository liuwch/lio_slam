cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(cloud_viewer)

# set(yaml-cpp_DIR /opt/ros/dashing/opt/yaml_cpp_vendor/lib/cmake/yaml-cpp/)
# find_package(yaml-cpp REQUIRED)
find_package(Boost 1.65 REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 3.3.4 REQUIRED)

include_directories(include
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PCL_INCLUDE_DIRS}
    )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#add_executable (bin2pcd bin2pcd.cpp)
#target_link_libraries (bin2pcd ${PCL_LIBRARIES})

#add_executable (pcd_viewer pcd_viewer.cpp)
#target_link_libraries (pcd_viewer ${PCL_LIBRARIES})

add_executable (pcd_vis pcd_vis.cpp)
target_link_libraries (pcd_vis ${PCL_LIBRARIES} pthread
                ${Boost_LIBRARIES}
                ${Boost_FILESYSTEM_LIBRARY}
                ${Boost_SYSTEM_LIBRARY} )
                # ${YAML_CPP_LIBRARIES} )

# add_executable (pcd_cat pcd_cat.cpp)
# target_link_libraries (pcd_cat ${PCL_LIBRARIES} pthread )
