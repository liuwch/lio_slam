project(gtsam_test)
cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
find_package(Boost COMPONENTS thread filesystem date_time system REQUIRED)
FIND_PACKAGE(GTSAM REQUIRED)

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${GTSAM_INCLUDE_DIR})
INCLUDE_DIRECTORIES("/usr/include/eigen3")
add_executable(test isam_test.cpp)
# 链接库 
target_link_libraries(test ${Boost_LIBRARIES} -lgtsam -ltbb)
install(TARGETS test RUNTIME DESTINATION bin)
