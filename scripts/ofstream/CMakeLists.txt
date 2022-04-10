cmake_minimum_required(VERSION 3.5)

project(osftream_tutorial)

find_package(Boost 1.65 REQUIRED)

include_directories(
    ${Boost_INCLUDE_DIRS}
)

add_executable(useOfstream read_write.cpp)

target_link_libraries(useOfstream
  ${Boost_LIBRARIES}
)