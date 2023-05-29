cmake_minimum_required(VERSION 3.22)
include(CMakePrintHelpers)

# to be filled with all targets the tools subsystem
set(PCL_TOOLS_ALL_TARGETS)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB SOURCE_FILES "${SOURCE_DIR}/*.cpp")
file(GLOB HEADER_FILES "${INCLUDE_DIR}/*.h")

add_library(pcl_octree_local)

target_sources(pcl_octree_local
        PRIVATE
            ${SOURCE_FILES}
        PUBLIC
            ${HEADER_FILES})

target_include_directories(pcl_octree_local
        PUBLIC
            ${INCLUDE_DIR}
        )

target_link_libraries(pcl_octree_local ${PCL_LIBRARIES})
