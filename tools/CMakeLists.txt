cmake_minimum_required(VERSION 3.22)
include(CMakePrintHelpers)

# to be filled with all targets the tools subsystem
set(PCL_TOOLS_ALL_TARGETS)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB SOURCE_FILES "${SOURCE_DIR}/*.hpp" "${SOURCE_DIR}/*.cpp")
file(GLOB HEADER_FILES "${INCLUDE_DIR}/*.h")

add_library(pcl_octree_tools ${SOURCE_FILES} ${HEADER_FILES} src/planarity_extractor.cpp include/planarity_extractor.h include/types.h src/utils.cpp include/utils.h)

target_include_directories(pcl_octree_tools
        PUBLIC
        ${INCLUDE_DIR})

target_link_libraries(pcl_octree_tools ${PCL_LIBRARIES})
