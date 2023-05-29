cmake_minimum_required(VERSION 3.22)
include(CMakePrintHelpers)

# to be filled with all targets the tools subsystem
set(PCL_TOOLS_ALL_TARGETS)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB SOURCE_FILES "${SOURCE_DIR}/*.cpp")
file(GLOB HEADER_FILES "${INCLUDE_DIR}/*.h")

add_library(local_tools)

target_sources(local_tools
        PRIVATE
            ${SOURCE_FILES}
        PUBLIC
            ${HEADER_FILES}
        )

target_link_libraries(local_tools
        PUBLIC
            ${PCL_LIBRARIES}
            pcl_octree_local
        )

target_include_directories(local_tools
        PUBLIC
            ${INCLUDE_DIR}
        )
