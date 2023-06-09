cmake_minimum_required(VERSION 3.22)
include(CMakePrintHelpers)

# to be filled with all targets the tools subsystem
set(PCL_TOOLS_ALL_TARGETS)

set(SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src)
set(INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/include)

file(GLOB SOURCE_FILES "${SOURCE_DIR}/*.cpp")
file(GLOB HEADER_FILES "${INCLUDE_DIR}/*.h")

add_library(tpbr_solver)

target_sources(tpbr_solver
        PRIVATE
        ${SOURCE_FILES}
        PUBLIC
        ${HEADER_FILES}
        )

target_link_libraries(tpbr_solver
        PUBLIC
        ${PCL_LIBRARIES}
        pcl_octree_local
        local_tools
        )

target_include_directories(tpbr_solver
        PUBLIC
        ${INCLUDE_DIR}
        )
