cmake_minimum_required(VERSION 3.22)
include(CMakePrintHelpers)
set_property(GLOBAL PROPERTY GLOBAL_DEPENDS_DEBUG_MODE 1)


project(voxel_based_registration)

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.13 REQUIRED)
find_package(VTK REQUIRED)

# Link dependencies
link_libraries(
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
)

# Add definitions
add_definitions(
        ${PCL_DEFINITIONS}
)

set(CMAKE_CXX_FLAGS "-g")

# Add subdirectories
add_subdirectory(custom_lib)
add_subdirectory(tools)
add_subdirectory(solver)
add_subdirectory(python)
#add_subdirectory(third_lib)

# Add include directories
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(TOOLS_INCLUDE_DIRS ${ROOT}/tools/include)

# Add source files
file(GLOB
        TOOLS_HEADER_FILES
        "${TOOLS_INCLUDE_DIRS}/*.h"
        "custom_lib/include/*.h"
        )

set(SOURCE_FILES main.cpp)

# Include dependencies
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${TOOLS_INCLUDE_DIRS}
)

# Add executable
add_executable(main ${SOURCE_FILES})
target_link_libraries(main
        PRIVATE
            ${PCL_LIBRARIES}
            pcl_octree_local
            local_tools
            tpbr_solver
        )

target_include_directories(main PUBLIC ${CMAKE_CURRENT_LIST_DIR})
