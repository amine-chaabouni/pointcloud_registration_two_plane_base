cmake_minimum_required(VERSION 3.22)
include(CMakePrintHelpers)
set_property(GLOBAL PROPERTY GLOBAL_DEPENDS_DEBUG_MODE 0)


project(voxel_based_registration)

set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL 1.13 REQUIRED)
find_package(VTK REQUIRED)

# Include dependencies
include_directories(
        ${EIGEN3_INCLUDE_DIR}
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
)

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
add_subdirectory(tools)

# Add include directories
set(ROOT ${CMAKE_CURRENT_SOURCE_DIR})
set(TOOLS_INCLUDE_DIRS ${ROOT}/tools/include)

# Add source files
file(GLOB
        TOOLS_HEADER_FILES
        "${TOOLS_INCLUDE_DIRS}/*.h"
        )

set(SOURCE_FILES main.cpp)

# Add executable
add_executable(main ${SOURCE_FILES} ${TOOLS_HEADER_FILES})
target_include_directories(main PRIVATE ${TOOLS_INCLUDE_DIRS})
target_link_libraries(main ${PCL_LIBRARIES} pcl_octree_tools)

