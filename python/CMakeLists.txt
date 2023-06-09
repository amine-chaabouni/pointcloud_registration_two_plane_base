cmake_minimum_required(VERSION 3.22)

project(tpbr_python_bindings)

find_package(pybind11)
pybind11_add_module(tpbr_python tpbr_python/tpbr_python.cc)

message(STATUS "Python Interpreter Version: ${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}")

target_link_libraries(tpbr_python PUBLIC local_tools)

# fix for clang
# see: https://github.com/pybind/pybind11/issues/1818
if (CMAKE_CXX_COMPILER_ID STREQUAL "Clang")
    target_compile_options(tpbr_python PUBLIC -fno-sized-deallocation)
endif ()

# make sure to output the build file to tpbr_python folder
SET_TARGET_PROPERTIES(tpbr_python
        PROPERTIES
        PREFIX ""
        LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/tpbr_python"
        )

# copy package __init__.py file
configure_file(tpbr_python/__init__.py
        ${CMAKE_CURRENT_BINARY_DIR}/tpbr_python/__init__.py
        )

# copy setup.py file
configure_file(setup.py.in
        ${CMAKE_CURRENT_BINARY_DIR}/setup.py
        )

file(COPY .
        DESTINATION .
        FILES_MATCHING
        PATTERN *.py)