cmake_minimum_required(VERSION 3.3)

project(
    i2c-slave
    LANGUAGES C
)

# Place all cmake-compiled libraries and executables in the top level build directory.
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

add_subdirectory(smbus)
add_subdirectory(test)

if(CMAKE_COMPILER_IS_GNUCC)
    add_compile_options(-Wall --Werror)
endif()
