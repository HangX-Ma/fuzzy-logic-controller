cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(fuzzy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set (CMAKE_C_COMPILER "E:/mingw64/bin/gcc.exe")
set (CMAKE_CXX_COMPILER "E:/mingw64/bin/g++.exe")

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "Please select another folder to build the project.")
endif()

# configurations for vscode CMake tools
if("${CMAKE_BUILD_TYPE}" STREQUAL "Release")
    message(STATUS "Maximum optimization for speed")
    add_compile_options(-Ofast)
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "RelWithDebInfo")
    message(STATUS "Maximum optimization for speed, debug info included")
    add_compile_options(-Ofast -g)
elseif("${CMAKE_BUILD_TYPE}" STREQUAL "MinSizeRel")
    message(STATUS "Maximum optimization for size")
    add_compile_options(-Os)
else()
    message(STATUS "Minimal optimization, debug info included")
    add_compile_options(-Og -g)
endif()

add_compile_options(
    -Wall # print warning info
    -Wextra # print extra warning info
    -Wconversion # data conversion truncation warning
    -Wsign-conversion # sign <-> unsigned conversion warning
)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

file(GLOB_RECURSE SOURCES
    "src/*.*"
)

find_package(Python3 REQUIRED
    COMPONENTS
        Interpreter
        Development
        NumPy
)

add_executable(${PROJECT_NAME} ${SOURCES})

target_link_libraries(${PROJECT_NAME} ${Python3_LIBRARIES} ${Python3_NumPy_LIBRARIES})

target_include_directories(${PROJECT_NAME}
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/include/fuzzy
        ${CMAKE_SOURCE_DIR}/library
        ${Python3_INCLUDE_DIRS}
        ${Python3_NumPy_INCLUDE_DIRS}
)

# build doxygen
message(STATUS "Build documentation open")
include(cmake/build_doxygen.cmake)
build_doxygen()
