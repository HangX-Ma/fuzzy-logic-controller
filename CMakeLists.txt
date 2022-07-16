cmake_minimum_required(VERSION 3.11 FATAL_ERROR)

project(fuzzy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_BUILD_TYPE DEBUG)

set (CMAKE_C_COMPILER "E:/mingw64/bin/gcc.exe")
set (CMAKE_CXX_COMPILER "E:/mingw64/bin/g++.exe")

if(${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_BINARY_DIR})
    message(FATAL_ERROR "Please select another folder to build the project.")
endif()


set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_POSITION_INDEPENDENT_CODE ON) 


add_library(fc SHARED "")

target_compile_options(fc INTERFACE -std=c++11 -fPIC -Wall)

target_include_directories(fc
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
    )

target_sources(fc
    PRIVATE
        ${CMAKE_SOURCE_DIR}/src/FuzzyControl.cpp
        ${CMAKE_SOURCE_DIR}/src/Membership.cpp
    )

set_target_properties(fc PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)


add_executable(fuzzy src/example.cpp library/matplotlibcpp.h)

target_link_libraries(fuzzy INTERFACE fc)

target_include_directories(fuzzy
    PRIVATE
        ${CMAKE_SOURCE_DIR}/include
    )