cmake_minimum_required(VERSION 3.18 FATAL_ERROR)

project(fuzzy LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

MESSAGE(STATUS "operation system is ${CMAKE_SYSTEM}")

if(CMAKE_SYSTEM_NAME MATCHES "Windows")
    set(CMAKE_C_COMPILER "E:/mingw64/bin/gcc.exe")
    set(CMAKE_CXX_COMPILER "E:/mingw64/bin/g++.exe")
endif()

# generate a compile commands file as complete database for vim-YouCompleteMe or some other similar tools
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

    # -Wconversion # data conversion truncation warning
    # -Wsign-conversion # sign <-> unsigned conversion warning
)

# build shared option
option(CRISP_BUILD_SHARED "Build shared library" OFF)

add_definitions(
    "-DFC_USE_DOUBLE"
    "-DFC_USE_MATPLOTLIB"

    # "-DFC_USE_DEBUG_MSG"
    "-DFC_USE_INFO_MSG"
    "-DFC_USE_WARN_MSG"
)

set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

add_subdirectory(library)

# fuzzy library
file(GLOB_RECURSE FUZZY_SOURCES "src/*.*")
file(GLOB FUZZY_HEADERS "${CMAKE_CURRENT_SOURCE_DIR}/include/fuzzy/*.h")
source_group("Header Files\\fuzzy" FILES ${FUZZY_HEADERS})

if(CRISP_BUILD_SHARED OR BUILD_SHARED_LIBS)
    add_library(fuzzy SHARED ${FUZZY_SOURCES} ${FUZZY_HEADERS})
else()
    add_library(fuzzy STATIC ${FUZZY_SOURCES} ${FUZZY_HEADERS})
endif()

target_link_libraries(fuzzy matplotlib_cpp fmt-header-only Eigen3::Eigen)
target_include_directories(
    fuzzy PUBLIC
    ${CMAKE_SOURCE_DIR}/include
)

# example
add_executable(fuzzy_demo example/demo.cc)
target_link_libraries(fuzzy_demo PRIVATE fuzzy)

# build doxygen
option(DOXYGEN "Compile doxygen files" OFF)

if(DOXYGEN)
    message(STATUS "Build documentation open")
    include(cmake/build_doxygen.cmake)
    build_doxygen()
endif()