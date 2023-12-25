cmake_minimum_required(VERSION 3.8 FATAL_ERROR)

project(matplotlib_cpp LANGUAGES CXX)

# matplotlib Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
    INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_compile_features(matplotlib_cpp INTERFACE
    cxx_std_17
)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
    Python3::Python
    Python3::Module
)

find_package(Python3 COMPONENTS NumPy)

if(Python3_NumPy_FOUND)
    target_link_libraries(matplotlib_cpp INTERFACE
        Python3::NumPy
    )
else()
    target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()