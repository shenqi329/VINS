cmake_minimum_required(VERSION 3.4)

set(sophus_include_dir ${CMAKE_CURRENT_SOURCE_DIR})
set(thirdpartylibs_include_dir ${CMAKE_CURRENT_SOURCE_DIR})
add_library(sophus_LIB INTERFACE)
target_include_directories(
        sophus_LIB
        INTERFACE
        ${sophus_include_dir}
        ${thirdpartylibs_include_dir}
)
