cmake_minimum_required(VERSION 3.4.1)

set(msckf_mono_SRCS
        src/corner_detector.cpp
        )


set(TARGET_NAME msckf_mono)
add_library(
        ${TARGET_NAME}
        STATIC
        ${msckf_mono_SRCS}
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(
        ${TARGET_NAME}
        fast
        eigen_LIB
        ${OpenCV_LIBS}
)
