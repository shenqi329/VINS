cmake_minimum_required(VERSION 3.4.1)

SET(CMAKE_BUILD_TYPE Release)

set(magicpen_SRCS
        src/MagicPenMaLiang.cpp
        src/polypartition.cpp
        src/MagicPenRender.cpp
        src/MagicPen3DModel.cpp
        )

set(TARGET_NAME magicpen)
add_library(
        ${TARGET_NAME}
        STATIC
        ${magicpen_SRCS}
)

SET(OpenCV_DIR D:/dev/opencv-4.4.0-android-sdk/sdk/native/jni)
find_package(OpenCV REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/)
target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(
        ${TARGET_NAME}
        EGL
        GLESv3
        ${OpenCV_LIBS}
)
