cmake_minimum_required(VERSION 3.4.1)

set(ov2slam_SRCS
        src/camera_calibration.cpp
        src/ceres_parametrization.cpp
        src/estimator.cpp
        src/feature_extractor.cpp
        src/feature_tracker.cpp
        src/frame.cpp
        src/logger.cpp
        src/loop_closer.cpp
        src/map_manager.cpp
        src/map_point.cpp
        src/mapper.cpp
        src/multi_view_geometry.cpp
        src/optimizer.cpp
        src/ov2slam.cpp
        src/slam_params.cpp
        src/visual_front_end.cpp
        )


set(omp_binary_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../src/main/jniLibs/${ANDROID_ABI}/libomp.a)
add_library(
        omp_LIB
        STATIC
        IMPORTED
        GLOBAL
)
set_target_properties(
        omp_LIB
        PROPERTIES IMPORTED_LOCATION
        ${omp_binary_dir}
)


set(TARGET_NAME ov2slam)
add_library(
        ${TARGET_NAME}
        STATIC
        ${ov2slam_SRCS}
)

SET(OpenCV_DIR D:/dev/opencv-4.4.0-android-sdk/sdk/native/jni)
find_package(OpenCV REQUIRED)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include/ceres_parametrization)
target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(
        ${TARGET_NAME}
        eigen3_LIB
        sophus_LIB
        ceres_LIB
        omp_LIB
        ${OpenCV_LIBS}
)


add_subdirectory(Thirdparty)
add_subdirectory(Thirdparty/Sophus)