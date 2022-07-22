set(TARGET_NAME VINS_Android)
set(${TARGET_NAME}_SRCS
    ViewController.cpp
)

SET(OpenCV_DIR D:/dev/opencv-4.4.0-android-sdk/sdk/native/jni)
find_package(OpenCV REQUIRED)


set(omp_binary_dir ${CMAKE_CURRENT_SOURCE_DIR}/../../jniLibs/${ANDROID_ABI}/libomp.a)
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

add_library(
    ${TARGET_NAME}
    STATIC
    ${${TARGET_NAME}_SRCS}
)

target_link_libraries( 
    ${TARGET_NAME}
    VINSMobile
    android
    omp_LIB
    log
    ${OpenCV_LIBS}
)

target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})