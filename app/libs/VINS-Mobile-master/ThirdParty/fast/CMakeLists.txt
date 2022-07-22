set(fast_SRCS
        src/fast_10.cpp
        src/fast_10_score.cpp
        src/nonmax_3x3.cpp
        )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

set(TARGET_NAME fast)
add_library(${TARGET_NAME} STATIC ${fast_SRCS})

target_include_directories(${TARGET_NAME} INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/include)