set(eigen_include_dir ${CMAKE_CURRENT_SOURCE_DIR}/eigen3)
# needed for eigen aswell because 2 different entry points for including are used
# (eigen3 & eigen3/Eigen)
set(thirdpartylibs_include_dir ${CMAKE_CURRENT_SOURCE_DIR})
add_library(eigen3_LIB INTERFACE)
target_include_directories(
        eigen3_LIB
        INTERFACE
        ${eigen_include_dir}
        ${thirdpartylibs_include_dir}
)

set(ceres_include_dir ${CMAKE_CURRENT_SOURCE_DIR}/ceres-solver/include)
set(ceres_miniglog_include_dir ${CMAKE_CURRENT_SOURCE_DIR}/ceres-solver/internal/ceres/miniglog)
set(ceres_binary_dir ${CMAKE_CURRENT_SOURCE_DIR}/ceres-solver/obj/local/${ANDROID_ABI}/libceres.a)
add_library(
        ceres_LIB
        STATIC
        IMPORTED
        GLOBAL
)
set_target_properties(
        ceres_LIB
        PROPERTIES IMPORTED_LOCATION
        ${ceres_binary_dir}
)

set_target_properties(
        ceres_LIB
        PROPERTIES INTERFACE_INCLUDE_DIRECTORIES
        "${ceres_include_dir};${ceres_miniglog_include_dir}"
)