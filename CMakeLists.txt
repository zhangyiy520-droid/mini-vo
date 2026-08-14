cmake_minimum_required(VERSION 3.16)
project(mini_vo VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus CONFIG REQUIRED)
find_package(g2o CONFIG REQUIRED)

# --- Library ---
add_library(mini_vo STATIC
    src/initializer.cpp
    src/tracker.cpp
    src/vo_system.cpp
    src/pose_utils.cpp
    src/core/observation.cpp
    src/core/map.cpp
    src/core/se3_utils.cpp
    src/backend/g2o_graph_builder.cpp
    src/backend/pose_optimizer.cpp
    src/backend/point_optimizer.cpp
    src/backend/robust_policy.cpp
    src/backend/bundle_adjuster.cpp
    src/backend/local_window_selector.cpp
    src/backend/local_bundle_adjuster.cpp
    src/backend/backend.cpp
    src/camera/pinhole_camera.cpp
    src/backend/reprojection_error.cpp
)
target_include_directories(mini_vo PUBLIC include)
target_link_libraries(mini_vo PUBLIC
    ${OpenCV_LIBS}
    Eigen3::Eigen
    Sophus::Sophus
    g2o::core
    g2o::types_sba
    g2o::solver_dense
)

# --- Main executable ---
add_executable(vo_bin src/main.cpp)
target_link_libraries(vo_bin mini_vo)

# --- Learning executable: Day35 Gauss-Newton ---
add_executable(day35_gauss_newton apps/day35_gauss_newton.cpp)
target_link_libraries(day35_gauss_newton PRIVATE mini_vo Eigen3::Eigen)

# --- Tests (optional) ---
option(BUILD_TESTS "Build unit tests" OFF)
if(BUILD_TESTS)
    enable_testing()

    add_executable(test_initializer test/test_initializer.cpp)
    target_link_libraries(test_initializer PRIVATE mini_vo)
    add_test(NAME test_initializer COMMAND test_initializer)

    add_executable(test_tracker test/test_tracker.cpp)
    target_link_libraries(test_tracker PRIVATE mini_vo)
    add_test(NAME test_tracker COMMAND test_tracker)

    add_executable(test_observation test/test_observation.cpp)
    target_link_libraries(test_observation PRIVATE mini_vo)
    add_test(NAME test_observation COMMAND test_observation)

    add_executable(test_map test/test_map.cpp)
    target_link_libraries(test_map PRIVATE mini_vo)
    add_test(NAME test_map COMMAND test_map)

    add_executable(test_camera test/test_camera.cpp)
    target_link_libraries(test_camera PRIVATE mini_vo)
    add_test(NAME test_camera COMMAND test_camera)

    add_executable(test_reprojection_error test/test_reprojection_error.cpp)
    target_link_libraries(test_reprojection_error PRIVATE mini_vo)
    add_test(NAME test_reprojection_error COMMAND test_reprojection_error)

    add_executable(test_se3_utils test/test_se3_utils.cpp)
    target_link_libraries(test_se3_utils PRIVATE mini_vo)
    add_test(NAME test_se3_utils COMMAND test_se3_utils)

    add_executable(test_g2o_graph_builder test/test_g2o_graph_builder.cpp)
    target_link_libraries(test_g2o_graph_builder PRIVATE mini_vo)
    add_test(NAME test_g2o_graph_builder COMMAND test_g2o_graph_builder)

    add_executable(test_pose_optimizer test/test_pose_optimizer.cpp)
    target_link_libraries(test_pose_optimizer PRIVATE mini_vo)
    add_test(NAME test_pose_optimizer COMMAND test_pose_optimizer)

    add_executable(test_point_optimizer test/test_point_optimizer.cpp)
    target_link_libraries(test_point_optimizer PRIVATE mini_vo)
    add_test(NAME test_point_optimizer COMMAND test_point_optimizer)

    add_executable(test_robust_policy test/test_robust_policy.cpp)
    target_link_libraries(test_robust_policy PRIVATE mini_vo)
    add_test(NAME test_robust_policy COMMAND test_robust_policy)

    add_executable(test_bundle_adjuster test/test_bundle_adjuster.cpp)
    target_link_libraries(test_bundle_adjuster PRIVATE mini_vo)
    add_test(NAME test_bundle_adjuster COMMAND test_bundle_adjuster)

    add_executable(test_local_window_selector test/test_local_window_selector.cpp)
    target_link_libraries(test_local_window_selector PRIVATE mini_vo)
    add_test(NAME test_local_window_selector COMMAND test_local_window_selector)

    add_executable(test_local_bundle_adjuster test/test_local_bundle_adjuster.cpp)
    target_link_libraries(test_local_bundle_adjuster PRIVATE mini_vo)
    add_test(NAME test_local_bundle_adjuster COMMAND test_local_bundle_adjuster)

    add_executable(test_backend test/test_backend.cpp)
    target_link_libraries(test_backend PRIVATE mini_vo)
    add_test(NAME test_backend COMMAND test_backend)
endif()
