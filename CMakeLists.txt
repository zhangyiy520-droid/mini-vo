cmake_minimum_required(VERSION 3.16)
project(mini_vo VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Dependencies ---
find_package(OpenCV REQUIRED)

# --- Library ---
add_library(mini_vo STATIC
    src/initializer.cpp
    src/tracker.cpp
    src/vo_system.cpp
    src/pose_utils.cpp
    src/core/observation.cpp
    src/core/map.cpp
    src/camera/pinhole_camera.cpp
    src/backend/reprojection_error.cpp
)
target_include_directories(mini_vo PUBLIC include)
target_link_libraries(mini_vo PUBLIC ${OpenCV_LIBS})

# --- Main executable ---
add_executable(vo_bin src/main.cpp)
target_link_libraries(vo_bin mini_vo)

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
endif()
