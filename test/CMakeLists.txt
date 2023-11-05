set(CMAKE_CXX_STANDARD 11)

set(LIB_NAME test)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(test_stereo_cam test_camera_stereo.cpp)
target_link_libraries(test_stereo_cam vulcan_vo_lib ${OpenCV_LIBRARIES})
target_include_directories(test_stereo_cam PRIVATE ${CMAKE_BINARY_DIR}/include)