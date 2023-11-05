set(CMAKE_CXX_STANDARD 11)

include_directories(include)

add_executable(test_gst_cam test_camera_stereo.cpp)
target_link_directories(test_gst_cam vulcan_vo_lib)
