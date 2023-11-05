set(CMAKE_CXX_STANDARD 11)

set(LIB_NAME test)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
find_package(vpi 2.3 REQUIRED)

#test camera capture image.
add_executable(test_stereo_cam test_camera_stereo.cpp)
target_link_libraries(test_stereo_cam vulcan_vo_lib ${OpenCV_LIBRARIES})
target_include_directories(test_stereo_cam PRIVATE ${CMAKE_BINARY_DIR}/include)

#test vpi stereo depth.
add_executable(test_stereo_depth test_stereo_depth.cpp)
target_link_libraries(test_stereo_depth vulcan_vo_lib ${OpenCV_LIBRARIES} vpi)
target_include_directories(test_stereo_depth PRIVATE ${CMAKE_BINARY_DIR}/include)