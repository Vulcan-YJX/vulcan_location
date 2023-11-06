set(CMAKE_CXX_STANDARD 11)

set(LIB_NAME vulcan_vo_lib)

# find_package(vpi 2.3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)
# find_package(vpi REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(${LIB_NAME} SHARED
    # src/optflow_lk.cpp
    # src/optflow_dense.cpp
    # src/feature_extraction/point_generation.cpp
    # src/feature_extraction/orb_detector.cpp
    # src/feature_extraction/stereo_depth.cpp
    src/camera/camera.cpp
    src/camera/camera_info.cpp
)

target_link_libraries(${LIB_NAME} ${OpenCV_LIBRARIES} yaml-cpp)

install(TARGETS ${LIB_NAME}
        LIBRARY DESTINATION ${CMAKE_BINARY_DIR}/lib)

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/include
        DESTINATION ${CMAKE_BINARY_DIR})

file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/config
        DESTINATION ${CMAKE_BINARY_DIR})