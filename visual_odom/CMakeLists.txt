set(CMAKE_CXX_STANDARD 11)

set(LIB_NAME vulcan_vo_lib)

find_package(vpi 2.3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

include_directories(include)

add_library(${LIB_NAME} SHARED
    src/optflow_lk.cpp
    src/optflow_dense.cpp
)

target_link_libraries(${LIB_NAME} vpi opencv_core
opencv_imgproc opencv_imgcodecs opencv_videoio)