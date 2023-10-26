set(CMAKE_CXX_STANDARD 11)

set(LIB_NAME vulcan_vo_lib)

# find_package(vpi 2.3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(include
    ${OpenCV_INCLUDE_DIRS}
)

add_library(${LIB_NAME} SHARED
    # src/optflow_lk.cpp
    # src/optflow_dense.cpp

    src/camera/camera.cpp
)

target_link_libraries(${LIB_NAME} ${OpenCV_LIBRARIES} yaml-cpp)