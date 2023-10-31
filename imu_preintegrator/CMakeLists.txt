set(CMAKE_CXX_STANDARD 11)

set(LIB_NAME vulcan_imu_lib)

include_directories(include)

add_library(${LIB_NAME} SHARED
    src/FusionAhrs.c
    src/FusionOffset.c
    src/FusionCompass.c
)
