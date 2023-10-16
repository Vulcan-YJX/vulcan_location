set(Lib_name vulcan_wheel_lib)

include_directories(include)

add_library(${Lib_name} SHARED 
    src/differential_wheel.cpp
)

target_include_directories(${Lib_name} PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)