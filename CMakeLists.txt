cmake_minimum_required(VERSION 3.10)
project(vulcan_location)

# Add subdirectory that contains another CMakeLists.txt
# add_subdirectory(wheel_odom)

# add_subdirectory(gps_position)

add_subdirectory(visual_odom)
# If the subdirectory exports a library, you can link against it
# add_executable(${PROJECT_NAME} main.cpp)
# target_link_libraries(${PROJECT_NAME}  mySubdirectoryLibrary)
