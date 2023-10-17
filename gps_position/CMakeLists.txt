set(Lib_name gps_position_lib)

#find argus set .h and .so file dir
set(EXTERNAL_PROJECT_DIR ${CMAKE_BINARY_DIR}/install)
set(EXTERNAL_PROJECT_NAME VulcanSerial)
set(GIT_URL "https://github.com/Vulcan-YJX/VulcanSerial.git")
set(GIT_DIR_NAME "VulcanSerial")
set(GIT_TAG "master")
include(ExternalProject)

externalproject_add(${EXTERNAL_PROJECT_NAME}
    DOWNLOAD_COMMAND git clone --branch ${GIT_TAG} ${GIT_URL}
    DOWNLOAD_DIR ${EXTERNAL_PROJECT_DIR}
    URL ${RELEASE_FILE_URL}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ""
    LOG_DOWNLOAD OFF
)

set(INCLUDE_DIR ${EXTERNAL_PROJECT_DIR}/${GIT_DIR_NAME}/include/VulcanSerial)
set(SRC_DIR ${EXTERNAL_PROJECT_DIR}/${GIT_DIR_NAME}/src)

include_directories(${INCLUDE_DIR})
add_library(vulcan_serial SHARED
    ${SRC_DIR}/SerialPort.cpp
)

include_directories(include)
add_library(${Lib_name} SHARED
    src/gps_coordinate.cpp
)
target_link_libraries(${Lib_name} vulcan_serial)