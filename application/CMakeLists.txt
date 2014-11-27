# provide path to dependency libraries

INCLUDE_DIRECTORIES(${CMAKE_SOURCE_DIR}/include)
#INCLUDE_DIRECTORIES(${XENOMAI_RTDM_INCLUDE_DIR})

LINK_DIRECTORIES(${${PROJECT_NAME}_BINARY_DIR}/src)
LINK_DIRECTORIES(/usr/xenomai/lib)

ADD_EXECUTABLE(test-vsa1modelserver vsa1modelserver.cpp)
TARGET_LINK_LIBRARIES(test-vsa1modelserver vsa1)
PKG_CONFIG_USE_DEPENDENCY (test-vsa1modelserver libxenomai_rtdm)
PKG_CONFIG_USE_DEPENDENCY (test-vsa1modelserver libxenomai_native)
