cmake_minimum_required(VERSION 3.0.2)
project(insta360)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-D__STDC_CONSTANT_MACROS")
set(INSTA_INCLUDE_DIR ./include/insta360)
set(INSTA_SDK /home/guillaume/Develop/Insta/CameraSDK-20231011_164338--1.1.0-Linux/lib/libCameraSDK.so) #TODO: set absolute path to Insta SDK lib

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  message_filters
  sensor_msgs
  visp_bridge
)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)
find_package(VISP REQUIRED)
if(VISP_FOUND)
	include(${VISP_USE_FILE})
endif(VISP_FOUND)

find_package(PER REQUIRED per_core per_io)
if(PER_FOUND)
	include(${PER_USE_FILE})
endif(PER_FOUND)


catkin_package()

pkg_check_modules(LIBAV REQUIRED IMPORTED_TARGET
        libavdevice
        libavfilter
        libavformat
        libavcodec
        libswresample
        libswscale
        libavutil
        )

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS} 
)


##### INSTA360 ONE X2 BRINGUP
add_executable(${PROJECT_NAME} src/insta360.cpp 
			    src/videoDecoder.cpp 
			    src/insta360Camera.cpp
        )
target_include_directories(${PROJECT_NAME} PRIVATE ${INSTA_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBS}
        ${OpenCV_LIBRARIES}
		${INSTA_SDK}
	    PkgConfig::LIBAV  -lpthread -ludev
)


##### DUAL-HEMISPHERICAL STITCHER
set(SOURCES src/dualhemispherical2equi.cpp )
add_executable(
		dualhemisphreical2equi
		${SOURCES}
)
target_link_libraries(
		dualhemisphreical2equi
		${catkin_LIBRARIES}
)
        
    
      



