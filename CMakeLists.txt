cmake_minimum_required(VERSION 3.0.2)
project(insta360)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-D__STDC_CONSTANT_MACROS")
set(INSTA_INCLUDE_DIR ./include/insta360)
set(INSTA_SDK /home/nathan/ros/melodic/catkin_ws/src/insta360/lib/libCameraSDK.so) #TODO: set absolute path to Insta SDK lib

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  message_filters
  sensor_msgs
)

find_package(PkgConfig REQUIRED)
find_package(OpenCV REQUIRED)

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
        
        
        
        
    
      



