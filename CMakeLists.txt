cmake_minimum_required(VERSION 2.8)
project(face_video_segment)

# Version
# ===================================================
set(FACE_VIDEO_SEGMENT_MAJOR_VERSION 0)
set(FACE_VIDEO_SEGMENT_MINOR_VERSION 5)
set(FACE_VIDEO_SEGMENT_VERSION ${MFACE_VIDEO_SEGMENT_MAJOR_VERSION}.${FACE_VIDEO_SEGMENT_MINOR_VERSION})

# Global configurations
# ===================================================
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
if(UNIX)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
else()
	 SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX")
endif()

# Find dependencies
# ===================================================
# Boost
if(MSVC)
	set(Boost_USE_STATIC_LIBS ON)	# only find static libs
endif(MSVC)
set(Boost_USE_MULTITHREADED ON)
find_package( Boost REQUIRED program_options filesystem)
if(Boost_FOUND)
	include_directories(${Boost_INCLUDE_DIRS})
	if(MSVC)
		# On Windows the Boost libraries are automatically linked
		link_directories(${Boost_LIBRARY_DIRS})
	endif(MSVC)	
endif()

find_package(OpenCV REQUIRED highgui imgproc imgcodecs)
find_package(dlib REQUIRED)
find_package(glog REQUIRED)
find_package(video_segment REQUIRED)

# Add target
# ===================================================
add_executable(face_video_segment face_video_segment.cpp FaceViewer.h FaceViewer.cpp landmarks_unit.h landmarks_unit.cpp
	face_segmentation_unit.h face_segmentation_unit.cpp)
target_include_directories(face_video_segment PRIVATE ${VIDEO_SEGMENT_INCLUDE_DIRS} ${gflags_INCLUDE_DIRS} ${dlib_INCLUDE_DIRS})
target_link_libraries(face_video_segment PRIVATE ${VIDEO_SEGMENT_LIBRARIES} ${gflags_LIBRARIES} ${dlib_LIBRARIES})