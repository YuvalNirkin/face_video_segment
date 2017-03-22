cmake_minimum_required(VERSION 3.0.2)
project(face_video_segment)

# Version
# ===================================================
set(FACE_VIDEO_SEGMENT_MAJOR_VERSION 0)
set(FACE_VIDEO_SEGMENT_MINOR_VERSION 8)
set(FACE_VIDEO_SEGMENT_VERSION ${FACE_VIDEO_SEGMENT_MAJOR_VERSION}.${FACE_VIDEO_SEGMENT_MINOR_VERSION})

# Global configurations
# ===================================================
if(WIN32)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DNOMINMAX")
	set(DEF_INSTALL_CMAKE_DIR cmake)
else()
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	set(DEF_INSTALL_CMAKE_DIR lib/cmake/nirkin_face)
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
	"Installation directory for CMake files")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Optional 3rd party components
# ===================================================
option(WITH_BOOST_STATIC "Boost static libraries" ON)

# Build components
# ===================================================
option(BUILD_DOCS "Build documentation using Doxygen" ON)
option(BUILD_INTERFACE_MATLAB "Build interface for Matlab" ON)

# Find dependencies
# ===================================================
# Boost
set(Boost_USE_STATIC_LIBS ${WITH_BOOST_STATIC})
set(BOOST_ALL_DYN_LINK NOT ${WITH_BOOST_STATIC})
if(WIN32)
	if(${WITH_BOOST_STATIC})
		add_definitions(-DBOOST_ALL_NO_LIB)
	else()
		add_definitions(-DBOOST_ALL_DYN_LINK)
	endif()
endif()
find_package(Boost REQUIRED filesystem program_options)

find_package(OpenCV REQUIRED highgui imgproc imgcodecs)
find_package(dlib REQUIRED)
find_package(glog REQUIRED)
find_package(protobuf REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(video_segment REQUIRED)
find_package(find_face_landmarks REQUIRED)

# Doxygen
find_package(Doxygen)

# Process subdirectories
# ===================================================
add_subdirectory(face_video_segment)
add_subdirectory(fvs_find_regions)
add_subdirectory(fvs_segment)
add_subdirectory(fvs_write_keyframes)
add_subdirectory(fvs_editor)

# interfaces
if(BUILD_INTERFACE_MATLAB)
	add_subdirectory(interfaces/matlab)
endif()

# Documentation
if(BUILD_DOCS)
	add_subdirectory(doc)
endif()

# Export configuration
# ===================================================

# Add all targets to the build-tree export set
export(TARGETS face_video_segment
  FILE "${PROJECT_BINARY_DIR}/face_video_segment-targets.cmake")
  
# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE face_video_segment)

# Create config files
configure_file(cmake/face_video_segment-config.cmake.in
  "${PROJECT_BINARY_DIR}/face_video_segment-config.cmake" @ONLY)
configure_file(cmake/face_video_segment-config-version.cmake.in
  "${PROJECT_BINARY_DIR}/face_video_segment-config-version.cmake" @ONLY)
  
# Install config files
install(FILES
  "${PROJECT_BINARY_DIR}/face_video_segment-config.cmake"
  "${PROJECT_BINARY_DIR}/face_video_segment-config-version.cmake"
  DESTINATION "cmake" COMPONENT dev)
  
# Install the export set for use with the install-tree
install(EXPORT face_video_segment-targets DESTINATION cmake COMPONENT dev)