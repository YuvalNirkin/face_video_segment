# Source
set(FVS_SRC 
	landmarks_unit.cpp
	video_reader_unit2.cpp
	video_writer_unit2.cpp
	utilities.cpp
	face_segmentation_unit.cpp
	face_regions.cpp
	keyframer.cpp
	keyframe_unit.cpp
)
set(FVS_INCLUDE
	landmarks_unit.h
	video_reader_unit2.h
	video_writer_unit2.h
	utilities.h
	face_segmentation_unit.h
	face_regions.h
	keyframer.h
	keyframe_unit.h
)
set(PROTO_FILES face_video_segment.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
set(FVS_SRC ${FVS_SRC} ${PROTO_SRCS} ${PROTO_FILES})
set(FVS_INCLUDE ${FVS_INCLUDE} ${PROTO_HDRS})

# Target
add_library(face_video_segment ${FVS_SRC} ${FVS_INCLUDE})
target_include_directories(face_video_segment PRIVATE 
	${CMAKE_BINARY_DIR}/face_video_segment
)
target_include_directories(face_video_segment PUBLIC 
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}>
	${VIDEO_SEGMENT_INCLUDE_DIRS}
	${gflags_INCLUDE_DIRS}
	${dlib_INCLUDE_DIRS}
	${FIND_FACE_LANDMARKS_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS}
)
target_link_libraries(face_video_segment PUBLIC 
	${VIDEO_SEGMENT_LIBRARIES}
	${gflags_LIBRARIES}
	${dlib_LIBRARIES}
	${FIND_FACE_LANDMARKS_LIBRARIES}
	${Boost_LIBRARIES}
)

# Installation
install(TARGETS face_video_segment
	 EXPORT face_video_segment-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
