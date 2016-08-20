# Source
set(FVS_SRC landmarks_unit.cpp face_segmentation_unit.cpp video_writer_unit2.cpp
	keyframe_writer_unit.cpp utilities.cpp)
set(FVS_INCLUDE landmarks_unit.h face_segmentation_unit.h video_writer_unit2.h
	keyframe_writer_unit.h utilities.h)
set(PROTO_FILES face_video_segment.proto)
protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${PROTO_FILES})
set(FVS_SRC ${FVS_SRC} ${PROTO_SRCS} ${PROTO_FILES})
set(FVS_INCLUDE ${FVS_INCLUDE} ${PROTO_HDRS})

# Target
add_library(face_video_segment ${FVS_SRC} ${FVS_INCLUDE})
target_include_directories(face_video_segment PUBLIC 
	${VIDEO_SEGMENT_INCLUDE_DIRS} ${gflags_INCLUDE_DIRS} ${dlib_INCLUDE_DIRS} ${FIND_FACE_LANDMARKS_INCLUDE_DIRS})
target_link_libraries(face_video_segment PUBLIC 
	${VIDEO_SEGMENT_LIBRARIES} ${gflags_LIBRARIES} ${dlib_LIBRARIES} ${FIND_FACE_LANDMARKS_LIBRARIES})

# Installation
install(TARGETS face_video_segment
	 EXPORT face_video_segment-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
