add_executable(face_video_segment face_video_segment.cpp FaceViewer.h FaceViewer.cpp landmarks_unit.h landmarks_unit.cpp
	face_segmentation_unit.h face_segmentation_unit.cpp video_writer_unit2.h video_writer_unit2.cpp
	keyframe_writer_unit.h keyframe_writer_unit.cpp)
target_include_directories(face_video_segment PRIVATE 
	${VIDEO_SEGMENT_INCLUDE_DIRS} ${gflags_INCLUDE_DIRS} ${dlib_INCLUDE_DIRS} ${FIND_FACE_LANDMARKS_INCLUDE_DIRS})
target_link_libraries(face_video_segment PRIVATE 
	${VIDEO_SEGMENT_LIBRARIES} ${gflags_LIBRARIES} ${dlib_LIBRARIES} ${FIND_FACE_LANDMARKS_LIBRARIES})

# Installation
install(TARGETS face_video_segment
	 EXPORT face_video_segment-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
