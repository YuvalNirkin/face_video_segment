add_executable(fvs_segment fvs_segment.cpp)
target_include_directories(fvs_segment PRIVATE 
	${CMAKE_SOURCE_DIR}/face_video_segment
	${CMAKE_BINARY_DIR}/face_video_segment)
target_link_libraries(fvs_segment PRIVATE face_video_segment)

# Installation
install(TARGETS fvs_segment
	 EXPORT face_video_segment-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
