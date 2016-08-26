add_executable(fvs_write_keyframes fvs_write_keyframes.cpp)
target_include_directories(fvs_write_keyframes PRIVATE 
	${CMAKE_SOURCE_DIR}/face_video_segment
	${CMAKE_BINARY_DIR}/face_video_segment)
target_link_libraries(fvs_write_keyframes PRIVATE face_video_segment)

# Installation
install(TARGETS fvs_write_keyframes
	 EXPORT face_video_segment-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
