add_executable(fvs_find_regions fvs_find_regions.cpp)
target_include_directories(fvs_find_regions PRIVATE 
	${CMAKE_SOURCE_DIR}/face_video_segment
	${CMAKE_BINARY_DIR}/face_video_segment)
target_link_libraries(fvs_find_regions PRIVATE face_video_segment)

# Installation
install(TARGETS fvs_find_regions
	 EXPORT face_video_segment-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
