# Target
add_executable(fvs_segment fvs_segment.cpp)
target_link_libraries(fvs_segment PRIVATE face_video_segment)
target_compile_definitions(fvs_segment PRIVATE 
	-DWITH_QT
	-DWITH_OPENCV_VIDEOIO
)

# Installation
install(TARGETS fvs_segment
	 EXPORT face_video_segment-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
