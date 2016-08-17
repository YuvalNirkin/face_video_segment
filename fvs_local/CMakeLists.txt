add_executable(fvs_local fvs_local.cpp FaceViewer.h FaceViewer.cpp)
target_include_directories(fvs_local PRIVATE ${CMAKE_SOURCE_DIR}/face_video_segment)
target_link_libraries(fvs_local PRIVATE face_video_segment)

# Installation
install(TARGETS fvs_local
	 EXPORT face_video_segment-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
