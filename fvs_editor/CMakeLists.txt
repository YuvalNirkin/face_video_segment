set(FVS_EDITOR_SRC fvs_editor.cpp editor.cpp editor_ui.cpp)
set(FVS_EDITOR_HDR editor.h)
qt5_wrap_cpp(FVS_EDITOR_HDR_MOC ${FVS_EDITOR_HDR})
qt5_add_resources(FVS_EDITOR_QRC fvs_editor.qrc)

add_executable(fvs_editor ${FVS_EDITOR_SRC} ${FVS_EDITOR_HDR}
	${FVS_EDITOR_HDR_MOC} ${FVS_EDITOR_QRC})
target_include_directories(fvs_editor PRIVATE 
	${CMAKE_SOURCE_DIR}/face_video_segment
	${CMAKE_BINARY_DIR}/face_video_segment)
target_link_libraries(fvs_editor PRIVATE face_video_segment)

# Installation
install(TARGETS fvs_editor
	 EXPORT face_video_segment-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)
