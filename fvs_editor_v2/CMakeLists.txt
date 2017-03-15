# Source
set(FVS_EDITOR_SRC fvs_editor_main.cpp fvs_editor.cpp 
	fvs_editor_states.h fvs_editor_states.cpp)
set(FVS_EDITOR_HDR fvs_editor.h)
qt5_wrap_cpp(FVS_EDITOR_HDR_MOC ${FVS_EDITOR_HDR})
qt5_wrap_ui(FVS_EDITOR_UI_MOC fvs_editor.ui)
qt5_add_resources(FVS_EDITOR_QRC fvs_editor.qrc)

if(MSVC)
	set(FVS_EDITOR_SRC ${FVS_EDITOR_SRC} fvs_editor.rc)
endif()

# Target
add_executable(fvs_editor ${FVS_EDITOR_SRC} ${FVS_EDITOR_HDR}
	${FVS_EDITOR_HDR_MOC} ${FVS_EDITOR_UI_MOC} ${FVS_EDITOR_QRC})
target_include_directories(fvs_editor PRIVATE 
	${CMAKE_SOURCE_DIR}/face_video_segment
	${CMAKE_BINARY_DIR}/face_video_segment
	${CMAKE_SOURCE_DIR}/fvs_editor
	${CMAKE_BINARY_DIR}/fvs_editor)
target_link_libraries(fvs_editor PRIVATE face_video_segment)

# Installation
install(TARGETS fvs_editor
	 EXPORT face_video_segment-targets
     RUNTIME DESTINATION bin COMPONENT dev
     LIBRARY DESTINATION lib COMPONENT dev
     ARCHIVE DESTINATION lib COMPONENT dev)