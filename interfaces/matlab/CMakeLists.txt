# Installation
file(GLOB MATLAB_SCRIPTS "*.m")
install(FILES ${MATLAB_SCRIPTS}
		DESTINATION interfaces/matlab)
#[[
install(DIRECTORY ${CMAKE_SOURCE_DIR}/interfaces/matlab/utilities
			DESTINATION interfaces/matlab
			FILES_MATCHING PATTERN "*.m")
]]