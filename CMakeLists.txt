cmake_minimum_required ( VERSION 2.6 )

project (Panoramix )

# sources extension
set(SRCEXT "cpp")

# required packages
find_package ( OpenCV REQUIRED )

macro( config_project PROJNAME LIBNAME )
	include_directories( ${${LIBNAME}_INCLUDE_DIR} )
	target_link_libraries( ${PROJNAME} ${${LIBNAME}_LIBRARY} )
endmacro()

add_executable( Panoramix main.cpp )

target_link_libraries(
	Panoramix
	${OpenCV_LIBS}
)

