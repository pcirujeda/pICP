cmake_minimum_required( VERSION 3.5 )

PROJECT( PlainICP )

find_package( OpenCV REQUIRED )
find_package( Boost COMPONENTS program_options REQUIRED )
include_directories(
	include
	${Boost_INCLUDE_DIR}
)

add_executable( PlainICP src/PlainICP.cxx )
target_link_libraries( PlainICP ${OpenCV_LIBRARIES} ${Boost_LIBRARIES} )
