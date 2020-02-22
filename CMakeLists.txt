cmake_minimum_required( VERSION 3.5 )

PROJECT( PlainICP )

set( CMAKE_CXX_STANDARD 17 )
set( CMAKE_CXX_STANDARD_REQUIRED TRUE )
set( CMAKE_CXX_EXTENSIONS OFF )

find_package( Boost COMPONENTS program_options REQUIRED )
find_package( Eigen3 REQUIRED )

include_directories(
	include
	${Boost_INCLUDE_DIRS}
	${EIGEN3_INCLUDE_DIRS}
)

add_executable( PlainICP src/PlainICP.cxx )
target_link_libraries( PlainICP ${Boost_LIBRARIES} )
