include( ExternalProject )

find_package( Threads REQUIRED )

# Download GoogleTest
ExternalProject_Add(
    gtest
    URL https://github.com/google/googletest/archive/master.zip
    PREFIX ${CMAKE_CURRENT_BINARY_DIR}/gtest
    INSTALL_COMMAND ""
)

# Get GTest source and binary directories from CMake project
ExternalProject_Get_Property(gtest source_dir binary_dir)

# Add GTest dependencies
add_library( libgtest IMPORTED STATIC GLOBAL )
add_library( libgmock IMPORTED STATIC GLOBAL )

add_dependencies( libgtest gtest )
add_dependencies( libgmock gtest )

set_target_properties( libgtest PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/lib/libgtest.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
)

set_target_properties( libgmock PROPERTIES
    "IMPORTED_LOCATION" "${binary_dir}/lib/libgmock.a"
    "IMPORTED_LINK_INTERFACE_LIBRARIES" "${CMAKE_THREAD_LIBS_INIT}"
)

include_directories(
    "${source_dir}/googletest/include"
    "${source_dir}/googlemock/include"
)

# Add test executable sources
set( TEST_SOURCE_FILES
    main.cpp
)

add_executable( testTarget ${TEST_SOURCE_FILES} )

target_link_libraries( testTarget
    libgtest
    libgmock
)

add_test(
    NAME testTarget
    COMMAND testTarget
)
