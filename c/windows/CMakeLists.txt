cmake_minimum_required(VERSION 3.12)
project(ui2c_project)

set(CMAKE_SUPPRESS_REGENERATION ON)
set(CMAKE_USE_RELATIVE_PATHS ON)

# Set the source files for the libraries and test application
set(LIBRARY_SOURCES
    ui2c.c
)

set(TEST_SOURCES
    test.c
)

# Set the include directories
include_directories(
    .
    ../../arduino
    ../inc
    ../..
)

# Create the static library
add_library(ui2c_static STATIC ${LIBRARY_SOURCES})
set_target_properties(ui2c_static PROPERTIES OUTPUT_NAME "ui2c_static")

# Create the shared library
add_library(ui2c_shared SHARED ${LIBRARY_SOURCES})
set_target_properties(ui2c_shared PROPERTIES OUTPUT_NAME "ui2c_shared")
if (MSVC)    
    target_sources(ui2c_static PRIVATE ui2c.def)
    target_sources(ui2c_shared PRIVATE ui2c.def)
endif()

# Create the test application
add_executable(test_app ${TEST_SOURCES})
target_link_libraries(test_app PRIVATE ui2c_static)

# Set the output directories
set_target_properties(ui2c_static ui2c_shared test_app
    PROPERTIES
    ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin
)

# Set compiler flags
target_compile_options(ui2c_static PRIVATE /W4 /WX)
target_compile_options(ui2c_shared PRIVATE /W4 /WX)
target_compile_options(test_app PRIVATE /W4 /WX)

# Set MSVC-specific settings
if(MSVC)
    # Enable multi-threaded compilation
    target_compile_options(ui2c_static PRIVATE /MP)
    target_compile_options(ui2c_shared PRIVATE /MP)
    target_compile_options(test_app PRIVATE /MP)

    # Use static runtime libraries
    foreach(target_var ui2c_static ui2c_shared test_app)
        string(REPLACE "/MD" "/MT" new_flags "${CMAKE_C_FLAGS_${target_var}}")
        set(CMAKE_C_FLAGS_${target_var} "${new_flags}" CACHE STRING "")
    endforeach()
endif()
