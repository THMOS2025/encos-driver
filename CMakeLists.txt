cmake_minimum_required(VERSION 3.12)
project(encos_driver LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Build the library
set(LIB_SOURCES
    src/driver.c 
    src/constant.c
    src/encos_command.c 
    src/socket_can_driver.c
    deps/log.c/src/log.c
)

add_library(encos_driver_lib SHARED ${LIB_SOURCES})
set_target_properties(encos_driver_lib PROPERTIES 
    OUTPUT_NAME "encos_driver_lib"
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
)

target_include_directories(encos_driver_lib
    PUBLIC  include 
    PRIVATE src deps/log.c/src/
)


add_executable(encos_driver src/main.c)

target_link_libraries(encos_driver PRIVATE encos_driver_lib)
