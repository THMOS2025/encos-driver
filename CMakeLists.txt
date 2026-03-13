cmake_minimum_required(VERSION 3.12)
project(encos_driver LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Set a default build type (Release) if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

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
    C_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN 1
)

target_include_directories(encos_driver_lib
    PUBLIC  include 
    PRIVATE src deps/log.c/src/
)

# 2. Link the math library (-lm) to the library target
# 'm' is the standard way to refer to the math library in CMake
target_link_libraries(encos_driver_lib PRIVATE m)

# 3. Add executable and link
add_executable(encos_driver src/main.c)
target_link_libraries(encos_driver PRIVATE encos_driver_lib m)
