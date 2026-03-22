cmake_minimum_required(VERSION 3.12)
project(encos_driver LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# --- Keep your existing Library and Executable logic ---
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(LIB_SOURCES
    src/driver.c 
    src/constant.c
    src/encos_command.c 
    src/socket_can_driver.c
    deps/log.c/src/log.c
)

add_library(encos_driver_lib SHARED ${LIB_SOURCES})
set_target_properties(encos_driver_lib PROPERTIES 
    C_VISIBILITY_PRESET default # Changed to default so pybind can see symbols
)
target_include_directories(encos_driver_lib PUBLIC include PRIVATE src deps/log.c/src/)
target_link_libraries(encos_driver_lib PRIVATE m)

# --- New Pybind11 Logic ---

# Find pybind11 - this requires pybind11 to be installed on your system
# find_package(pybind11 REQUIRED)

# pybind11_add_module(encos_driver src/driver_pybind.cpp)

# Link the python module to your driver library
# target_link_libraries(encos_driver PRIVATE encos_driver_lib)

# Ensure the python module can find your headers
# target_include_directories(encos_driver PRIVATE include src)

add_executable(encos_driver src/main.c)
add_executable(motor_test src/motor_test.c)

target_link_libraries(encos_driver PRIVATE encos_driver_lib m)
target_link_libraries(motor_test PRIVATE encos_driver_lib m)
target_link_libraries(main PRIVATE encos_driver_lib m)
