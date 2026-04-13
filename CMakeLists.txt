cmake_minimum_required(VERSION 3.12)
project(encos_driver LANGUAGES C CXX)

set(CMAKE_C_STANDARD 17)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
option(BUILD_PYTHON_BINDINGS "Build Python bindings with pybind11" OFF)

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

if(BUILD_PYTHON_BINDINGS)
    find_package(Python3 COMPONENTS Interpreter Development.Module QUIET)
    if(NOT Python3_Interpreter_FOUND)
        find_package(Python3 COMPONENTS Interpreter REQUIRED)
    endif()

    if(TARGET Python3::Module)
        set(PYBIND11_FINDPYTHON ON)
    else()
        set(PYBIND11_FINDPYTHON OFF)
        set(PYTHON_EXECUTABLE ${Python3_EXECUTABLE})
    endif()

    find_package(pybind11 CONFIG QUIET)

    if(NOT pybind11_FOUND)
        execute_process(
            COMMAND ${Python3_EXECUTABLE} -m pybind11 --cmakedir
            RESULT_VARIABLE pybind11_cmakedir_result
            OUTPUT_VARIABLE pybind11_cmakedir_output
            ERROR_VARIABLE pybind11_cmakedir_error
            OUTPUT_STRIP_TRAILING_WHITESPACE
        )

        if(pybind11_cmakedir_result EQUAL 0 AND EXISTS "${pybind11_cmakedir_output}")
            set(pybind11_DIR "${pybind11_cmakedir_output}")
            find_package(pybind11 CONFIG REQUIRED)
        else()
            message(FATAL_ERROR
                "pybind11 not found for Python interpreter: ${Python3_EXECUTABLE}\n"
                "Install pybind11 into that interpreter, or pass -Dpybind11_DIR=<path-to-pybind11Config.cmake>.\n"
                "Interpreter error: ${pybind11_cmakedir_error}"
            )
        endif()
    endif()

    pybind11_add_module(encos_python src/driver_pybind.cpp)
    target_link_libraries(encos_python PRIVATE encos_driver_lib)
    target_include_directories(encos_python PRIVATE include src)
endif()

add_executable(encos_driver src/main.c)
# add_executable(motor_test src/motor_test.c)
add_executable(main src/main.c)

target_link_libraries(encos_driver PRIVATE encos_driver_lib m)
# target_link_libraries(motor_test PRIVATE encos_driver_lib m)
target_link_libraries(main PRIVATE encos_driver_lib m)
