cmake_minimum_required(VERSION 3.10)
project(gimbal LANGUAGES C)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

set(GIMBAL_CORE_SOURCES
    src/gimbal.c
)

set(GIMBAL_DRIVER_SOURCES "")
set(_default_drivers
    drv_udp_tz0xxx
    drv_udp_c12
)

if(DEFINED SROBOTIS_CONTROL_GIMBAL_ENABLED_DRIVERS)
    set(_drivers "${SROBOTIS_CONTROL_GIMBAL_ENABLED_DRIVERS}")
else()
    set(_drivers "${_default_drivers}")
endif()

foreach(_drv IN LISTS _drivers)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/drivers/${_drv}.c")
        list(APPEND GIMBAL_DRIVER_SOURCES "src/drivers/${_drv}.c")
        message(STATUS "[gimbal] Including driver: ${_drv}")
    else()
        message(FATAL_ERROR "[gimbal] Driver '${_drv}' not found at src/drivers/${_drv}.c")
    endif()
endforeach()

add_library(gimbal SHARED
    ${GIMBAL_CORE_SOURCES}
    ${GIMBAL_DRIVER_SOURCES}
)

target_link_libraries(gimbal PRIVATE Threads::Threads m)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(gimbal PRIVATE -Wall -Wextra)
endif()

install(TARGETS gimbal
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
)

install(FILES include/gimbal.h
    DESTINATION include
)

option(BUILD_TESTS "Build gimbal demo programs" ON)

if(BUILD_TESTS)
    add_executable(test_gimbal_udp test/test_gimbal_udp.c)
    target_link_libraries(test_gimbal_udp PRIVATE gimbal Threads::Threads m)
    set_target_properties(test_gimbal_udp PROPERTIES BUILD_RPATH "$ORIGIN")

    install(TARGETS test_gimbal_udp
        RUNTIME DESTINATION bin
    )
endif()
