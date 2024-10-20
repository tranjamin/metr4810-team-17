cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Pull in FreeRTOS
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

# define projects and C standards
project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

# add all linked executables
add_executable( main 
    main.c
    delivery.c
    diagnostics.c
    extraction.c
    blink.c
    motors.c
    wifi.c
    rgb.c
    digitalio.c
    watchdog.c

    dhcpserver/dhcpserver.c
    )

# add all directories
target_include_directories(
    main PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/dhcpserver
)

# link all libraries
target_link_libraries(
    main
    pico_stdlib
    pico_time
    hardware_pwm
    hardware_watchdog
    FreeRTOS-Kernel 
    FreeRTOS-Kernel-Heap4
    pico_cyw43_arch_lwip_threadsafe_background
    )

# set outputs of pico
pico_add_extra_outputs(main)

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)

