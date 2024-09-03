cmake_minimum_required(VERSION 3.13)

# Pull in SDK (must be before project)
include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

# Pull in FreeRTOS
include($ENV{FREERTOS_KERNEL_PATH}/portable/ThirdParty/GCC/RP2040/FreeRTOS_Kernel_import.cmake)

project(main C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

add_executable( main 
    main.c
    controller.c
    delivery.c
    detection.c
    diagnostics.c
    extraction.c
    blink.c
    localisation.c
    motors.c
    pathplanning.c
    wifi.c
    rgb.c
    dhcpserver/dhcpserver.c
    dnsserver/dnsserver.c
    ../FreeRTOS-Kernel/portable/MemMang/heap_4.c
    )

target_include_directories(
    main PRIVATE 
    ${CMAKE_CURRENT_LIST_DIR}
    ${CMAKE_CURRENT_LIST_DIR}/..
    ${CMAKE_CURRENT_LIST_DIR}/dhcpserver
    ${CMAKE_CURRENT_LIST_DIR}/dnsserver
)
 
#for Pico W
if (PICO_CYW43_SUPPORTED)
target_link_libraries(
    main
    pico_stdlib 
    hardware_pwm
    FreeRTOS-Kernel 
    FreeRTOS-Kernel-Heap4
    # pico_cyw43_arch_lwip_poll
    pico_cyw43_arch_lwip_threadsafe_background
    # pico_cyw43_arch_lwip_sys_freertos
    )
endif()



# create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(main)

# enable usb output, disable uart output
pico_enable_stdio_usb(main 1)
pico_enable_stdio_uart(main 1)

