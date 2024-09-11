export PICO_SDK_PATH=${PWD}/../pico-sdk
export FREERTOS_KERNEL_PATH=${PWD}/../FreeRTOS-Kernel
cmake -G "MinGW Makefiles" -DPICO_BOARD=pico_w -S . -B build
(cd build && make)