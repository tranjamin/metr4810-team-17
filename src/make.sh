export PICO_SDK_PATH=${PWD}/../pico-sdk
export FREERTOS_KERNEL_PATH=${PWD}/../FreeRTOS-Kernel
cmake -G "MinGW Makefiles" -S . -B build
(cd build && make)