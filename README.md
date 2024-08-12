Build chain based off https://github.com/aws-iot-builder-tools/freertos-pi-pico/tree/main.

## Dependencies

For windows:
- Git
- CMake (https://cmake.org/download/)
- ARM GNU Toolchain (https://developer.arm.com/Tools%20and%20Software/GNU%20Toolchain)
- Make and MinGW
  - Easiest way is with Chocolatey. Install it (https://chocolatey.org/) and then run `choco install make; choco install mingw`

For Linux:
- CMake (`sudo apt install cmake`)
- GCC Toolchain (`sudo apt install gcc-arm-none-eabi`)
- Build Essentials (`sudo apt install build-essential`)

## Running

- `git clone <dir>` to clone the directory
- `git submodule update --init --recursive --remote`

In a linux shell (either native linux or git bash), run:
- `cd src; ./make.sh` to build the `main.c` file
- `cp build/app.uf2 /d/` to flash, assuming you have connected your Pico in bootsel mode and it registers as drive D.

If this doesn't work, try explicitly checking out the branch:
- `cd pico-sdk`
- `git checkout master`

## Latest Verified Submodule Versions
Current as of 12/08/24:
- FreeRTOS-Kernel: 81c62321
- pico-sdk: 8cc3dbb3
- tinyusb: 42326428
- cyw43-driver: faf36381
- lwip: STABLE-2_2_0_RELEASE
- mbedtls: mbedtls-2.28.8
- btstack: v1.6.1