Build chain based off https://github.com/aws-iot-builder-tools/freertos-pi-pico/tree/main.

## Dependencies

For Windows:
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
- `cp build/main.uf2 /d/` to flash, assuming you have connected your Pico in bootsel mode and it registers as drive D.

If this doesn't work, try explicitly intialising the sub-submodules:
- `cd pico-sdk`
- `git submodulate update --init --remote --recursive`

## Latest Verified Submodule Versions
Current as of 30/08/24:
- FreeRTOS-Kernel: V202110.00-SMP-15-g81c623212
- pico-sdk: 2.0.0-1-g0b30d32
- tinyusb: 0.15.0
- cyw43-driver: v1.0.3-4-g7ee0b7b
- lwip: STABLE-2_2_0_RELEASE
- mbedtls: v2.28.1-36-ga77287f8fa
- btstack: v1.6.1

TinyUSB Dependencies:

../pico-sdk/lib/tinyusb/hw/mcu/allwinner (heads/main)
../pico-sdk/lib/tinyusb/hw/mcu/bridgetek/ft9xx/ft90x-sdk (v2.6.0_release-7-g9106016)
../pico-sdk/lib/tinyusb/hw/mcu/broadcom (0837008)
../pico-sdk/lib/tinyusb/hw/mcu/broadcom/firmware.wiki (fc62b78)
../pico-sdk/lib/tinyusb/hw/mcu/gd/nuclei-sdk (0.3.1-59-g7eb7bfa9)
../pico-sdk/lib/tinyusb/hw/mcu/infineon/mtb-xmclib-cat3 (latest-v3.X)
../pico-sdk/lib/tinyusb/hw/mcu/microchip (heads/master)
../pico-sdk/lib/tinyusb/hw/mcu/mindmotion/mm32sdk (708a715)
../pico-sdk/lib/tinyusb/hw/mcu/nordic/nrfx (v2.1.0)
../pico-sdk/lib/tinyusb/hw/mcu/nuvoton (heads/master)
../pico-sdk/lib/tinyusb/hw/mcu/nxp/lpcopen (43c45c8)
../pico-sdk/lib/tinyusb/hw/mcu/nxp/mcux-sdk (MCUX_2.10.0-64-gae2ab01d9)
../pico-sdk/lib/tinyusb/hw/mcu/nxp/nxp_sdk (heads/main)
../pico-sdk/lib/tinyusb/hw/mcu/raspberry_pi/Pico-PIO-USB (0.4-41-g52805e6)
../pico-sdk/lib/tinyusb/hw/mcu/renesas/rx (heads/master)
../pico-sdk/lib/tinyusb/hw/mcu/silabs/cmsis-dfp-efm32gg12b (f1c31b7)
../pico-sdk/lib/tinyusb/hw/mcu/sony/cxd56/spresense-exported-sdk (heads/master-2-g2ec2a15)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_f0 (v2.3.4-1-g2fc25ee)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_f1 (v4.3.2)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_f2 (v2.2.4)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_f3 (v2.3.4-1-g5e4ee5e)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_f4 (v2.6.4-1-g2615e86)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_f7 (v1.2.5-1-gfc676ef)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_g0 (v1.3.0-1-g08258b2)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_g4 (v1.2.0)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_h7 (v1.9.0)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_l0 (v1.9.0-1-g06748ca)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_l1 (v2.3.1)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_l4 (v1.7.0)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_l5 (v1.0.3)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_u5 (v1.1.0)
../pico-sdk/lib/tinyusb/hw/mcu/st/cmsis_device_wb (v1.10.0)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32f0xx_hal_driver (v1.7.4-1-g0e95cd8)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32f1xx_hal_driver (v1.1.6)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32f2xx_hal_driver (v1.2.5)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32f3xx_hal_driver (v1.5.4)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32f4xx_hal_driver (v1.7.9)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32f7xx_hal_driver (v1.2.8-1-gf7ffdf6)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32g0xx_hal_driver (v1.3.0-1-g5b53e6c)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32g4xx_hal_driver (v1.2.0)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32h7xx_hal_driver (v1.9.0)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32l0xx_hal_driver (v1.10.3)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32l1xx_hal_driver (v1.4.2)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32l4xx_hal_driver (v1.12.0)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32l5xx_hal_driver (v1.0.3)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32u5xx_hal_driver (v1.0.2-1-g2e1d4cd)
../pico-sdk/lib/tinyusb/hw/mcu/st/stm32wbxx_hal_driver (v1.10.0)
../pico-sdk/lib/tinyusb/hw/mcu/ti (heads/master)
../pico-sdk/lib/tinyusb/hw/mcu/wch/ch32v307 (17761f5)
../pico-sdk/lib/tinyusb/lib/CMSIS_5 (5.0.0-Beta4-2219-g202852626)
../pico-sdk/lib/tinyusb/lib/FreeRTOS-Kernel (V10.4.0-kernel-only-66-g2a604f4a2)
../pico-sdk/lib/tinyusb/lib/lwip (STABLE-2_1_2_RELEASE)
../pico-sdk/lib/tinyusb/lib/sct_neopixel (e73e04c)
../pico-sdk/lib/tinyusb/tools/uf2 (1961540)
../pico-sdk/lib/tinyusb/tools/uf2/hidapi (hidapi-0.7.0-112-ga6a622f)