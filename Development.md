## Source Directory Listing:

Miscellaneous

- `CMakeLists.txt`: The CMake file used to build the executable
- `build/`: The output directory (ignored). Of note is `build/main.uf2`, which is the executable to be loaded onto the Pico
- `make.sh`: The shell script used to call cmake, with some environment variables set. 
- `FreeRTOSConfig.h`: The config file for FreeRTOS. Used to enable/disable various features, set memory modes, etc.

WiFi-specific files:
- `dhcpserver`: source code files for setting up the DHCP server
- `dnsserver`: source code files for setting us the DNS server
- `lwipopts`: config file for wifi (lwip == Lightweight TCP/IP)

Source code:
- `main.c`: the main file which is built as the target for the make process
- `xyz.c`/`xyz.h`: files for each individual task

## FreeRTOS Crash Course

FreeRTOS is a scheduler which manages the runtime of multiple tasks. This allows multiple tasks to be run effectively in parallel. The scheduler runs one task for a short time, and then switches it out (context switch) for another task. In this application, we will be using SMP (symmetric multiprocessing) with the two cores of the Pico, which lets us do tasks truly in parallel. We use the coremask of a task to assign it to a particular core.

### Scheduling

FreeRTOS has been configured to be priority pre-emptive. This means that higher priority tasks (larger priority number) will "interrupt" lower priority tasks and it will get preference when the scheduler is figuring out which task to run. If two tasks have the same priority, they will be round-robined (be run for the same amount of time). It's usually a good idea to set tasks by default to the same priority (usually +1 or +2) and then raise certain tasks which are more important. The baseline priority (+0) is the priority of the IDLE task, which does stuff like garbage collection and being boring. This isn't very important.

### Blocking

To prevent high priority tasks from hogging the runtime, most tasks have some form of delay. A task won't be scheduled if it is being "blocked". Usually we either block until we receive a certain signal or, if our task runs continuously, add a small amount of delay in between iterations of our loop - `vTaskDelay(delay_ticks)`. This manually blocks our task for a certain number of ticks and lets other tasks execute.

### Task Communication

To communicate between tasks, usually we use semaphores and queues. Semaphores block a task until the function `xSemaphoreGive` is called, which can be done by another function. No examples of this in the code yet but there probably will be one day. Queues let us transfer data from tasks - one task populates the queue (`xQueueSend`) and the second task takes from the queue (`xQueueReceive`). Data is copied by value. Good examples of using queues are in `diagnostics.c`. If you are handling queues or semaphores within interrupt service routines, make sure to use the `fromISR` equivalent of the functions above.

### Critical Sections

For some tasks, we may want to run certain sections without interruption. Usually this is when accessing shared memory or writing to output data registers (for example, writing to UART/SPI/I2C). This is because otherwise another task may try to write simultaneously and we'll end up with race conditions and data corruption. You can wrap certain chunks of code in critical sections using `tskENTER_CRITICAL()` and `tskEXIT_CRITICAL()`, but do it with caution. Make sure you aren't using any FreeRTOS API functions inside (pico sdk API functions should be fine but this hasn't been tested). Definitely don't send any debug messages, and if the critical sections don't work it might be because some features (i.e., debug logs) use `fromISR` from outside ISRs, which can mess with critical sections.

### Includes

All task files need `task.h` and `FreeRTOS.h` included. For tasks which use queues and semaphores, their respective header files also needs to be included (`queue.h` and `semph.h`). For referencing any config settings, include `FreeRTOSConfig.h`. Many other FreeRTOS Functions require including their own header file.

### Memory Management

All tasks in FreeRTOS are allocated their own stack, which is used for function calls, task status registers and task variables. These stacks are stored in the FreeRTOS heap and loaded in and out depending on which task is running. The overall heap is split up into chunks for each of these stacks, and then general memory (for calling things like `malloc`). The heap and stack sizes are configurable, in `FreeRTOSConfig.h->configTOTAL_HEAP_SIZE` and `XYZ.h->XYZ_TASK_STACK_SIZE`. Both dynamic and static memory allocation have been enabled in the config file, although these can be changed. More info here (we are using Heap 4): https://www.freertos.org/Documentation/02-Kernel/02-Kernel-features/09-Memory-management/01-Memory-management.

### API Reference

Full API reference can be found here: https://www.freertos.org/Documentation/00-Overview. There are also slides in the OneDrive from CSSE3010 about FreeRTOS for refernce.

Some common functions which might be used are:

Util:
- `vTaskDelay`: manually blocking tasks
- `taskENTER_CRITICAL`/`taskEXIT_CRITICAL`/`FROM_ISR variants`: enter and leave critical sections
- `pvPortMalloc`/`pvPortFree`: equivalents of malloc and free.

Queues:
- `xQueueCreate`: create a queue
- `xQueueSendToBack`/`xQueueSendToBackFromISR`: add data to a queue
- `xQueueSendToFront`/`xQueueSendToFrontFromISR`: add data to a (stack) queue
- `xQueueOverwrite`/`xQueueOverwriteFromISR`: add data to a queue even if it's full. Meant to be used for queues of size 1.
- `xQueueReceive`/`xQueueReceiveFromISR`: get data from the front of the queue
- `xQueuePeek`/`xQueuePeekFromISR`: look at the front of the queue but don't remove
  - most of these functions have a timeout parameter you can set for how long to wait until there is a timeout

Semaphores:
- `xSemaphoreCreteBinary`: create a basic semaphore
- `xSemaphoreTake`: block until the semaphore is available (or until timeout) and then take the semaphore. Any subsequent calls to `xSemaphoreTake` will now be blocked until the semaphore is given back.
- `xSemaphoreGive`: give back the semaphore.

if you want to handle a complex relationship between tasks feel free to reach out, because there's probably an in built way to do it.

### Common Bugs

Put any other common bugs found during development here:

- Everything breaks and the pico has a high pitched scratching sound: this is a memory error, and either the total heap is too big or the task stack is too small. Disconnect the pico and reduce/increase these values (you can also see the memory usage in the diagnostics).
- Whenever I use a critical section everything breaks: make sure you are not calling any FreeRTOS API functions from within the critical section, or if you are make sure to use from `fromISR` equivalent. If that doesn't fix it, have a look here: https://www.freertos.org/FreeRTOS_Support_Forum_Archive/February_2017/freertos_FromISR_functions_outside_of_the_ISR_context_f36e532cj.html
- Memory/config errors during build: probably an error in the `CmakeLists.txt` file or the FreeRTOS config. Make sure `FreeRTOS-Kernel-Heap4` and `FreeRTOS-Kernel ` are being linked as well as `pico_cyw43_arch_lwip_threadsafe_background` (NOT the `_freertos` version).
    

## Development Guide

### Header Files

`xyz.h`:
- always make sure to include header guards to prevent duplicate imports

```c
#ifndef XYZ_H
#define XYZ_H

// do everything

#endif
```

- the following FreeRTOS options should always be defined for every task.
```c
#define XYZ_TASK_NAME // the name of the task
#define XYZ_TASK_PRIORITY // the priority of the task (written as tskIDLE_PRIORITY + some number)
#define BLINK_TASK_STACK_SIZE // the stack size of the task (written as configMINIMAL_STACK_SIZE * some number)
#define BLINK_TASK_COREMASK // which core the task can run on (0x01 for task 1, 0x02 for task 2, 0x03 for both)
```

- the following functions are externed in the h file:
- `vXyzInit()`: a function which initialises all variables/hardware/etc
- `vXyzTask()`: an infinite loop of what the task will execute

### CMake
CMake is kinda confusing, and currently the vscod extension isn't properly highlighting linked libraries for intellisense (I'll try to fix this). CMake is used to automatically generate make files. There are a few key parts of it:

- `include()` commands: pulls in other cmake files. We need to do this for both the FreeRTOS kernel and the Pico SDK.
- `project()` command: defines the project name (the "target") and outputs.
- `set()` command: sets cmake and environment variables.
- `add_executable()` command: registers the name of the target and then all the c files which need to be pulled in (not including libraries). If you create new files, you need to add them here.
- `target_include_directories()` commands: specifies which directories to search through for files and libraries. If you make subfolders, there's a chance you may need to register them here.
- `target_link_libraries()` commands: links any libraries to the target. For a lot of the Pico SDK functions, they are defined in separate libraries and so therefore may need to be added here.
- other functions: perform special operations. Don't worry about these (they are also commented).

### Pico SDK

#### GPIO

Basic GPIO functionality can be included with `pico/stdlib.h` (make sure this is included in the c files and linked in the cmakelists). Well technically, it uses the `hardware/gpio.h` library but this is included in `pico/stdlib.h`. Things you can do include (see `blink.c` for basic examples):
- `gpio_init(pin_number)`: initialise the pin
- `gpio_set_dir(pin_number, gpio_dir)`: set the pin direction (either `GPIO_IN` or `GPIO_OUT`)
- `gpio_put(pin_number, value)`: output a value to a pin
- `gpio_set_function(pin_number, gpio_function)`: set the function of the pin (i.e., IO/I2C/SPI/UART)
- `gpio_pull_up(pin_number)`/`gpio_pull_down(pin_number)`: pull up or down a pin

For the function table of each pin, see https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#group_hardware_gpio. This also has documentation on how to use pin interrupts.

#### USB UART

UART is currently enabled for diagnostics through the USB port. This can be enabled in `CMakeLists.txt` using `pico_enable_stdio_usb(main 1)` and `pico_enable_stdio_uart(main 1)`. Currently, any calls to `stdout`, `stdin` or `stderr` will be routed through UART (through the USB).

#### PWM

PWM can be used by linking the `hardware_pwm` library and including `hardware/pwm.h`. The Pico has 16 slices (individual instances) of pwm, with each slice having two channels (output compares). The slices are completely independent but the two channels of the same slice have a shared timer, so their frequency is the same. However, they can have different duty cycles. Slices are GP0-1, GP2-3, GP4-5, etc. up to GP14-15. GP16-17 then replicates GP0-1, GP18-19 replicates GP2-3 and so on, so it is recommended that this second half of pins is not used. 

Common PWM API calls are (see `motors.c` for code examples and recommended layout):
- `pwm_gpio_to_slice_num(pin_numer)`: identifies the slice a GPIO is on. This should usually be a macro.
- `pwm_gpio_to_channel(pin_number)`: identifies the channel a GPIO is on. This should usually be a macro.
- `gpio_set_function(pin_number, GPIO_FUNC_PWM)`: set a GPIO to use PWM (part of stdlib).
- `pwm_set_phase_correct(slice_number, bool)`: turn phase correction on or off. On is usually good for motor efficiency.
- `pwm_set_clkdiv(slice_number, divider)`: set the clock divider of the PWM.
- `pwm_set_wrap(slice_number, wrap)`: set the wrap/TOP/overflow value of the PWM. Together with the clock divider specifies the frequency of the signal.
- `pwm_set_enabled(slice)`: enable pwm on a slice.
- `pwm_set_chan_level(slice, channel, compare_value)`: set the duty cycle of a channel in ticks.

In general, you need to initialise the gpio pin, set its direction, set its alternate function to PWM, set the pwm clock divider, wrap and level, and then enable the pwm.

#### External (GPIO) Interrupts

External interrupts are useful for taking actions upon a given input (such as a switch or a pushbutton). The RP2040 has one external interrupt bus per core, and the Pico W uses an NVIC to make these interrupts available on every pin. 

Because there are only two distinct interrupt handlers, all external interrupts should be captured in the `digitalio.h` file.

First, any external interrupt pin needs to be configured as a GPIO. This means intialising it, setting its direction, pulling it up or down and enabling it. Then, to enable an interrupt the `gpio_set_irq_enabled()` function needs to be called (there are alternatives detailed in the API docs). This takes in three parameters: the pin number, the event mask, and true/false to enable/disable interrupts. The event mask signifies what type of interrupts to look for. The options are `GPIO_IRQ_EDGE_FALL`, `GPIO_IRQ_EDGE_RISE`, `GPIO_IRQ_LEVEL_LOW`, and `GPIO_IRQ_LEVEL_HIGH`. Bitwise OR can be used to combine multiple (i.e., `|`).

The IRQ callback then needs to be registered i.e., `gpio_set_irq_callback(callback_fn)`. This is already done. Within the callback, there is a switch statement to select the pin, and then conditionals to check if the relevant interrupt has been fired (`events & EVENT_TYPE`). Take inspiration from the existing material there. Also make sure to add debouncing where relevant - also see existing code for this.

For the API docs, see https://www.raspberrypi.com/documentation/pico-sdk/hardware.html#hardware_gpio.

#### Timers

There are two types of high-level timers used: alarms and repeated timers. These can be used to block a certain task for the required period of time, with an example shown in `delivery.c`. The library is already included in `stdlib`, but if you want you can explicitly include `hardware/time.h`.

For single events, the use of alarms is recommended. To set up an alarm, create an alarm pool to go with it. You need to call `alarm_pool_init_default()` and then `alarm_pool_create(pool_number, max_num_alarms)`. Store the result of the latter because you need to reference it later. In general, you want to allocate 1 more alarm than what you'll need at any given time.

To add an alarm, there are a range of different functions, but the best will probably be `alarm_pool_add_alarm_in_ms`. This takes in parameters of the alarm pool you created beforehand, the number of ms to wait, the function to call when the alarm has been fired, any parameters into this callback, and then `false` (usually).

If your alarm pool has finished, it is then advisable to destroy it using `alarm_pool_destroy`.

For the API docs, see https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#alarm.

#### LwIP

WiFi is enabled using the `pico_cyw43_arch_lwip_threadsafe_background` library. This is black magic to me so good luck trying to figure it out.

#### General

In general, most tasks calling the SDK will only need `pico_stdlib` (import it as `pico/stdlib.h`). This include GPIO, UART, stdio, and a few other things (https://www.raspberrypi.com/documentation/pico-sdk/high_level.html#pico_stdlib). For more high-level libraries, reference (https://www.raspberrypi.com/documentation/pico-sdk/high_level.html).

For certain peripherals and features, including I2C, PIO, DMA, PWM, IRQ, and ADC,  the low-level hardware API needs to be included. Make sure to link the library (usually in the form of `hardware_xxx`) and then include it (`hardware/xxx.h`). The reference can be found here: https://www.raspberrypi.com/documentation/pico-sdk/hardware.html. 

It's usually easier to just reference the examples repo at https://github.com/raspberrypi/pico-examples/tree/master to see how to implement certain features. Make sure you check their CMakeFile to see if any extra libraries need to be linked. If you figure out how to use a library, add it to the above section for reference.

#### Debugging

The UART is registered through the diagnostics task and outputs both through the USB and wirelessly to the `/log` endpoint. As long as you've included `diagnostics.h` in the file, you can make calls to this log using `vDebugLog` in the same way that you can use `printf` (it has fancy varargs).

The onboard LED can be accessed using `cyw43_arch_gpio_put` as seen in `blink.c`. However, try to avoid changing this, as you can identify code crashes by checking when this LED stops flashing.

Instead, another way to debug is using the RGB LED. If you import `rgb.h`, you can call `setRGB_XXX` for red, blue and green to set the colours of the RGB.

### General Development

1. Have good version control and documentation pls pls pls.
2. For consistency, please make sure you are using the correct slugs for the submodules. In future, I might clone these directly for better control.
3. FreeRTOS can be annoying to debug. Start out by developing your feature in isolation (comment out the `xTaskCreate` line for the other tasks). You can test it using either the debug logger (need to activate the diagnostics task) or by controlling the onboard LED. Look at the `blink.c` file for how to control this. Usually, I keep the Blink task activated (if it stops blinking it's a signifier that your code has crashed) and the WiFI task (becaus I can go to the diagnostics page to look at memory and task resource usage).
   1. To connect to the WiFi, the password is `password` and the address is `192.168.4.1`. Go to `/ledtest` for the default page, `/diagnostics` to see memory allocation, and `/log` to see the debug log line by line (you will have to reload to get to the next line). 
   2. Once it works in isolation, reactive the other tasks to make sure it integrates (I might be able to help with this).
4. Account for the fact that other tasks may change. For example, always check to make sure a queue or semaphore is initialised (`!= NULL`).
5. Don't call `cyw43_arch_init()` more than once because there's a bug (at the moment it is initialised in the blink task).
6. Always free statically allocated memory after you're done with it.
7. Make the stack size of a task as small as possible.
8. Always account for edge cases because our code crashing is the worst thing possible (also a watchdog for this might be useful).
9. For some edge cases (especially to do with configuring sizes of things), it may be easier to detect these errors and warn the user compile-time. have a look at line 41-49 of `wifi.c`.
10. If UART isn't working idk how to fix this.
