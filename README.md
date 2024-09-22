# METR4810 Team 17

## Navigating

The `main` branch of the git repo is used for all production-ready releases of both the firmware and the client-side software.

Use the `dev` branch for development relating to firmware, and the `aruco-localise` branch for development relating to the client side.

## Robot Usage

### Powering

The robot should be powered with either the battery pack connected on MH20 or via USB through the RPi Pico W, but **not both** (as this will fry the board). 

### Motor Connections

The motor connections are routed as MH21-MH24 on the top left of the PCB. Connections are as follows:

- MH21: *unknown*
- MH22: *unknown*
- MH23: *unknown*
- MH24: *unknown*

### Onboard LED

The robot has an onboard LED which flashes yellow every half second. Any behaviour deviating from this norm is indicative of a critical failure.

### Status LED

The robot has a second LED located below the RPi Pico. It has the following colour codes:

- GREEN: Successful bootup
- YELLOW: Watchdog has reset the Pico (will happen after flashing as well)

### Onboard Buttons

The robot has several onboard pushbuttons with the following functions:

- PB1: Flash a colour on the LED
- PB2: Flash a colour on the LED
- PB3: Flash a colour on the LED
- PB4: Set the LED Blue
- PB5: Resets the Pico
  
### Onboard Shunt Jumpers

The robot has a number of shunts to change the settings of the PCB:

- J1: enable PB4 and J2
- J2: nonfunctional
- H31-33: shunt the two upper jumpers to enable ADC and shunt the two lower jumpers to enable PB1-3
- J5: enable the RGB LED

### Connecting via WiFi

After powering the Pico, connect to the METR4810 Team 17 WiFi network with password `password`.

Navigate to `192.168.4.1` which is the default address (using port 80).

Then, navigate to one of these endpoints (i.e., `192.168.4.1/log`)

### /diagnostics

This displays diagnostic information about the tasks and how much RAM is remaining

### /log

Displays the debugging log of the system (used by making calls to `vDebugLog`)

### /control

Allows for controlling of the robot. Click the links to send a command

## Firmware Development Guide

The build chain used for firmware is based off https://github.com/aws-iot-builder-tools/freertos-pi-pico/tree/main. However, this build chain does not use the official RP2040 SDK because it is currently broken - it uses our own fork of it.

### Dependencies

You need the following dependencies to build and flash firmware onto the Pico.

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

### Cloning and Initialising

- `git clone <dir>` to clone the directory
- `git submodule update --init --recursive --remote`

If this doesn't work, do:

- `cd pico-sdk`
- `clone https://github.com/tranjamin/pico-sdk.git` (or the ssh version)
- `git submodule update --init --recursive --remote`

### Building and Flashing

In a linux shell (either native linux or git bash), run:
- `cd src; ./make.sh` to build the `main.c` file
- `cp build/main.uf2 /d/` to flash, assuming you have connected your Pico in bootsel mode and it registers as drive D (hold down the bootsel button and connect the Pico).

### Development

For development, see the guide in `Development.md`.

## Client-Side Development Guide

### Installation

Create a python virtual environment with
```bash
python -m venv .venv
```
Activate it somehow, and hope powershell isn't really annoying. For windows, run:
```bash
Set-ExecutionPolicy Unrestricted -Scope Process
.venv/Scripts/Activate.ps1
```

Install the requirements using

```bash
pip install -r requirements.txt
```

### Running

To run the client-side application, run `python local_arucos_adaptive_origin.py`. Depending on what you want to do, you have the following options (eventually these will probably go into config files):

(You need to be on `add-pathplanning` branch to do this, but `aruco-localise` will have the latest localisation code)

- Choose which camera to use:
  - Change the first argument of `cv.VideoCapture()` at the start of `main`. You might have to try several different numbers (usually it's 0, 1, or 2).

- Choose what path to follow:
  - Change the path in `plan.set_waypoints` towards the start of `main`. You can either make it `SnakeWaypointSequence()` for the actual robot path, or `RectangleWaypointSequence()`/`MockLocalisationWaypointSequence()` for testing.

- Choose whether to use localisation or not:
  - To test in isolation of the localisation algorithm, set `localiser = MockLocalisation()` at the start of `main`. This will replace the algorithm with a stub which just returns the origin. To use localisation, use `localiser = Localisation()`.

- Choose whether to send commands to the robot:
  - To send the PWM commands to the robot, you must have connected to the `METR4810 Team 17` WiFi. Then, enable robot commands with `send_to_robot = True` near the start of `main`.

### Simulating

To run the simulator, run `python simulator.py`. As of right now, it only maps the plan of the robot on a visual grid.

- Change with path to follow:
  - Modify the argument of `sim.set_waypoints()` in the same way as how you would change the path when running normally.

### File Guide

I (Ben) am in the process of objectifying the code. As it stands, here is how it is laid out:

- `simulator.py`: The main file to hold simulation-specific code. 
  - It holds the `Simulator` class, which has a bunch of static members which can be modified to configure the settings of the simulator. Eventually will be put into a config file as well.
- `planning.py`: The main file to hold all data specific to pathplanning. It has the following classes:
  - `Waypoint`: a general class that holds an arbitrary waypoint. A waypoint is a 2D coordinate within the bounds of the environment, which optionally has a target heading and velocity upon reaching.
  - `DepositWaypoint`: a class representing the specific waypoint to go to when delivering bean.
  - `DepositHelperWaypoint`: a class representing a waypoint just under the deposit waypoint, used for lining up the robot.
  - `WaypointSequence`: an abstract class representing a sequence of waypoints. Shouldn't be called itself, and instead a subclass should be created which represents the path being followed. In the subclass' constructor, `super().__init__()` should be called first and then `self.waypoints` should be modified to set the waypoints. There is already a bunch of subclasses already made, with the primary one being `SnakeWaypointSequence`. The abstract class has the following useful functions:
    - `plan_to_deposit()`: adjusts the path to move immediately to the deposit chamber.
    - `plan_to_emergency()`: adjusts the path to move to the high ground. This will probably be moved in the future to its own class to allow for multiple strategies.
    - `get_current_waypoint()`: gets the waypoint the robot is currently tracking to.
    - `move_to_next_waypoint()`: changes the tracking to the next waypoint in the sequence.
  - `RobotGeometry`: a class holding information about the real-life robot's geometry and physical bounds. Should not be ever instantiated, and only used to perform calculations with the path planning.
  - `Pathplanner`: the main class of this file. Stores everything to do with the robot's path and how it follows waypoints. In particular has the following functions:
    - `set_waypoints()`: sets the waypoint set to follow
    - `set_controller()`: sets the controller which plans between waypoints
    - `update_robot_position()`: updates the path planner with the current robot position
    - `controller_step()`: computes a step by the controller to cause plan adjustments.
- `robot.py`: The file which takes care of controllers and the robot communication. The controller stuff should probably get moved in the future.
  - `Controller`: a class which takes care of planning between waypoints (i.e. to an arbitrary destination along a straight line path). Does weird stuff.
  - `Robot`: a class which takes care of communication to the physical robot. The important function is `send_control_action`, which will send the desired linear and angular velocity as PWMs to the robot.
- `estimators.py`: The file which encapsulates all to do with Kalman filters and pose estimation. I saw the `@` operator for matrix multiplication and noped out of there.
- `local_arucos_adaptive_origin.py`: The main file. Really should be renamed as `main`.
  - `MarkerCollection`: a class to hold a set of Aruco markers representing a single (rigid) object. Important functions are:
    - `register_marker()`: adds a marker to the collection
    - `estimate_pose()`: estimates the pose of the object as a translation and rotation vector from origin. I think.
  - `PiController`: I have no clue what this class is used for.
  - `Localisation`: The class which handles all localisation. Of note, contains:
    - `setup()`: configures the localisation problem in terms of an origin marker set and object marker set. Eventually will think of a good way to put this in a config file.
    - `get_position()`: gets the position of the object relative to the origin.
  - `MockLocalisation`: The mock class used to mimic `Localisation` but just return the origin repeatedly.
  - `main.py`: the monolithic function where everything happens.