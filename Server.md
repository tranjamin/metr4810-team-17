## How to use the WiFi Web Server

### Basic

After powering the Pico, connect to the METR4810 Team 17 WiFi network with password `password`.

Navigate to `192.168.4.1` which is the default address (using port 80).

Then, navigate to one of these endpoints (i.e., `192.168.4.1/ledtest`)

### /ledtest

This is the default endpoint which allows you to toggle in onboard LED

### /diagnostics

This displays diagnostic information about the tasks and how much RAM is remaining

### /log

Displays the debugging log of the system (used by making calls to `vDebugLog`)

### /control

Allows for controlling of the robot. Click the links to send a command
  