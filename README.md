# NTURT state controller

## Introduction

Implement the ready to drive (rtd) feature as required by the fsae rule. The same rtd button is also used for controlling the shutdown and reboot of the vehicle controle unit (vcu).

## Usage

The node may be run by

```bash=
rosrun nturt_state_controller nturt_state_controller_node
```

This node receive can message from `/received_messages` topic and send can message to `/sent_messages` that work with node `socketcan_bridge_node` from `socketcan_bridge` package.

It also send ros bool message to `/node_state` to disable/enable other nodes.

### Ready to drive (rtd)

Specially required by fsae rule, is will check the state of the tractive and electrical system before lighting the rtd button. Once the button is pressed when rtd light is lit on, a rtd sound will be played, and the car can thereafter be driven (positive torque output).

### Shutdown/reboot button

The ready to drive (rtd) button can be used to control the shutdown/reboot of the vehicle control unit (vcu). Press the button 3 ~ 4 times in a short will cause the vcu to reboot, 5 times orabove will cause it to shutdown.

## Known issue

### Compiling error due to `wiringPi` (only for arm computer that`s not rpi) (fixed, maybe)

Since `wiringPi` is a preinstalled package only for raspberry pi and no other computer can install it properly. Yet it's still required (in c++) in order to control gpios of rpi.

To avoid running compiling error due to not finding `wiringPi`, the work around is

1. Check if `__arm__` macro is defined it the c++ source files. If not, don't use `wiringPi`.
2. Check `if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")` in `CMakeLists.txt`. If false, don't link the binary to `wiringPi`.

Hence, computers that's not arm structure will be fine. Yet computers that are arm structure and not rpi will still compile error. There's probably no solution to such problem, hence this error may not be patched.
