# National Taiwan University Racing Team Software Development Log
###### tags: `development_log` `NTURT`
##### Group: electrical system group
##### Person in charge: 羅紀翔
##### Authors: 羅紀翔 高仕懷
##### Subsystem: RP1
##### Subsystem number: RP6
##### Software name: state_controller
##### Repository: [github](https://github.com/NTURacingTeam/nturt_state_controller.git)
##### Started designing date: 2022/4/21
##### Current version: 1.0
##### Last modified date: 2022/9/20

---

## Engineering goal:

Implement ready to drive (rtd) feature required in fsae rule, and shutdown/reboot control.

## Program structure:

### wiringPi

Since `wiringPi` cutrrentlly only exist on rpi, it is implemented to use preprocessor the check if `__arm__` is defined to avoid running into compilation error.

### ROS timer

The shutdown/reboot control is implemented by ros timmer to call a callback function after a period of time the button is not pressed.

## Included libraries:

- wiringPi

## Testing environment:

- ros noetic

##### Testing hardware:

- asus tuf gaming a15 FA506II-0031A 4800H
- raspberry pi 3B+

##### Operating system:

- ubuntu 20.04
- raspbian 32-bit
- docker virtual environment from [NTURacingTeam/docker](https://github.com/NTURacingTeam/docker) with image `ros_matlab`, `ros_rpi` based on ubuntu20.04

##### Compiler(intepreter) version:

- gcc 9.4.0 (Ubuntu 9.4.0-1ubuntu1~20.04.1)

---

## Testing result of 1.0:

### Ready to drive (rtd)

Worked successfully to check the can signal from dash board and front box, brake actuated and only play rtd sound once after multiple button pressed.

### Shutdown/ reboot control

Worked successfully to shutdown/reboot after corresponding presses.

## Todos in 1.0:

