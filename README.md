# hydraulic-robot-arm-pico

Programs for the Raspberry Pico to control hydraulic actuators via brushless gear pump. The pico receives its instructions from the Olimex via a serial connection.

## pico-position

Position based controller with two operating modes depending on the message received.

### Speed mode
```
SPD <id_motor> <value>\n
```
  * <id_motor> : 0 shoulder, 1 elbow, 2 wrist
  * <value> : 0 full reverse, 2048 neutral, 4095 full forward
The PWM sent to the ESCs are proportional to <value>.

### Angle mode
```
ANG <id_motor> <value>\n
```
  * <id_motor> : 0 shoulder, 1 elbow, 2 wrist
  * <value> : desired angle, [-15,45] shoudler, [-90,0] elbow, [-97, 110] wrist
A PID controller is used to achieve desired angles.



## pico-vector (WIP)

Velocity based controller with two operating modes depending on the message received.

### Speed mode
```
SPD <val1> <val2> <val3>\n
```
  * <val1> : shoulder, 0 full reverse, 2048 neutral, 4095 full forward
  * <val2> : elbow, 0 full reverse, 2048 neutral, 4095 full forward
  * <val3> : wrist, 0 full reverse, 2048 neutral, 4095 full forward
The PWM sent to the ESCs are proportional to <vali>.

### Vector mode
```
VEC <val1> <val2> <val3>\n
```
  * <val1> : x component of desired tip velocity vector
  * <val2> : y component of desired tip velocity vector
  * <val3> : desired wrist angle [-97, 110]
The control loop is not yet implemented.



## Instructions

To build one of the program execute ```build.sh```, it will generates a .uf2 file that needs to be paste to the Pico (by connecting it with a USB cable while pressing the bootloader button).
Make sure the Pico SDK has been cloned too and is well referenced in ```CmakeLists.txt```.
