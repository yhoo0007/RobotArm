# ESP32 Robot Arm
## Overview
A 4-dof robot arm controlled by 2 ESP32 microcontrollers created with the intention of being used as part of a bottling line. The arm uses 4 closed-loop stepper motors (one for each dof) attached to 4 corresponding hardware timers among the 2 microcontrollers. One of the microcontroller acts as the 'main' controller (receives commands from WiFi, calculates speed and angles of motors, records position checkpoints, etc), while the other is simply responsible for controlling the motor at the end of the arm (4th dof). The end effector of the arm is a suction cup for picking and placing bottles by their caps.

## The Arm
The construction of the arm is based on the 4-dof 'articulated manipulator' with the 4th degree of freedom being a 1-dimensional rotation of the end effector. This rotation is used to ensure that the suction cup is always facing perpendicularly downwards no matter the position of the arm. The following diagram illustrates the design of the arm.

![alt text](https://github.com/yhoo0007/RobotArm/blob/master/imgs/Design.png "Arm Design")

The two gearboxes for the lower and upper arm are secured to machined aluminium pieces which form the arm structure of the arm. The motors are then coupled to the gearboxes. The two gearboxes have worm gears which provide a 50:1 reduction from the stepper motor. Another feature of the worm gears is that the arm is able to hold its position when powered off, which would not be possible with normal gears. A timing belt is used for the base motor to provide a 100:14 reduction. The stepper motors drivers are also set to 1600 pulse/rev which further increases the positioning resolution of the motor.

## The Control
As mentioned, the robot is controlled by a pair of ESP32 microcontrollers. These controllers are compatible with the Arduino IDE but also feature built in WiFi capabilities. This allows for omittance of a mechanical user interface or terminal on the machine itself. Instead, control of the robot arm is done from the Blynk mobile app. Below is a screenshot of the user interface.

![alt text](https://github.com/yhoo0007/RobotArm/blob/master/imgs/Blynk_UI.jpg "Blynk UI")

The app allows positioning of the robot arm effector based on coordinates (x, y, z) in millimeters. It also allows for delays and valve on/off switches. These commands can be saved as 'checkpoints' in the robot arm which can then be replayed. In production, the checkpoints would be programmed from this interface and the movement cycle would be triggered by a signal from the main line controller. Arm movements can also be given a time parameter in which to complete the move in but this feature has not been made available on the Blynk interface.

Each ESP32 microcontroller has 4 hardware timers. In this program, one of the timers is used as a 'motor controller' which periodically updates the speed of the other motors. This is so that movement of the robot arm follows a smooth acceleration/deceleration profile instead of stark movements. The other timers are used to generate pulses for the motors with each motor having its own timer. An attempt was made to run all the motors off one timer but was ultimately unsuccessful. Having a hardware timer dedicated to each motor ensures accurate frequency generation (for accurate timing of movements), and more importantly pulse-perfect movements as errors in the number of pulses sent can result in accumulative positioning errors.

As there are 4 motors on the robot arm, there are not enough timers on a single ESP32 to control the entire arm. Hence the second ESP32 is used to control the movement of the end-effector. Inverse kinematics calculations for translating coordinates to motor angles are done on the main ESP32 and sent over UART to the second ESP32 such that the movement of all motors are coordinated correctly.

## Additional Notes
The two microcontrollers are socketed onto a motherboard which provides electrical isolation between the microcontrollers and the motor drivers via optocouplers. This is to protect the sensitive, low voltage microcontroller circuits from the high voltage, high current motor driving circuits.

The arm has not been tested for its maximum mechanical capabilities but its performance has been satisfactory for its intended use case with decent speed and little to no positioning errors (coordinate accuracy of ~1mm).

There is currently no 'easy' method for calibrating the position of the robot arm besides manually moving to a position of known coordinates and pressing the 'calibrate' button on Blynk. Possible solutions to this would be the incorporation an absolute encoder or some fancy distance/light sensors.

Currently there is also no proper ability for the arm to perform more advanced movements such as defined arcs, lines, or other complex paths. The current motor controlling code (and my brain) will have to undergo major refactoring in order to implement these features.
