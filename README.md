# Rover Mini-Project: Lidar Bot
Group project for ECE/CS6780 Embedded SYS. Focusing on 2D lidar integration with STM32.
#### Contributors ####
Aidan Rhind
Emma Wadsworth
Michael Macias
## Introduction ##
This rover has been built to complete the task of navigating a simple maze without running into any of the obstacles. The rover can be operated remotely, however the operator cannot see the maze, and only has access to data from the rovers sensors transmitted over a 9600 baud serial connection.\
The rover design is equipped with a 360 degree LiDAR sensor, two motors with associated drivers, and a bluetooth serial tranciever. The essential idea is that the operator will recieve LiDAR data, and return control signals which the rover will follow as it navigates the maze. The rover is able to transmit LiDAR data to the user, recieve signals from the user, and complete forward and backward motion as well as pivots left and right.

![schem](https://user-images.githubusercontent.com/74335040/234171549-d08da05c-f69a-4462-8446-3461c402e0fd.jpg)
## Hardware Setup ##
![wiring](https://user-images.githubusercontent.com/74335040/234171776-6e808a7f-367d-43e4-af37-32d20265b4e3.jpg)
### Component Information ###
Component|Part|Information|
--- | --- | --- 
LiDAR | XV11 Lidar sensor | https://www.ev3dev.org/docs/tutorials/using-xv11-lidar/ | 
Motors | insert part here | insert link here |
USART | HC-05 - Bluetooth Module | https://components101.com/wireless/hc-05-bluetooth-module |
STM32 | STM32F4DISCO | insert link here |
Motor Drivers | Custom (in class) | [HW_full_solution.zip](https://github.com/aidanrhind/Lidar_Bot_6780/files/11316879/HW_full_solution.zip)|

As a note, the STM board was selected to be large enough to handle the amount of LiDAR data the sensor produced--this is the most notable design constraint.\
Another consideration was the power for the motors; a single 9V battery could not provide sufficient current to the motors, so two were used.\
!!! insert picture of rover here !!!\
The rover is powered with two 9V batteries and a usb charge block, and mounted to a small 3D printed chassis
## Control ##
The user recieves LiDAR data in a form with angle then distance (more detail pending). The user will then input direction controls, which are transmitted back to the rover. blah blah PID loop.
