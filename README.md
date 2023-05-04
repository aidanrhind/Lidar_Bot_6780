# Rover Mini-Project: Lidar Bot
Group project for ECE/CS6780 Embedded SYS. Focusing on 2D lidar integration with STM32.
#### Contributors ####
Aidan Rhind
Emma Wadsworth
Michael Macias
## Introduction ##
This rover has been built to complete the task of navigating a simple maze without running into any of the obstacles. The rover can be operated remotely, however the operator cannot see the maze, and only has access to data from the rovers sensors transmitted over a 9600 baud serial connection.\
The rover design is equipped with a 360 degree LiDAR sensor, two motors with associated drivers, and a bluetooth serial tranciever. The essential idea is that the operator will recieve LiDAR data, and return control signals which the rover will follow as it navigates the maze. The rover is able to transmit LiDAR data to the user, recieve signals from the user, and complete forward and backward motion as well as pivots left and right.
The data from the initial LiDAR design was not interperetable, as the LiDAR sensor transmitted error codes and extra information beyond the scope of the client side code, this frequently crashed the program. For testing purposes, the LiDAR sensor was replaced with an ultrasonic sensor mounted to a stepper motor which could provide distance measures in three directions--left, right, and center.

![schem](https://user-images.githubusercontent.com/74335040/234171549-d08da05c-f69a-4462-8446-3461c402e0fd.jpg)
## Hardware Setup ##
Note: This is original design--in the final version, the LiDAR block was switched out for an ultrasonic sensor.
![wiring](https://user-images.githubusercontent.com/74335040/234171776-6e808a7f-367d-43e4-af37-32d20265b4e3.jpg)
### Component Information ###
Component|Part|Information|
--- | --- | --- 
LiDAR (unused) | XV11 Lidar sensor | https://www.ev3dev.org/docs/tutorials/using-xv11-lidar/ | 
Ultrasonic Sensor | HC-SR04 | https://www.sparkfun.com/products/15569
Motors | Metal Gearmotor 37Dx70L | https://www.pololu.com/product/4753 |
USART | HC-05 - Bluetooth Module | https://components101.com/wireless/hc-05-bluetooth-module |
STM32 (2X) | STM32F4DISCO | @Aidan please insert link here |
Motor Drivers | Custom (in class) | [HW_full_solution.zip](https://github.com/aidanrhind/Lidar_Bot_6780/files/11316879/HW_full_solution.zip)|

As a note, the STM board was selected to be large enough to handle the amount of LiDAR data the sensor produced--this is the most notable design constraint. In the end, the LiDAR was switched out for the ultrasonic sensor, but the original plans are in this repository alongside the modified version.\
Another consideration was the power for the motors; a single 9V battery could not provide sufficient current to the motors, so two were used.\
The rover is powered with two 9V batteries and a usb charge block, and mounted to a small 3D printed chassis\
![rov](https://user-images.githubusercontent.com/74335040/236120934-72d851d8-591c-48d5-b2a4-18016f8c012b.jpg)\
## Control ##
In the final (modified) version of the rover, the operator recieved distance input from a single ultrasonic sensor. This sensor was mounted on an arduino stepper motor, acnd could be controlled to look left and right via the serial connection. From this data, the operator would be able to interperete the location of surrounding obstacles and make decisions regarding rover control. This was accomplished on one of the two STM32 boards, separate from the rest of the rover systems.\
At this point, the user would interact with the other microcontroller, inputting a direction instruction to it. The rover had two PID controlled motors which would be activated to turn the rover left or right, or drive it forward.
