# Rover Mini-Project: Lidar Bot
Group project for ECE/CS6780 Embedded SYS. Focusing on 2D lidar integration with STM32.
#### Contributors ####
Aidan Rhind
Emma Wadsworth
Michael Macias
## Introduction ##
This rover has been built to complete the task of navigating a simple maze without running into any of the obstacles. The rover can be operated remotely, however the operator cannot see the maze, and only has access to data from the rovers sensors transmitted over a 9600 baud serial connection.\
The rover design is equipped with a 360 degree LiDAR sensor, two motors with associated drivers, and a bluetooth serial tranciever. The essential idea is that the operator will recieve LiDAR data, and return control signals which the rover will follow as it navigates the maze. The rover is able to transmit LiDAR data to the user, recieve signals from the user, and complete forward and backward motion as well as pivots left and right.\
!!! insert basic schematic here !!!
## Hardware Setup ##
Insert intro to how the rover is put together\
!!! insert wiring diagram here !!!
### Component Information ###
!!! insert final version of BoM with datasheets included !!!\
make notes on specs custom parts, namely chassis and motor drivers\
also briefly discuss design choices (ie larger STM board needed to process LiDAR data)\
!!! insert picture of rover here !!!
## Control ##
will be filled by EW pending code submission--I will summarize our approach to the dual PI loop as well as how things fit together. I cannot speak much to the LiDAR sensor, however beyond when and how it reports data to the user.\
!!! insert flowchart here !!!
