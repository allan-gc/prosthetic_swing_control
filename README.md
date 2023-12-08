# Torque Controlled Prosthetic Elbow

## Overview 

This repository contains a full Zephyr application used with a Teensy 4.1 for controlling a motorized prosthetic elbow. The system implements a step detection algorithm and a PI torque control system to output a corresponding arm swing. 

## Teensy 4.1 

The system uses a Teensy 4.1 with the Zephyr RTOS to control the torque output of the motor. All of the source code uses the Zephyr library for interfacing with the Teensy peripherals. The source code and corresponding configuration files must be added to your Zephyr project directory in order to build and flash the code to the Teensy. 

### To Build 
Add the source and configuration files to a directory within your zephyr project: ~/zephyrproject/zephyr/your_directory

Then run the following command from within the ~/zephyrproject/zephyr directory :

`west build -p auto -b teensy41 path_to_your_directory`

### To Flash
Flashing with the west build system does not seem to work with the Teensy architectures. To flash, I have been using this [Teensy command line tool](https://github.com/PaulStoffregen/teensy_loader_cli). More about this issue can be found [here](https://github.com/zephyrproject-rtos/zephyr/issues/30204).

To flash, press the reset button on the Teensy 4.1. The run the following command line:  
`./teensy_loader_cli --mcu=TEENSY41 -w ./path_to_zephyr_hex_file`. 

For example, if running from the ~/zephyrproject/zephyr directory, the command would be:   
`./teensy_loader_cli --mcu=TEENSY41 -w ./build/zephyr/zephyr.hex`
