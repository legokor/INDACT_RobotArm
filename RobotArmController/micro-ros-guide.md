# Micro-ROS Guide

## Installation

The GIT repository contains the [micro_ros_stm32cubemx_utils repository](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils.git) as a GIT submodule.
To download or update it run the command: `git submodule update --init --recursive`.

After the micro_ros_stm32cubemx_utils repository is downloaded, follow the instructions in its readme file and build the microros library using docker ("Using this package with STM32CubeIDE" section, 2. step).
Alternatively use the `prebuild.bat` script to do the same. **NOTE: The project can only be built successfuly if the microros library is already built.**
