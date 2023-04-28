@echo off

title pre-Build Commands
echo This is the pre-build script!
docker pull microros/micro_ros_static_library_builder:humble
docker run --rm -v C:\Users\sanyi\Documents\dev\Repositories\INDACT_RobotArm\ROS_Test_Firmware:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:humble