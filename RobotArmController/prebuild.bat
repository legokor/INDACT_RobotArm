@echo off

REM Check if the required parameters are provided
if "%1"=="" (
    echo.
    echo Missing argument: ABSOLUTE_PATH_TO_PROJECT
    echo Usage: prebuild.bat ABSOLUTE_PATH_TO_PROJECT
    echo Example as STM32CubeIDE prebuild step: ${workspace_loc:/${ProjName}}\prebuild.bat ${workspace_loc:/${ProjName}}
    exit /b 1
)

set ABSOLUTE_PATH_TO_PROJECT=%1

echo.
echo [INFO] Pulling Docker image...
docker pull microros/micro_ros_static_library_builder:humble

if errorlevel 1 (
    echo.
    echo [ERROR] Failed to pull the Docker image.
    exit /b 1
)

echo.
echo [INFO] Running Docker container...
docker run --rm -v %ABSOLUTE_PATH_TO_PROJECT%:/project --env MICROROS_LIBRARY_FOLDER=micro_ros_stm32cubemx_utils/microros_static_library_ide microros/micro_ros_static_library_builder:humble

if errorlevel 1 (
    echo.
    echo [ERROR] Failed to run the Docker container.
    exit /b 1
)

echo.
echo [INFO] micro-ros build completed successfully.
