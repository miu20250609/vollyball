cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME standard_robot_c)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}


)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
        BSP/algorithm/pid.c
        BSP/algorithm/crc.c
        BSP/algorithm/rls.c
        BSP/algorithm/user_lib.c
        BSP/algorithm/pid_enhanced.c
        BSP/usart/bsp_uart.c
        BSP/can/bsp_can.c
        BSP/dwt/bsp_dwt.c
        BSP/pwm/bsp_pwm.c
        Module/motor/dji/m3508/m3508.c
        Module/motor/DJI/GM6020/gm6020.c
        Module/motor/DJI/M2006/m2006.c
        Module/motor/LK/MF9025/mf9025.c
        Module/motor/RS/rs02.c
        Module/motor/VESC/vesc.c
        Module/DBUS/dbus.c
        Module/NX/nx.c
        Module/NUC/nuc.c
        Module/Odom/odom.c
        Module/VOFA/vofa.c
        Module/IMU/BMI088/BMI088Middleware.c
        Module/IMU/BMI088/BMI088driver.c
        Module/IMU/EKF/kalman_filter.c
        Module/IMU/EKF/QuaternionEKF.c
        Module/IMU/INS/ins.c
        Module/LED/led.c
        Module/HC-SR04/hc.c
        APP/task_init.c
        APP/robovolley.c
        APP/daemon.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined include paths
        BSP
        Module
        APP
        BSP/algorithm
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
        ARM_MATH_CM4;ARM_MATH_MATRIX_CHECK;ARM_MATH_ROUNDING
        MADE_BY_CMAKE

        CHASSIS
        #GIMBAL
        OMNI
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
