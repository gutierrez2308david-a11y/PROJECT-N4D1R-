# PROJECT-N4D1R-
Custom Teensy 4.1 flight computer &amp; dual axis TVC rocket build 
# Dual-Axis Thrust Vector Control (TVC) Rocket & Custom Flight Computer

## Overview
A self-directed aerospace engineering project designing and building a custom flight computer and dual-axis engine gimbal stabilization system.

## Hardware & Avionics Architecture
* **Microcontroller:** Teensy 4.1 (ARM Cortex-M7 @ 600 MHz)
* **IMU:** MPU 9265 (9-DOF Accelerometer/Gyroscope/Magnetometer) via SPI Protocol
* **Altimeter:** BMP280 Barometric Pressure Sensor via SPI Protocol
* **Actuation:** Dual 9g Micro Servos driving a 3D-printed gimbal assembly
* **CAD Software:** Autodesk Fusion 360
* **Firmware:** C++ (Closed-loop PID control algorithm)
* **Rocket Sim** Fins have a mass of 0.01 and Coefficient of drag of 0 (for simulattion purposes, actual rocket will not have fins)

## Video Demonstrations & Testing
* [📺 Watch Gimbal Actuation & Servo Test (YouTube)]((https://www.youtube.com/@DavidGutierrez-k6c/shorts))


## Key Technical Achievements
1. Hand-wired SPI data bus resolving multi-device bus conflicts between BMP280 and MPU 9265.
2. Programmed discrete PID loop in C++ to process sensor telemetry and calculate real-time servo gimbal angles.
3. Designed and FDM 3D-printed structural gimbal mounts in Fusion 360 to withstand motor burn stresses.
