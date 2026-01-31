GeoSense
GPS-Independent Localization Using Embedded Sensor Fusion

This repository contains the implementation of a sensor-fusion–based localization system that estimates geographic position without relying on GPS, using IMU data, magnetometer readings, Wi-Fi RSSI, and adaptive correction mechanisms.

The project demonstrates a lightweight, deployable alternative to GPS for indoor or GPS-denied environments using embedded hardware.

Project Overview

Traditional GPS-based localization fails indoors and in dense urban environments.
This project proposes a GPS-independent localization framework that:

Uses IMU sensors (MPU6050) for motion sensing

Uses magnetometer (QMC5883L) for heading estimation

Incorporates Wi-Fi RSSI as a signal-strength cue

Applies symbolic regression + adaptive LUT correction

Improves accuracy over time using online learning

Core Components

Sensor Fusion

Accelerometer, gyroscope, magnetometer

Complementary filtering for heading estimation

Exponential Moving Average (EMA) noise filtering

Localization Model

Symbolic regression–based latitude & longitude estimation

RSSI-normalized signal modeling

Heading-aware correction

Adaptive LUT (Lookup Table)

Binned by RSSI strength and heading

Learns correction offsets online

Reduces localization error iteratively

Evaluation

Comparison between predicted, corrected, and actual location

Error measured in meters

Implemented in analysis notebooks

Repository Structure
├── FINAL SOURCE CODE.py        # Embedded sensor fusion & localization
├── Comparison.ipynb           # Error analysis & comparison
├── *.ipynb                    # Experiments and validation
├── README.md

Hardware & Inputs

MPU6050 (Accelerometer + Gyroscope)

QMC5883L (Magnetometer)

Wi-Fi RSSI (signal strength)

Embedded Linux device (e.g., Raspberry Pi)

Key Highlights

Works in GPS-denied environments

No external maps required

Lightweight and real-time

Learns corrections online

Suitable for indoor navigation, IoT devices, and edge systems

Applications

Indoor localization

Smart IoT devices

Assistive navigation

Embedded robotics

Low-power positioning systems
