Computer Vision Based Robotic Hand Mimicry

This project implements a real-time control system for a 3D-printed robotic hand using Computer Vision and Digital Signal Processing techniques. The system analyzes hand movements from a video file and translates them into precise robotic finger movements via an Arduino and PCA9685 servo driver.



## Features
* Hand Movement Mimicking: Accurate tracking and translation of finger kinematics.
* Smooth Motion: Implements a Low-Pass Filter (Exponential Smoothing) to eliminate servo jitter and provide natural movement.
* Soft Start Sequence: Protects hardware by gradually initializing servos to the starting position.
* CAD Library: Includes a comprehensive set of 3D models in both STEP and STL formats.

##  Project Structure
* `MimicHand_control.py`: Python script for video processing, landmark analysis, and serial communication.
* `Arduino_prep.ino`: Arduino firmware for PCA9685 PWM control and system initialization.
* `CAD_Models/`:
    * `STEP_Files/`: Editable 3D models for design modifications.
    * `STL_Files/`: Ready-to-print files for 3D manufacturing.

## Hardware Requirements
* Arduino Uno / Nano
* PCA9685 16-Channel 12-bit PWM Servo Driver
* 5x Servo Motors (SG90 or MG90S)
* External 5V Power Supply
* Video source for movement analysis



##  Calibration
The finger movement sensitivity can be adjusted in `MimicHand_control.py` by modifying the interpolation ranges in the `map_finger` function to match specific mechanical limits.

##  License
This project is licensed under the MIT License.
