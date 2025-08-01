# NSSSIP 2025 Error State Kalman Filter
    Welcome to the Readme, here you will find the following documentation of the ESKF:
    - Documentation of the Filter
    - Documentation of Arduino Library Implementation
    - Documentation of the Testing Setup
    - Example sensor input data
    - Example Outputs from the filter

## Filter Documentation
- Error State Kalman Filter Overview
- Assumptions
- Modular Sensor Update System
- Explanations of the linearizations
- Gating Mechanisms
- Direct, Indirect observations
- Weaknesses 
- Next Steps (fubar recovery, dynamic noise adjustment, takeoff detection, etc)
- Sources
    
## Arduino Library Implementation
- Installation and dependencies - Teensy
- Attach altitude_control.ino
- What to set using what
- Other dependencies required
- Units for the inputs

## Desktop Testing
- Description of Workflow
- Symbolink the library
- csv in and out format
- Setting acc_bias, gyro_bias
- Using the display_results function (make more elegant)

## Results
- Last two datasets, decent data from them
- Pros, Cons
- Discussion of sensor data improvements
- Discussion of improved gating for orientation

## Input Data Descriptions
- Squareflight
- Triflight2

## Next Steps
- Throw errors for the falling: drone flipping, bad covariance trace, large accel/gyro biases, no GPS lock
- Trim down compile time
- Make the display have arguments and be useful
- Add read until eof in main