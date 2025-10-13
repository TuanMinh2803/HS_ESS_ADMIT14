STM32-based Control system for an Autonomous vehicle

The repository hosts a control system for ADMIT14 Autonomous car of HS Esslingen. Aimed to be an educational platform for students in various fields and focused on autonomous driving as well as robotics.

Key features of control system:
- IMU for measurements of acceleration, angular velocity
- Integration of different IMUs
- Distance sensors for detecting the surrounding environment
- Multiple communication protocols to different peripherals


Dev notes:
- 09.10.2025:
    - Median filter fully implemented for control system. Filtered ADC values are computed only after conversion is completed and a Callback for completion event is raised.
    - Measured distance currently is wrong. Not sure if due to MCU being damaged earlier (tried powering with Castle ESC) or errors in polynomial function. Sensor may need re-calibration and update Poly func with new coefficients.

- 13.10.2025:
    - Finished driver for ICM20948. Waiting for IMU to arrive and continue with Kalman Filter testing