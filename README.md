Note 09.10.2025

+. Median filter fully implemented for control system. Filtered ADC values are computed only after conversion is completed and a Callback for completion event is raised.

-. Measured distance currently is wrong. Not sure if due to MCU being damaged earlier (tried powering with Castle ESC) or errors in polynomial function. Sensor may need re-calibration and update Poly func with new coefficients.
