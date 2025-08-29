# Autonomous-Car
This project implements a simple autonomous car using the MSP432E401Y microcontroller. The
system drives the car forward and uses an I2C-based time-of-flight (ToF) sensor (VL53L1X) to detect
obstacles. When the vehicle approaches a wall within a certain distance, the microcontroller brings
the vehicle to a stop.
The project demonstrates integration of several embedded systems concepts:
1. Peripheral configuration (GPIO, I2C) on the MSP432E401Y.
2. Real-time sensor interfacing over I2C.
3. Motor control via H-Bridge driver.
4. System design that blends hardware (motors, sensors, microcontroller) with low-level
firmware.
