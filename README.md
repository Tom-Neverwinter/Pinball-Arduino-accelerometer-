# Arduino-Based Accelerometer Pinball Controller

This project uses an Arduino Leonardo (or compatible ATmega32U4-based board) and an MPU-6050 accelerometer to create a USB joystick controller for virtual pinball software like Visual Pinball X (VPX) and Future Pinball. The accelerometer translates the tilt of the device into joystick X and Y axis movements, providing a realistic way to nudge and control the virtual pinball table.

## Features

*   **Tilt Control:** Accurately maps accelerometer readings to joystick X and Y axes for controlling the tilt/nudge mechanism in pinball simulations.
*   **Adjustable Sensitivity:** Easily fine-tune the sensitivity of the tilt control using the `ACCEL_TILT_SENSITIVITY` constant in the code, allowing you to customize the responsiveness to your preference.
*   **Customizable Mapping:** Employs a custom mapping function (`customMap()`) with a cubic curve for finer control over the joystick output, especially for subtle movements.
*   **Joystick Scaling:** Includes a `JOYSTICK_SCALING_FACTOR` constant, to provide an additional layer of control over the joystick output, independent of the sensitivity.
*   **Calibration:** Includes a calibration routine to compensate for sensor offsets, ensuring the joystick is centered when the device is level.
*   **VPX/Future Pinball Compatibility:** Designed to work seamlessly with virtual pinball software that supports joystick input for tilt/nudge.

## Hardware Requirements

*   **Arduino Board:** An Arduino Leonardo, Pro Micro, or any other ATmega32U4-based board is required. These boards have built-in USB HID capabilities, allowing them to act as a joystick.
*   **Accelerometer:** An MPU-6050 6-axis accelerometer and gyroscope module. The code is specifically written for the MPU-6050 but could be adapted for other accelerometers.
*   **GY-87 Module (Optional):** The GY-87 module is a convenient breakout board that includes the MPU-6050, HMC5883L magnetometer, and BMP180 barometer. While the magnetometer is not used in this code, the GY-87 can still be used as a carrier for the MPU-6050.

## Software Requirements

*   **Arduino IDE:** To compile and upload the code to your Arduino.
*   **Joystick Library:** The Arduino Joystick library by Matthew Heironimus. Install it through the Arduino IDE Library Manager (*Sketch -> Include Library -> Manage Libraries...*).
*   **Adafruit MPU6050 Library:** Install it through the Arduino IDE Library Manager.
*   **Adafruit Sensor Library:** Install it through the Arduino IDE Library Manager.
*   **Virtual Pinball Software:** VPX, Future Pinball, or any other pinball software that supports joystick input for tilt/nudge.

## Code Description

The code is written in C++ for the Arduino platform. Here's a breakdown of the main sections:

### Includes

```c++
#include <Joystick.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
