# Pinball-Arduino-accelerometer-
easy Pinball Arduino accelerometer 

GY-87 accelerometer
pro micro Arduino 

Connect the MPU6050 to the Arduino:

```
VCC to 5V on the Arduino.
GND to GND on the Arduino.
SCL to A5 on the Arduino (or the appropriate I2C clock pin on your specific Arduino model).
SDA to A4 on the Arduino (or the appropriate I2C data pin on your specific Arduino model).
```

This script uses 4g as it should cover the best sensitivity to range for tilt and bumping

```
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Joystick.h>

Adafruit_MPU6050 mpu;

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID, 
  JOYSTICK_TYPE_JOYSTICK, 0, 0,
  true, true, true,     // X, Y, Z Axis
  true, true, true,     // Rx, Ry, Rz
  false, false,         // rudder, throttle
  false, false, false); // accelerator, brake, steering

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Wire.begin();

    Serial.println("Initializing MPU6050...");
    if (!mpu.begin()) {
        Serial.println("Failed to find MPU6050 chip. Please check connections.");
        while (1) {
            delay(10);
        }
    }
    Serial.println("MPU6050 Found!");

    // Set accelerometer and gyro range
    mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
    mpu.setGyroRange(MPU6050_RANGE_250_DEG);

    // Initialize Joystick
    Joystick.begin();
    Joystick.setXAxisRange(-1023, 1023);
    Joystick.setYAxisRange(-1023, 1023);
    Joystick.setZAxisRange(-1023, 1023);
    Joystick.setRxAxisRange(-1023, 1023);
    Joystick.setRyAxisRange(-1023, 1023);
    Joystick.setRzAxisRange(-1023, 1023);
}

void loop() {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    // Print raw accelerometer and gyroscope values for debugging
    Serial.print("Raw Accel X: "); Serial.print(a.acceleration.x);
    Serial.print(" Y: "); Serial.print(a.acceleration.y);
    Serial.print(" Z: "); Serial.print(a.acceleration.z);
    Serial.print(" | Raw Gyro X: "); Serial.print(g.gyro.x);
    Serial.print(" Y: "); Serial.print(g.gyro.y);
    Serial.print(" Z: "); Serial.println(g.gyro.z);

    // Map accelerometer values to joystick ranges
    int16_t x = map(constrain(a.acceleration.x * 8192, -32768, 32767), -32768, 32767, -1023, 1023);
    int16_t y = map(constrain(a.acceleration.y * 8192, -32768, 32767), -32768, 32767, -1023, 1023);
    int16_t z = map(constrain(a.acceleration.z * 8192, -32768, 32767), -32768, 32767, -1023, 1023);

    // Map gyroscope values to joystick ranges
    int16_t rx = map(constrain(g.gyro.x * 131, -32768, 32767), -32768, 32767, -1023, 1023);
    int16_t ry = map(constrain(g.gyro.y * 131, -32768, 32767), -32768, 32767, -1023, 1023);
    int16_t rz = map(constrain(g.gyro.z * 131, -32768, 32767), -32768, 32767, -1023, 1023);

    // Set joystick axes
    Joystick.setXAxis(x);
    Joystick.setYAxis(y);
    Joystick.setZAxis(z);
    Joystick.setRxAxis(rx);
    Joystick.setRyAxis(ry);
    Joystick.setRzAxis(rz);

    Joystick.sendState();

    // Print mapped values for debugging
    Serial.print("Mapped Accel X: "); Serial.print(x);
    Serial.print(" Y: "); Serial.print(y);
    Serial.print(" Z: "); Serial.print(z);
    Serial.print(" | Mapped Gyro X: "); Serial.print(rx);
    Serial.print(" Y: "); Serial.print(ry);
    Serial.print(" Z: "); Serial.println(rz);

    delay(500);
}
```
install https://github.com/Cleric-K/vJoySerialFeeder

so we can pick and switch our axis
Keep z on atlegends for our plunger and change our x/y for sweet sweet tilt action

 Configure the Joystick in Windows
Open the Game Controllers Settings:

Press Win + R to open the Run dialog.
Type joy.cpl and press Enter.
Verify the Joystick:

In the Game Controllers window, you should see the Arduino listed as a joystick.
Select it and click Properties.
Test the Joystick:

In the Test tab, you can see the axes and buttons.
Move the MPU6050 to see the corresponding axes move in the Properties window. This verifies that the joystick is working correctly.

==========================================================================================================================================

The MPU6050 accelerometer can measure acceleration in different ranges: ±2g, ±4g, ±8g, and ±16g. These ranges represent the maximum acceleration values the sensor can measure in each axis (X, Y, Z) in terms of gravitational acceleration (g). Here's a breakdown of the differences and their impact on the readings:

Range Selection:

2g: The sensor can measure accelerations from -2g to +2g.
4g: The sensor can measure accelerations from -4g to +4g.
8g: The sensor can measure accelerations from -8g to +8g.
16g: The sensor can measure accelerations from -16g to +16g.
Resolution and Sensitivity:

The resolution (or sensitivity) of the accelerometer is determined by the range selected. Lower ranges (like 2g) provide higher sensitivity and finer resolution, meaning small changes in acceleration are more noticeable. Higher ranges (like 16g) provide lower sensitivity and coarser resolution, meaning the sensor can measure larger accelerations but with less detail in smaller changes.
For example, in the 2g range, the sensor can detect smaller movements and vibrations with higher precision. In the 16g range, the sensor can measure more significant accelerations, which might be useful in high-impact scenarios but with less detail for small changes.
Mapping to Joystick Range:

When mapping the sensor's raw data to a joystick's range, the factor used in the map function changes depending on the selected range.
In the code provided, this factor helps convert the sensor's readings to a range suitable for the joystick inputs. The mapping factor ensures that the full range of the sensor's output is utilized correctly.
Here are the specifics for each range:

2g Range:

Factor: 16384 (since 1g = 16384 LSBs (Least Significant Bits))
Example: For a ±2g range, the raw accelerometer value of 1g will be 16384, and -1g will be -16384.
High sensitivity, suitable for detecting small movements.
8g Range:

Factor: 4096 (since 1g = 4096 LSBs)
Example: For a ±8g range, the raw accelerometer value of 1g will be 4096, and -1g will be -4096.
Moderate sensitivity, suitable for more significant movements but still maintaining a good level of detail.
16g Range:

Factor: 2048 (since 1g = 2048 LSBs)
Example: For a ±16g range, the raw accelerometer value of 1g will be 2048, and -1g will be -2048.
Lower sensitivity, suitable for detecting very high accelerations but with less detail on small changes.
