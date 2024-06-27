# Pinball-Arduino-accelerometer-
easy Pinball Arduino accelerometer 

GY-87 accelerometer
pro micro Arduino 

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
    mpu.setAccelerometerRange(MPU6050_RANGE_16_G);
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
    int16_t x = map(constrain(a.acceleration.x * 2048, -32768, 32767), -32768, 32767, -1023, 1023);
    int16_t y = map(constrain(a.acceleration.y * 2048, -32768, 32767), -32768, 32767, -1023, 1023);
    int16_t z = map(constrain(a.acceleration.z * 2048, -32768, 32767), -32768, 32767, -1023, 1023);

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
