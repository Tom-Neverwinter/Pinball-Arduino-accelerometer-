# Arduino Pinball Accelerometer (Nudge Controller)

Turns an ATmega32U4 board (Arduino Leonardo, Pro Micro, etc.) plus a 6-axis
accelerometer into a USB joystick for virtual pinball software (VPX, Future
Pinball, etc.). Tilting/nudging the cabinet moves the joystick X/Y axes so
the software can register table nudges and tilts.

The original version targeted the **MPU-6050**, which is now discontinued.
This version drops it in favor of three parts that are all still in active
production, and the same sketch works with any one of them by changing a
single line.

## Supported sensors

Pick one — wiring (SDA/SCL/VIN/GND) is the same I2C hookup for all three,
only the address/library differs:

| Sensor | Library to install (Library Manager) |
|---|---|
| TDK **ICM-42688** | `DFRobot_ICM42688` |
| Bosch **BMI270** | `SparkFun BMI270 Arduino Library` |
| ST **LSM6DSOX** / LSM6 family | `Adafruit LSM6DS` (+ `Adafruit Unified Sensor`) |

You also need the `Joystick` library by Matthew Heironimus
(*Sketch → Include Library → Manage Libraries...*).

## Setup

1. Wire your chosen breakout's `SDA`/`SCL`/`VIN`/`GND` to the board's I2C pins.
2. Install the one library from the table above that matches your sensor
   (you only need that one, not all three).
3. Open `PinballAccelerometer/PinballAccelerometer.ino` and set:
   ```c++
   #define IMU_SENSOR SENSOR_LSM6   // or SENSOR_ICM42688 / SENSOR_BMI270
   ```
4. Flash it, open Serial Monitor at 115200 baud, and keep the board still
   for the ~3 second auto-calibration on boot.
5. Nudge the cabinet left/right and forward/back while watching the
   `TiltX`/`TiltY` debug output. If a direction feels backwards or swapped,
   fix it in the sketch — no math required:
   ```c++
   #define ACCEL_INVERT_X 1   // flip to -1 to invert
   #define ACCEL_INVERT_Y 1   // flip to -1 to invert
   #define ACCEL_SWAP_XY  0   // set to 1 if X/Y are swapped on your mount
   ```
   This exists because the three recommended breakout boards don't all
   share the same physical axis orientation on their silkscreen.
6. In your pinball software, map the nudge/tilt input to this device's
   joystick X/Y axes.

## How it works

- **Tilt** (`atan2` of the filtered accel vector) maps the cabinet's static
  lean to a joystick position, with a deadzone (`DEADZONE_ANGLE`) so small
  vibration doesn't move the axis, ramping to full deflection at
  `MAX_ANGLE`.
- **Nudge/shove** detection watches the *rate* of change of acceleration
  (m/s³, normalized by real elapsed time so it behaves the same regardless
  of which sensor's data rate is in play) and briefly forces the joystick
  to full-scale in the shoved direction for `SHOVE_DURATION` ms. A cooldown
  (`SHOVE_COOLDOWN_MS`) and a "don't retrigger mid-pulse" guard keep
  sustained vibration (flippers, solenoids, a ball drop) from latching the
  joystick at max deflection instead of giving a short, VPX-friendly pulse.
- A low-pass filter (`LPF_ALPHA`) smooths the tilt signal; jerk detection
  runs on the unfiltered signal so real bumps aren't blunted.

All of the above are `#define` constants near the top of the sketch —
tune `JERK_RATE_THRESHOLD`, `DEADZONE_ANGLE`, `MAX_ANGLE`, `LPF_ALPHA`,
`SHOVE_DURATION`, and `SHOVE_COOLDOWN_MS` to taste on your own table.

If the sensor isn't detected at boot, the onboard LED blinks and the
sketch keeps retrying rather than hanging forever, so a loose connector
doesn't require a full power cycle once reseated.

## Known limitation: one accelerometer per VPX instance

Visual Pinball X only reads nudge input from a single controller device.
If your cabinet also uses a separate USB device for the plunger (or
anything else with an axis), VPX may not distinguish between them
correctly. The commonly used workaround is
[joy2key](https://www.vpforums.org/index.php?showtopic=50747) to remap/
merge inputs at the OS level — this is a VPX/input-routing limitation, not
something this sketch can fix on its own.

## Hardware requirements

- An ATmega32U4-based board (Leonardo, Pro Micro, etc.) — needed for native
  USB HID joystick support.
- One of the three accelerometers listed above.

## Software requirements

- Arduino IDE
- `Joystick` library (Matthew Heironimus)
- The one sensor library matching your chosen chip (see table above)
