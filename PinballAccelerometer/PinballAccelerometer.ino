#include <Joystick.h>
#include <Wire.h>
#include <math.h>

// ========== SENSOR SELECTION ==========
// The MPU6050 this project used to target is discontinued. Pick whichever
// of these three replacements you actually have wired up and set
// IMU_SENSOR to match. All three are I2C, so wiring (SDA/SCL/VIN/GND) is
// unchanged from the MPU6050 -- only the address pin (AD0/SDO/SA0) differs
// per chip, handled below.
#define SENSOR_ICM42688 1  // TDK ICM-42688 -- needs "DFRobot_ICM42688" library
#define SENSOR_BMI270   2  // Bosch BMI270   -- needs "SparkFun BMI270 Arduino Library"
#define SENSOR_LSM6     3  // ST LSM6DSOX/LSM6 family -- needs "Adafruit LSM6DS" library

#define IMU_SENSOR SENSOR_LSM6   // <<< set this to your hardware

#if IMU_SENSOR == SENSOR_ICM42688
  #include <DFRobot_ICM42688.h>
  // AD0/SDO low = 0x68 (DFRobot_ICM42688_I2C_L_ADDR), high = 0x69 (_H_ADDR)
  DFRobot_ICM42688_I2C imu(DFRobot_ICM42688_I2C_L_ADDR);
#elif IMU_SENSOR == SENSOR_BMI270
  #include <SparkFun_BMI270_Arduino_Library.h>
  BMI270 imu;
#elif IMU_SENSOR == SENSOR_LSM6
  #include <Adafruit_LSM6DSOX.h>   // swap for Adafruit_LSM6DS33 etc. for other LSM6 parts; same API
  #include <Adafruit_Sensor.h>
  Adafruit_LSM6DSOX imu;
#else
  #error "Set IMU_SENSOR to SENSOR_ICM42688, SENSOR_BMI270, or SENSOR_LSM6"
#endif

// ========== CALIBRATION OFFSETS ==========
float accelXZeroG = 0;
float accelYZeroG = 0;

// ========== AXIS ORIENTATION (per-board) ==========
// The three recommended sensor boards do not share the same physical
// silkscreen axis orientation. After wiring a new board, watch the Serial
// "TiltX/TiltY" debug output while nudging left/right and forward/back, and
// flip these until the sign/axis matches the physical motion -- no math
// edits needed elsewhere.
#define ACCEL_INVERT_X 1   // 1 or -1
#define ACCEL_INVERT_Y 1   // 1 or -1
#define ACCEL_SWAP_XY  0   // 1 = swap X/Y (some boards mount rotated 90 deg)

// ========== FILTER + JOYSTICK CONSTANTS ==========

// Accel read delay
#define ACCEL_READ_DELAY_MS 5

// Low-pass filter alpha (0.0 = heavy smoothing, 1.0 = minimal smoothing)
#define LPF_ALPHA 0.2f

// Tilt angles for piecewise mapping
// e.g. under ±3 degrees => 0 joystick, above ±15 => full joystick
#define DEADZONE_ANGLE 3.0f
#define MAX_ANGLE      15.0f

// If you only do angle-based tilt, you might keep 45 here, but for a
// piecewise approach, we explicitly define in the piecewise function
// how angles map to the joystick range. So TILT_RANGE isn't used
// as before, but let's keep a reference:
#define TILT_RANGE 45.0f

// For normal "small vibrations" vs real "shoves". This is a *rate*
// (m/s^3 = change in acceleration per second), not a raw per-sample delta,
// so it stays consistent no matter what ODR the chosen sensor runs at.
// Tune on your table: raise it if flippers/solenoids cause false nudges,
// lower it if real bumps aren't registering.
#define JERK_RATE_THRESHOLD 500.0f  // (m/s^3)
#define SHOVE_DURATION   100  // (ms) how long to hold full-scale joystick after a shove
#define SHOVE_COOLDOWN_MS 50  // (ms) minimum gap before a new shove can trigger

// Joystick range
#define JOYSTICK_MIN -1023
#define JOYSTICK_MAX  1023

// ========== GLOBAL VARIABLES ==========
// Filtered accel values
float filteredAccelX = 0, filteredAccelY = 0, filteredAccelZ = 0;
// Raw accel values
float rawAccelX = 0, rawAccelY = 0, rawAccelZ = 0;

// Variables for jerk detection
static float oldAccelX = 0, oldAccelY = 0, oldAccelZ = 0;
static unsigned long oldReadMicros = 0;      // Timestamp of the previous sample
static bool  isShoveActive = false;          // Are we in a shove pulse?
static float shoveDirX = 0, shoveDirY = 0;   // Which axis got shoved
static unsigned long lastShoveTime = 0;      // When did the current shove start?
static unsigned long lastShoveEndTime = 0;   // When did the last shove pulse end?
static unsigned int  imuFaultCount = 0;      // Consecutive failed sensor reads

// ========== JOYSTICK OBJECT ==========
Joystick_ Joystick(
  JOYSTICK_DEFAULT_REPORT_ID,
  JOYSTICK_TYPE_GAMEPAD,
  0, // # of buttons
  0, // # of hat switches
  true,  // X
  true,  // Y
  false, // Z
  false, // Rx
  false, // Ry
  false, // Rz
  false, // rudder
  false, // throttle
  false, // accelerator
  false, // brake
  false  // steering
);

// ========== FUNCTION PROTOTYPES ==========
bool imuBegin();
bool imuReadAccel(float &x, float &y, float &z);
void calibrateAccelerometer();
void readAccelerometer();
void handleTilt();
float lowPassFilter(float newValue, float previousValue, float alpha);

// This function will map tilt angles with a deadzone and a piecewise “full” portion
float piecewiseAngleMapping(float angle);

// ========== SETUP ==========
void setup() {
  Serial.begin(115200);
  Wire.begin();

  // Initialize IMU. Retry instead of hanging forever so a loose connector
  // or a cold-boot I2C hiccup doesn't brick the USB joystick permanently --
  // the LED blinks while waiting so it's visible without a serial monitor.
  pinMode(LED_BUILTIN, OUTPUT);
  while (!imuBegin()) {
    Serial.println("IMU Not Found! Retrying...");
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(500);
  }
  Serial.println("IMU Found!");
  digitalWrite(LED_BUILTIN, LOW);
  calibrateAccelerometer();

  // Initialize joystick
  Joystick.begin();
  Joystick.setXAxisRange(JOYSTICK_MIN, JOYSTICK_MAX);
  Joystick.setYAxisRange(JOYSTICK_MIN, JOYSTICK_MAX);

  // Initialize filtered values to zero
  filteredAccelX = 0.0f;
  filteredAccelY = 0.0f;
  filteredAccelZ = 0.0f;

  // Also init oldAccel for jerk detection
  oldAccelX = 0.0f;
  oldAccelY = 0.0f;
  oldAccelZ = 0.0f;
}

// ========== MAIN LOOP ==========
void loop() {
  readAccelerometer();
  handleTilt();
  delay(ACCEL_READ_DELAY_MS);
}

// ========== IMU BACKEND (per-sensor init + read, in m/s^2) ==========
bool imuBegin() {
#if IMU_SENSOR == SENSOR_ICM42688
  if (imu.begin() != 0) return false;
  imu.setODRAndFSR(ACCEL, ODR_500HZ, FSR_2); // ODR 500Hz, +-4g (matches old MPU6050 range)
  imu.startAccelMeasure(LN_MODE);
  return true;
#elif IMU_SENSOR == SENSOR_BMI270
  return imu.beginI2C() == BMI2_OK;
#elif IMU_SENSOR == SENSOR_LSM6
  if (!imu.begin_I2C()) return false;
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_4_G); // matches old MPU6050 range
  return true;
#endif
}

// Returns false if the sensor reported a bus/read error (BMI270, LSM6). The
// ICM42688 DFRobot library doesn't expose a per-read status, so its reads
// always report success -- rely on imuBegin()'s check for that sensor.
bool imuReadAccel(float &x, float &y, float &z) {
  bool ok = true;
#if IMU_SENSOR == SENSOR_ICM42688
  const float MG_TO_MS2 = 0.00980665f;
  x = imu.getAccelDataX() * MG_TO_MS2;
  y = imu.getAccelDataY() * MG_TO_MS2;
  z = imu.getAccelDataZ() * MG_TO_MS2;
#elif IMU_SENSOR == SENSOR_BMI270
  const float G_TO_MS2 = 9.80665f;
  ok = (imu.getSensorData() == BMI2_OK);
  x = imu.data.accelX * G_TO_MS2;
  y = imu.data.accelY * G_TO_MS2;
  z = imu.data.accelZ * G_TO_MS2;
#elif IMU_SENSOR == SENSOR_LSM6
  sensors_event_t accel, gyro, temp;
  ok = imu.getEvent(&accel, &gyro, &temp);
  x = accel.acceleration.x;
  y = accel.acceleration.y;
  z = accel.acceleration.z;
#endif

#if ACCEL_SWAP_XY
  float tmp = x; x = y; y = tmp;
#endif
  x *= ACCEL_INVERT_X;
  y *= ACCEL_INVERT_Y;

  return ok;
}

// ========== CALIBRATE ACCELEROMETER ==========
void calibrateAccelerometer() {
  Serial.println("Calibrating accelerometer... Keep the device still.");
  delay(3000);

  const int calibrationSamples = 500;
  float sumX = 0, sumY = 0;
  int gotSamples = 0;

  while (gotSamples < calibrationSamples) {
    float x, y, z;
    if (imuReadAccel(x, y, z)) {
      sumX += x;
      sumY += y;
      gotSamples++;
    }
    delay(5);
  }

  // Average to find zero-G offsets
  accelXZeroG = sumX / calibrationSamples;
  accelYZeroG = sumY / calibrationSamples;

  oldReadMicros = micros();

  Serial.print("Calibration complete. Zero G offsets: X=");
  Serial.print(accelXZeroG);
  Serial.print(", Y=");
  Serial.println(accelYZeroG);
}

// ========== READ ACCELEROMETER + DETECT JERK ==========
void readAccelerometer() {
  float accelX, accelY, accelZ;
  if (!imuReadAccel(accelX, accelY, accelZ)) {
    // Bad I2C read (bus glitch/EMI from coils) -- skip this cycle rather
    // than feeding a stale/zero sample into the filter and jerk math.
    imuFaultCount++;
    if (imuFaultCount % 100 == 1) {
      Serial.print("IMU read fault, count=");
      Serial.println(imuFaultCount);
    }
    return;
  }
  imuFaultCount = 0;

  // Subtract offset
  rawAccelX = accelX - accelXZeroG;
  rawAccelY = accelY - accelYZeroG;
  rawAccelZ = accelZ; // Usually around 9.8 m/s^2 if upright

  // Low-pass filter
  filteredAccelX = lowPassFilter(rawAccelX, filteredAccelX, LPF_ALPHA);
  filteredAccelY = lowPassFilter(rawAccelY, filteredAccelY, LPF_ALPHA);
  filteredAccelZ = lowPassFilter(rawAccelZ, filteredAccelZ, LPF_ALPHA);

  // ---- Detect if there's a big sudden change (jerk) for a nudge ----
  // Normalized to a *rate* (change per second) so the same physical bump
  // trips the same threshold regardless of which sensor's ODR/loop timing
  // is actually in play.
  unsigned long nowMicros = micros();
  float dtSeconds = (nowMicros - oldReadMicros) / 1000000.0f;
  if (dtSeconds < 0.0005f) dtSeconds = 0.0005f; // guard against divide-by-near-zero

  float jerkRateX = (rawAccelX - oldAccelX) / dtSeconds;
  float jerkRateY = (rawAccelY - oldAccelY) / dtSeconds;

  // Update old readings for next pass
  oldAccelX = rawAccelX;
  oldAccelY = rawAccelY;
  oldAccelZ = rawAccelZ;
  oldReadMicros = nowMicros;

  // Only arm a new shove if one isn't already in progress and we're past
  // the cooldown -- otherwise sustained vibration (flippers, solenoids,
  // ball drops) keeps re-triggering every loop and pins the joystick at
  // max deflection instead of a short pulse.
  unsigned long now = millis();
  bool canTrigger = !isShoveActive && (now - lastShoveEndTime >= SHOVE_COOLDOWN_MS);
  if (canTrigger && (fabs(jerkRateX) > JERK_RATE_THRESHOLD || fabs(jerkRateY) > JERK_RATE_THRESHOLD)) {
    isShoveActive = true;
    lastShoveTime = now;

    // Decide if it's primarily an X or Y shove, based on which jerk is bigger
    if (fabs(jerkRateX) > fabs(jerkRateY)) {
      // X axis shove
      shoveDirX = (jerkRateX > 0) ? 1.0f : -1.0f;
      shoveDirY = 0.0f;
    } else {
      // Y axis shove
      shoveDirY = (jerkRateY > 0) ? 1.0f : -1.0f;
      shoveDirX = 0.0f;
    }

    Serial.println("Nudge detected!");
  }
}

// ========== HANDLE TILT AND SHOVE ==========
void handleTilt() {
  // 1) Calculate tilt angles in degrees
  //    For "side to side" => tiltX = atan2(Y,Z)
  //    For "forward/back" => tiltY = atan2(X,Z)
  float tiltX = atan2(filteredAccelY, filteredAccelZ) * RAD_TO_DEG;
  float tiltY = atan2(filteredAccelX, filteredAccelZ) * RAD_TO_DEG;

  // 2) Map angles to joystick with piecewise approach (deadzone + quick ramp)
  float joystickX = piecewiseAngleMapping(tiltX);
  float joystickY = piecewiseAngleMapping(tiltY);

  // 3) If there's an active shove, override joystick for a brief duration
  if (isShoveActive) {
    unsigned long elapsed = millis() - lastShoveTime;
    if (elapsed < SHOVE_DURATION) {
      // Force full joystick on whichever axis was shoved
      if (shoveDirX != 0) {
        joystickX = (shoveDirX > 0) ? JOYSTICK_MAX : JOYSTICK_MIN;
      }
      if (shoveDirY != 0) {
        joystickY = (shoveDirY > 0) ? JOYSTICK_MAX : JOYSTICK_MIN;
      }
    } else {
      // Time to end the shove pulse
      isShoveActive = false;
      shoveDirX = 0;
      shoveDirY = 0;
      lastShoveEndTime = millis();
    }
  }

  // 4) Possibly invert Y to match your preferred "push forward is negative"
  joystickY = -joystickY;

  // 5) Constrain final values to joystick range
  joystickX = constrain(joystickX, JOYSTICK_MIN, JOYSTICK_MAX);
  joystickY = constrain(joystickY, JOYSTICK_MIN, JOYSTICK_MAX);

  // 6) Send to joystick
  Joystick.setXAxis((int)joystickX);
  Joystick.setYAxis((int)joystickY);
  Joystick.sendState();

  // ========== Debug Output ==========
  Serial.print("TiltX: "); Serial.print(tiltX);
  Serial.print(" TiltY: "); Serial.print(tiltY);
  Serial.print(" -> JoyX: "); Serial.print(joystickX);
  Serial.print(" JoyY: "); Serial.print(joystickY);
  Serial.print("  [ShoveActive="); Serial.print(isShoveActive);
  Serial.println("]");
}

// ========== PIECEWISE ANGLE MAPPING ==========
// If |angle| <= DEADZONE_ANGLE => 0
// If |angle| >= MAX_ANGLE => ±1023
// Else scale linearly in-between
float piecewiseAngleMapping(float angle) {
  float absAngle = fabs(angle);

  // Deadzone
  if (absAngle <= DEADZONE_ANGLE) {
    return 0.0f;
  }
  // Full scale
  else if (absAngle >= MAX_ANGLE) {
    return (angle > 0) ? JOYSTICK_MAX : JOYSTICK_MIN;
  }
  // In-between => map from [DEADZONE_ANGLE..MAX_ANGLE] to [0..1023]
  else {
    // E.g. if angle=5 deg, with deadzone=3, max=15 => scale = (5-3)/(15-3)=2/12=0.1667 => ~170
    float scale = (absAngle - DEADZONE_ANGLE) / (MAX_ANGLE - DEADZONE_ANGLE);
    float value = scale * (float)JOYSTICK_MAX;
    return (angle > 0) ? value : -value;
  }
}

// ========== LOW-PASS FILTER ==========
// newValue: current measurement
// previousValue: last filtered reading
// alpha: 0..1   (0 => heavy smoothing, 1 => no smoothing)
float lowPassFilter(float newValue, float previousValue, float alpha) {
  return previousValue + alpha * (newValue - previousValue);
}
