#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps612.h"

/*
  Arduino Nano + 2 MPU6050 + EMG A10-09
  MPU part upgraded from MPU_success.cpp.
  EMG part is intentionally unchanged: analogRead(A0), same Serial column.

  Output Serial for MATLAB run_emg_live_local.m:
      time_ms,elbow_vector_filtered_deg,emg_raw

  MPU upper arm: 0x68, AD0/ADDR = GND
  MPU forearm:   0x69, AD0/ADDR = VCC
  EMG OUT:       A0

  ZERO pose:
      - Keep arm straight and still during startup.
      - Send 'c' or 'C' in Serial Monitor to re-zero if needed.
*/

// ============================= CONFIG =============================
const uint8_t MPU_UPPER_ADDR = 0x68;
const uint8_t MPU_LOWER_ADDR = 0x69;
const int EMG_PIN = A0;

const uint32_t SERIAL_BAUD = 115200;
const uint32_t I2C_CLOCK_HZ = 100000UL;      // stable for long wires / hand soldered PCB
const uint16_t OUTPUT_INTERVAL_MS = 10;      // 100 Hz, keep EMG sampling behavior close to old code
const uint16_t ZERO_SAMPLES = 80;
const uint32_t ZERO_TIMEOUT_MS = 5000UL;

const uint8_t MEDIAN_WINDOW = 5;
const float EMA_ALPHA = 0.22f;               // lower = smoother, higher = faster response
const float MAX_ELBOW_DEG = 160.0f;

MPU6050 mpuUpper(MPU_UPPER_ADDR);
MPU6050 mpuLower(MPU_LOWER_ADDR);

uint8_t fifoUpper[64];
uint8_t fifoLower[64];

bool dmpReady = false;
uint32_t lastOutputMs = 0;

// ============================= MATH DATA =============================
struct Q {
  float w;
  float x;
  float y;
  float z;
};

struct V3 {
  float x;
  float y;
  float z;
};

// Local longitudinal arm axis. If both MPU boards are mounted in the same direction,
// the angle between these axes is stable even if the exact board direction is reversed.
const V3 ARM_AXIS_LOCAL = {1.0f, 0.0f, 0.0f};

Q qUpperZero = {1.0f, 0.0f, 0.0f, 0.0f};
Q qLowerZero = {1.0f, 0.0f, 0.0f, 0.0f};
Q qRelativeZero = {1.0f, 0.0f, 0.0f, 0.0f};

Q qUpperPrevious = {1.0f, 0.0f, 0.0f, 0.0f};
Q qLowerPrevious = {1.0f, 0.0f, 0.0f, 0.0f};
bool previousQValid = false;

float angleWindow[MEDIAN_WINDOW] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
uint8_t angleWindowCount = 0;
uint8_t angleWindowIndex = 0;
float filteredElbowDeg = 0.0f;
bool filterInitialized = false;

// ============================= QUATERNION HELPERS =============================
float clampFloat(float value, float lower, float upper) {
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

Q normalizeQ(Q q) {
  float n = sqrtf(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
  if (n < 1e-8f) return {1.0f, 0.0f, 0.0f, 0.0f};

  q.w /= n;
  q.x /= n;
  q.y /= n;
  q.z /= n;
  return q;
}

Q conjugateQ(const Q &q) {
  return {q.w, -q.x, -q.y, -q.z};
}

Q multiplyRawQ(const Q &a, const Q &b) {
  Q r;
  r.w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z;
  r.x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y;
  r.y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x;
  r.z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w;
  return r;
}

Q multiplyOrientationQ(const Q &a, const Q &b) {
  return normalizeQ(multiplyRawQ(a, b));
}

float dotQ(const Q &a, const Q &b) {
  return a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;
}

Q negateQ(Q q) {
  return {-q.w, -q.x, -q.y, -q.z};
}

V3 rotateVectorByQuaternion(const Q &q, const V3 &v) {
  Q p = {0.0f, v.x, v.y, v.z};
  Q rotated = multiplyRawQ(multiplyRawQ(q, p), conjugateQ(q));
  return {rotated.x, rotated.y, rotated.z};
}

float dotV3(const V3 &a, const V3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Q calculateRelativeOrientation(const Q &qUpper, const Q &qLower) {
  return multiplyOrientationQ(conjugateQ(qUpper), qLower);
}

Q calculateElbowDelta(const Q &qUpper, const Q &qLower) {
  Q qRelativeNow = calculateRelativeOrientation(qUpper, qLower);
  return multiplyOrientationQ(conjugateQ(qRelativeZero), qRelativeNow);
}

float calculateElbowVectorAngleDeg(const Q &qElbowDelta) {
  V3 currentLowerAxisInUpperFrame = rotateVectorByQuaternion(qElbowDelta, ARM_AXIS_LOCAL);
  float cosine = clampFloat(dotV3(ARM_AXIS_LOCAL, currentLowerAxisInUpperFrame), -1.0f, 1.0f);
  return acosf(cosine) * 180.0f / PI;
}

// ============================= ANGLE FILTER =============================
void resetAngleFilter() {
  for (uint8_t i = 0; i < MEDIAN_WINDOW; i++) {
    angleWindow[i] = 0.0f;
  }
  angleWindowCount = 0;
  angleWindowIndex = 0;
  filteredElbowDeg = 0.0f;
  filterInitialized = false;
}

float medianCurrentWindow() {
  float sorted[MEDIAN_WINDOW];

  for (uint8_t i = 0; i < angleWindowCount; i++) {
    sorted[i] = angleWindow[i];
  }

  for (uint8_t i = 0; i < angleWindowCount; i++) {
    for (uint8_t j = i + 1; j < angleWindowCount; j++) {
      if (sorted[j] < sorted[i]) {
        float temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }

  if (angleWindowCount == 0) return 0.0f;
  return sorted[angleWindowCount / 2];
}

void updateAngleFilter(float rawAngle, float &medianAngle, float &filteredAngle) {
  angleWindow[angleWindowIndex] = rawAngle;
  angleWindowIndex = (angleWindowIndex + 1) % MEDIAN_WINDOW;

  if (angleWindowCount < MEDIAN_WINDOW) {
    angleWindowCount++;
  }

  medianAngle = medianCurrentWindow();

  if (!filterInitialized) {
    filteredElbowDeg = medianAngle;
    filterInitialized = true;
  } else {
    filteredElbowDeg += EMA_ALPHA * (medianAngle - filteredElbowDeg);
  }

  filteredAngle = filteredElbowDeg;
}

// ============================= MPU READ =============================
bool readQuaternion(MPU6050 &mpu, uint8_t *fifoBuffer, Q &out) {
  if (!mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    return false;
  }

  Quaternion sensorQ;
  mpu.dmpGetQuaternion(&sensorQ, fifoBuffer);
  out = normalizeQ({sensorQ.w, sensorQ.x, sensorQ.y, sensorQ.z});
  return true;
}

bool readBoth(Q &qUpper, Q &qLower) {
  if (!readQuaternion(mpuUpper, fifoUpper, qUpper)) return false;
  if (!readQuaternion(mpuLower, fifoLower, qLower)) return false;

  // q and -q are the same orientation. Keep signs continuous to avoid sudden jumps.
  if (previousQValid) {
    if (dotQ(qUpper, qUpperPrevious) < 0.0f) qUpper = negateQ(qUpper);
    if (dotQ(qLower, qLowerPrevious) < 0.0f) qLower = negateQ(qLower);
  }

  qUpperPrevious = qUpper;
  qLowerPrevious = qLower;
  previousQValid = true;
  return true;
}

// ============================= INIT DMP =============================
bool initializeOneMPU(MPU6050 &mpu, const __FlashStringHelper *name) {
  Serial.print(F("WAIT: Checking "));
  Serial.println(name);

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.print(F("ERROR: Cannot find "));
    Serial.println(name);
    return false;
  }

  uint8_t status = mpu.dmpInitialize();
  if (status != 0) {
    Serial.print(F("ERROR: DMP init failed for "));
    Serial.print(name);
    Serial.print(F(", code="));
    Serial.println(status);
    return false;
  }

  Serial.print(F("WAIT: Calibrating "));
  Serial.println(name);
  Serial.println(F("WAIT: Keep sensor still..."));

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);
  mpu.resetFIFO();

  Serial.print(F("WAIT: DMP OK: "));
  Serial.println(name);
  return true;
}

// ============================= ZERO POSE =============================
bool calibrateZeroPose() {
  Serial.println(F("WAIT: ZERO pose. Keep arm straight and still..."));
  delay(300);

  previousQValid = false;
  mpuUpper.resetFIFO();
  mpuLower.resetFIFO();

  Q upperFirst = {1.0f, 0.0f, 0.0f, 0.0f};
  Q lowerFirst = {1.0f, 0.0f, 0.0f, 0.0f};
  Q upperSum = {0.0f, 0.0f, 0.0f, 0.0f};
  Q lowerSum = {0.0f, 0.0f, 0.0f, 0.0f};

  uint16_t count = 0;
  uint32_t startMs = millis();

  while (count < ZERO_SAMPLES && millis() - startMs < ZERO_TIMEOUT_MS) {
    Q qUpper;
    Q qLower;

    if (!readBoth(qUpper, qLower)) {
      continue;
    }

    if (count == 0) {
      upperFirst = qUpper;
      lowerFirst = qLower;
    }

    if (dotQ(qUpper, upperFirst) < 0.0f) qUpper = negateQ(qUpper);
    if (dotQ(qLower, lowerFirst) < 0.0f) qLower = negateQ(qLower);

    upperSum.w += qUpper.w;
    upperSum.x += qUpper.x;
    upperSum.y += qUpper.y;
    upperSum.z += qUpper.z;

    lowerSum.w += qLower.w;
    lowerSum.x += qLower.x;
    lowerSum.y += qLower.y;
    lowerSum.z += qLower.z;

    count++;
  }

  if (count < 10) {
    Serial.println(F("ERROR: Not enough DMP packets for ZERO pose."));
    return false;
  }

  qUpperZero = normalizeQ(upperSum);
  qLowerZero = normalizeQ(lowerSum);
  qRelativeZero = calculateRelativeOrientation(qUpperZero, qLowerZero);

  qUpperPrevious = qUpperZero;
  qLowerPrevious = qLowerZero;
  previousQValid = true;

  resetAngleFilter();

  Serial.print(F("WAIT: ZERO OK, samples="));
  Serial.println(count);
  Serial.println(F("WAIT: MPU angle now uses relative quaternion + longitudinal arm-axis vector."));
  Serial.println(F("time_ms,elbow_vector_filtered_deg,emg_raw"));
  return true;
}

// ============================= SETUP =============================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println(F("WAIT: Starting 2x MPU6050 + EMG system..."));
  Serial.println(F("WAIT: MPU formula = stable quaternion/vector method from MPU_success.cpp"));
  Serial.println(F("WAIT: EMG path unchanged: analogRead(A0)."));

  Wire.begin(); // Nano/Uno: SDA=A4, SCL=A5
  Wire.setClock(I2C_CLOCK_HZ);

  bool upperOK = initializeOneMPU(mpuUpper, F("Upper MPU 0x68"));
  bool lowerOK = initializeOneMPU(mpuLower, F("Forearm MPU 0x69"));

  dmpReady = upperOK && lowerOK;
  if (!dmpReady) {
    Serial.println(F("ERROR: MPU initialization failed. System stopped."));
    return;
  }

  Serial.println(F("WAIT: In 3 seconds, current straight/still arm pose will become 0 deg."));
  delay(3000);

  if (!calibrateZeroPose()) {
    dmpReady = false;
  }
}

// ============================= LOOP =============================
void loop() {
  if (!dmpReady) return;

  if (Serial.available() > 0) {
    char cmd = (char)Serial.read();
    if (cmd == 'c' || cmd == 'C') {
      calibrateZeroPose();
      lastOutputMs = millis();
      return;
    }
  }

  uint32_t now = millis();
  if (now - lastOutputMs < OUTPUT_INTERVAL_MS) return;

  Q qUpper;
  Q qLower;

  if (!readBoth(qUpper, qLower)) return;
  lastOutputMs = now;

  Q elbowDelta = calculateElbowDelta(qUpper, qLower);

  float elbowRaw = calculateElbowVectorAngleDeg(elbowDelta);
  elbowRaw = clampFloat(elbowRaw, 0.0f, MAX_ELBOW_DEG);

  float elbowMedian = 0.0f;
  float elbowFiltered = 0.0f;
  updateAngleFilter(elbowRaw, elbowMedian, elbowFiltered);
  elbowFiltered = clampFloat(elbowFiltered, 0.0f, MAX_ELBOW_DEG);

  // EMG is intentionally not changed.
  int emg_raw = analogRead(EMG_PIN);

  Serial.print(now);
  Serial.print(',');
  Serial.print(elbowFiltered, 2);
  Serial.print(',');
  Serial.println(emg_raw);
}
