#include <Arduino.h>
#include <Wire.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

/*
  TWO MPU6050 - ELBOW FLEXION BY ARM-AXIS VECTOR
  =================================================
  Muc tieu:
    1) Van in goc thay doi rieng cua tung MPU de debug.
    2) Van in rel_dX/Y/Z va goc cu axis-Y de so sanh.
    3) Tinh goc khuyu tay moi bang goc giua truc doc hai doan tay.

  Cach gan MPU:
    - Hai MPU gan cung huong, giong "mat dong ho" tren tay trai.
    - +X huong doc canh tay ve phia cac ngon tay.
    - +Z vuong goc ra khoi mat cam bien.
    - Upper MPU (0x68): bap tay.
    - Lower MPU (0x69): cang tay.

  Vi sao dung truc X?
    - +X cua upper bieu dien huong doc cua bap tay.
    - +X cua lower bieu dien huong doc cua cang tay.
    - Goc giua hai huong doc nay chinh la do gap khuyu tay.
    - Khi xoay long ban tay quanh truc doc cang tay, huong +X gan nhu
      khong doi, vi vay goc khuuy it bi anh huong hon cach chi lay rel_dY.

  Tu the ZERO:
    - Duoi thang tay va giu dung tu the moc cua ban.
    - Long ban tay up xuong.
    - Giu yen trong luc code lay trung binh.
    - Gui 'c' hoac 'C' qua Serial de dat lai ZERO.

  Output CSV:
    time_ms,
    upper_dX_deg,upper_dY_deg,upper_dZ_deg,
    lower_dX_deg,lower_dY_deg,lower_dZ_deg,
    rel_dX_deg,rel_dY_deg,rel_dZ_deg,
    elbow_axisY_old_deg,
    elbow_vector_raw_deg,elbow_vector_median_deg,elbow_vector_filtered_deg

  Cot de su dung ve sau:
    - elbow_vector_filtered_deg: goc gap khuuyu tay moi, da lam muot.
*/

// ============================= CAU HINH =============================
const uint8_t MPU_UPPER_ADDR = 0x68; // AD0 = GND
const uint8_t MPU_LOWER_ADDR = 0x69; // AD0 = VCC

const uint32_t SERIAL_BAUD = 115200;
const uint32_t I2C_CLOCK_HZ = 100000UL; // On dinh hon khi day I2C dai/PCB han tay.
const uint16_t OUTPUT_INTERVAL_MS = 20; // 50 Hz
const uint16_t ZERO_SAMPLES = 80;

const uint8_t MEDIAN_WINDOW = 5;
const float EMA_ALPHA = 0.22f; // Tang len neu muon bam nhanh hon; giam neu muon muot hon.

MPU6050 mpuUpper(MPU_UPPER_ADDR);
MPU6050 mpuLower(MPU_LOWER_ADDR);

uint8_t fifoUpper[64];
uint8_t fifoLower[64];

bool dmpReady = false;
uint32_t lastOutputMs = 0;

// ============================= DU LIEU TOAN =============================
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

const V3 ARM_AXIS_LOCAL = {1.0f, 0.0f, 0.0f}; // +X doc theo tung doan tay.

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

// ============================= QUATERNION =============================
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

float clampFloat(float value, float lower, float upper) {
  if (value < lower) return lower;
  if (value > upper) return upper;
  return value;
}

float wrapDeg(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

V3 rotateVectorByQuaternion(const Q &q, const V3 &v) {
  Q p = {0.0f, v.x, v.y, v.z};
  Q rotated = multiplyRawQ(multiplyRawQ(q, p), conjugateQ(q));
  return {rotated.x, rotated.y, rotated.z};
}

float dotV3(const V3 &a, const V3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

/*
  Twist theo X/Y/Z chi giu lai de debug va de so sanh cong thuc cu.
*/
float twistAngleDeg(const Q &q, char axis) {
  float component = q.z;

  if (axis == 'X') component = q.x;
  if (axis == 'Y') component = q.y;

  float n = sqrtf(q.w * q.w + component * component);
  if (n < 1e-8f) return 0.0f;

  float w = q.w / n;
  float v = component / n;
  return wrapDeg(2.0f * atan2f(v, w) * 180.0f / PI);
}

/*
  qRelative: huong cua cang tay trong he toa do cua bap tay.
  Day la y tuong quan trong ke thua tu chuong trinh mo phong 3D.
*/
Q calculateRelativeOrientation(const Q &qUpper, const Q &qLower) {
  return multiplyOrientationQ(conjugateQ(qUpper), qLower);
}

/*
  Sau ZERO, tat ca chuyen dong tuong doi bat dau tu identity.
  Cach nay bu tru viec hai MPU co the khong duoc dat thang tuyet doi.
*/
Q calculateElbowDelta(const Q &qUpper, const Q &qLower) {
  Q qRelativeNow = calculateRelativeOrientation(qUpper, qLower);
  return multiplyOrientationQ(conjugateQ(qRelativeZero), qRelativeNow);
}

/*
  Cong thuc goc moi:
    - Lay vector +X ban dau cua cang tay.
    - Xoay vector do bang quaternion chuyen dong tuong doi sau ZERO.
    - Tinh goc giua +X ban dau va +X hien tai.

  Neu lower chi xoay quanh truc doc canh tay (+X), vector +X khong doi,
  do do goc khuuyu khong bi nham thanh dong tac xoay co tay.
*/
float calculateElbowVectorAngleDeg(const Q &qElbowDelta) {
  V3 currentLowerAxisInUpperFrame = rotateVectorByQuaternion(qElbowDelta, ARM_AXIS_LOCAL);
  float cosine = clampFloat(dotV3(ARM_AXIS_LOCAL, currentLowerAxisInUpperFrame), -1.0f, 1.0f);
  return acosf(cosine) * 180.0f / PI;
}

// ============================= LOC GOC =============================
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

// ============================= DOC MPU =============================
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

  // q va -q la cung mot huong. Dong bo dau de cac cot debug khong nhay bat ngo.
  if (previousQValid) {
    if (dotQ(qUpper, qUpperPrevious) < 0.0f) qUpper = negateQ(qUpper);
    if (dotQ(qLower, qLowerPrevious) < 0.0f) qLower = negateQ(qLower);
  }

  qUpperPrevious = qUpper;
  qLowerPrevious = qLower;
  previousQValid = true;
  return true;
}

// ============================= KHOI TAO DMP =============================
bool initializeOneMPU(MPU6050 &mpu, const __FlashStringHelper *name) {
  Serial.print(F("# Checking "));
  Serial.println(name);

  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.print(F("# ERROR: Khong tim thay "));
    Serial.println(name);
    return false;
  }

  uint8_t status = mpu.dmpInitialize();
  if (status != 0) {
    Serial.print(F("# ERROR: DMP init failed for "));
    Serial.print(name);
    Serial.print(F(", code="));
    Serial.println(status);
    return false;
  }

  Serial.print(F("# Calibrating "));
  Serial.println(name);
  Serial.println(F("# Giu cam bien dung yen..."));

  mpu.CalibrateAccel(6);
  mpu.CalibrateGyro(6);
  mpu.setDMPEnabled(true);
  mpu.resetFIFO();

  Serial.print(F("# DMP OK: "));
  Serial.println(name);
  return true;
}

// ============================= ZERO POSE =============================
bool calibrateZeroPose() {
  Serial.println(F("# ZERO: Duoi thang tay, long ban tay up xuong, GIU YEN..."));
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

  while (count < ZERO_SAMPLES && millis() - startMs < 5000UL) {
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
    Serial.println(F("# ERROR: Khong du du lieu DMP de ZERO."));
    return false;
  }

  qUpperZero = normalizeQ(upperSum);
  qLowerZero = normalizeQ(lowerSum);
  qRelativeZero = calculateRelativeOrientation(qUpperZero, qLowerZero);

  qUpperPrevious = qUpperZero;
  qLowerPrevious = qLowerZero;
  previousQValid = true;

  resetAngleFilter();

  Serial.print(F("# ZERO OK, samples="));
  Serial.println(count);
  Serial.println(F("# New elbow angle = angle between longitudinal +X arm axes."));
  Serial.println(F("# elbow_axisY_old_deg is only for comparison."));
  Serial.println(F("time_ms,upper_dX_deg,upper_dY_deg,upper_dZ_deg,lower_dX_deg,lower_dY_deg,lower_dZ_deg,rel_dX_deg,rel_dY_deg,rel_dZ_deg,elbow_axisY_old_deg,elbow_vector_raw_deg,elbow_vector_median_deg,elbow_vector_filtered_deg"));
  return true;
}

// ============================= SETUP =============================
void setup() {
  Serial.begin(SERIAL_BAUD);
  delay(1000);

  Serial.println(F("# TWO MPU6050 - VECTOR ELBOW FLEXION"));
  Serial.println(F("# Upper=0x68, Lower=0x69, +X points toward fingers."));
  Serial.println(F("# I2C=100kHz for better stability on wearable wiring."));

  Wire.begin(); // Nano/Uno: SDA=A4, SCL=A5
  Wire.setClock(I2C_CLOCK_HZ);

  bool upperOK = initializeOneMPU(mpuUpper, F("Upper MPU 0x68"));
  bool lowerOK = initializeOneMPU(mpuLower, F("Lower MPU 0x69"));

  dmpReady = upperOK && lowerOK;
  if (!dmpReady) {
    Serial.println(F("# STOP: Khoi tao MPU that bai."));
    return;
  }

  Serial.println(F("# Sau 3 giay he thong se lay tu the hien tai lam 0 do."));
  delay(3000);
  calibrateZeroPose();
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

  // Goc rieng tung MPU so voi tu the ZERO: giu lai de kiem tra.
  Q upperDelta = multiplyOrientationQ(conjugateQ(qUpperZero), qUpper);
  Q lowerDelta = multiplyOrientationQ(conjugateQ(qLowerZero), qLower);

  float upperDX = twistAngleDeg(upperDelta, 'X');
  float upperDY = twistAngleDeg(upperDelta, 'Y');
  float upperDZ = twistAngleDeg(upperDelta, 'Z');

  float lowerDX = twistAngleDeg(lowerDelta, 'X');
  float lowerDY = twistAngleDeg(lowerDelta, 'Y');
  float lowerDZ = twistAngleDeg(lowerDelta, 'Z');

  // Chuyen dong tuong doi cang tay so voi bap tay.
  Q elbowDelta = calculateElbowDelta(qUpper, qLower);

  float relDX = twistAngleDeg(elbowDelta, 'X');
  float relDY = twistAngleDeg(elbowDelta, 'Y');
  float relDZ = twistAngleDeg(elbowDelta, 'Z');

  // Cong thuc cu giu lai de doi chieu.
  float elbowAxisYOld = fabsf(relDY);

  // Cong thuc moi + loc.
  float elbowRaw = calculateElbowVectorAngleDeg(elbowDelta);
  float elbowMedian = 0.0f;
  float elbowFiltered = 0.0f;
  updateAngleFilter(elbowRaw, elbowMedian, elbowFiltered);

  Serial.print(now);
  Serial.print(',');
  Serial.print(upperDX, 2);
  Serial.print(',');
  Serial.print(upperDY, 2);
  Serial.print(',');
  Serial.print(upperDZ, 2);
  Serial.print(',');
  Serial.print(lowerDX, 2);
  Serial.print(',');
  Serial.print(lowerDY, 2);
  Serial.print(',');
  Serial.print(lowerDZ, 2);
  Serial.print(',');
  Serial.print(relDX, 2);
  Serial.print(',');
  Serial.print(relDY, 2);
  Serial.print(',');
  Serial.print(relDZ, 2);
  Serial.print(',');
  Serial.print(elbowAxisYOld, 2);
  Serial.print(',');
  Serial.print(elbowRaw, 2);
  Serial.print(',');
  Serial.print(elbowMedian, 2);
  Serial.print(',');
  Serial.println(elbowFiltered, 2);
}
