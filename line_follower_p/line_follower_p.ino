/*
 * P-only line follower for the TI-RSLK (MSP-EXP432P401R) using the ECE3 library.
 * Baseline version of line_follower.ino: same sensors, calibration, error function,
 * start and stop logic, but steering is just u = KP * error at a constant base speed.
 *
 * Operation:
 *   1. Power on -> WAIT (red LED blinks, motors off). Uses the fixed calibration below.
 *   2. Press any front bumper -> ARMED (green LED). Car waits ARM_DELAY_MS so your
 *      hand is clear, then starts TRACKING.
 *   3. Car stops when it sees the solid black crossbar (or if a bumper is hit).
 *
 * Sign convention: error > 0 means the line is to the RIGHT of center, and the car
 * turns right (left wheel faster).
 */

#include <ECE3.h>

// 1 = print sensor readings / error over Serial (9600). While waiting for the bumper
// the motors are off, so you can slide the car over the line and check the error sign.
#define DEBUG 0

// ---------------- Tunables ----------------
const int   BASE_SPEED   = 50;     // PWM (0-255), constant
const float KP           = 0.015;  // per unit error (error range is -3500..3500)
const int   ERROR_SIGN   = 1;      // flip to -1 if the car steers away from the line

const unsigned long LOOP_PERIOD_US = 5000;  // control loop runs every 5 ms
const unsigned long ARM_DELAY_MS   = 1000;
const unsigned long END_IGNORE_MS  = 300;   // don't look for the crossbar right at start

const int LOST_THRESHOLD   = 200;  // all sensors below this (0-1000) = line lost
const int BLACK_THRESHOLD  = 700;  // sensor above this counts as black for crossbar
const int CROSSBAR_SENSORS = 7;    // this many black sensors = crossbar
const int CROSSBAR_SAMPLES = 3;    // for this many consecutive samples

// ---------------- Fixed calibration (raw IR values, sensor 0 = rightmost) ----------------
const uint16_t minVal[8] = {978, 680, 794, 657, 702, 711, 862, 885};
const uint16_t maxVal[8] = {2500, 2500, 2488, 2139, 2046, 2500, 2500, 2500};

// Sensor 0 (P7.0) is on the right side of the RSLK, sensor 7 (P7.7) on the left,
// so the weights go from +3500 (right) to -3500 (left).
const int WEIGHTS[8] = {3500, 2500, 1500, 500, -500, -1500, -2500, -3500};

// ---------------- Pins ----------------
const int LEFT_NSLP  = 31;  // P3.7
const int LEFT_DIR   = 29;  // P5.4
const int LEFT_PWM   = 40;  // P2.7
const int RIGHT_NSLP = 11;  // P3.6
const int RIGHT_DIR  = 30;  // P5.5
const int RIGHT_PWM  = 39;  // P2.6

const int BUMPERS[6] = {24, 25, 6, 27, 8, 28};  // P4.0, P4.2, P4.3, P4.5, P4.6, P4.7 (active LOW)

// ---------------- State ----------------
enum Mode { WAIT, ARMED, TRACKING, STOPPED };
Mode mode = WAIT;

uint16_t raw[8];
int norm[8];

// Filled in by readLine()
int maxNorm = 0;
int blackCount = 0;

float lastValidError = 0;
int crossbarCount = 0;
unsigned long lastLoopUs = 0;
unsigned long trackStartMs = 0;

// ---------------- Helpers ----------------
void setMotor(int dirPin, int pwmPin, int speed) {
  speed = constrain(speed, -255, 255);
  digitalWrite(dirPin, speed >= 0 ? LOW : HIGH);  // LOW = forward
  analogWrite(pwmPin, abs(speed));
}

void drive(int left, int right) {
  setMotor(LEFT_DIR, LEFT_PWM, left);
  setMotor(RIGHT_DIR, RIGHT_PWM, right);
}

bool anyBumperPressed() {
  for (int i = 0; i < 6; i++) {
    if (digitalRead(BUMPERS[i]) == LOW) return true;
  }
  return false;
}

void waitForBumperRelease() {
  while (anyBumperPressed()) delay(10);
  delay(50);  // debounce
}

// Read the sensors, scale each to 0 (white) .. 1000 (black) using the fixed calibration,
// and return the line position error. Returns ±3500 toward the last seen side if the
// line is lost.
float readLine() {
  ECE3_read_IR(raw);

  long weightedSum = 0;
  long total = 0;
  maxNorm = 0;
  blackCount = 0;
  for (int i = 0; i < 8; i++) {
    long range = (long)maxVal[i] - minVal[i];
    long v = ((long)raw[i] - minVal[i]) * 1000 / range;
    norm[i] = constrain(v, 0, 1000);

    weightedSum += (long)WEIGHTS[i] * norm[i];
    total += norm[i];
    if (norm[i] > maxNorm) maxNorm = norm[i];
    if (norm[i] > BLACK_THRESHOLD) blackCount++;
  }

  if (maxNorm < LOST_THRESHOLD) {
    // Line slid off the edge of the array: turn hard toward where we last saw it
    return (lastValidError >= 0) ? 3500 : -3500;
  }
  lastValidError = ERROR_SIGN * (float)weightedSum / total;
  return lastValidError;
}

void stopCar() {
  drive(0, 0);
  mode = STOPPED;
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, HIGH);
}

void printSensors(float error) {
  for (int i = 7; i >= 0; i--) {  // printed left -> right as seen from above
    Serial.print(norm[i]);
    Serial.print('\t');
  }
  Serial.print("err=");
  Serial.println(error);
}

// ---------------- Modes ----------------
void waitStep() {
  digitalWrite(RED_LED, (millis() / 250) % 2);

#if DEBUG
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    lastPrint = millis();
    printSensors(readLine());
  }
#endif

  if (anyBumperPressed()) {
    waitForBumperRelease();
    digitalWrite(RED_LED, LOW);
    digitalWrite(GREEN_LED, HIGH);
    mode = ARMED;
  }
}

void armedStep() {
  delay(ARM_DELAY_MS);
  digitalWrite(GREEN_LED, LOW);
  lastValidError = 0;
  crossbarCount = 0;
  trackStartMs = millis();
  lastLoopUs = micros();
  mode = TRACKING;
}

void trackingStep() {
  unsigned long now = micros();
  if (now - lastLoopUs < LOOP_PERIOD_US) return;
  lastLoopUs = now;

  if (anyBumperPressed()) {  // safety stop
    stopCar();
    return;
  }

  float error = readLine();

  // End-of-track crossbar
  if (millis() - trackStartMs > END_IGNORE_MS) {
    crossbarCount = (blackCount >= CROSSBAR_SENSORS) ? crossbarCount + 1 : 0;
    if (crossbarCount >= CROSSBAR_SAMPLES) {
      stopCar();
      return;
    }
  }

  float u = KP * error;

  int left  = BASE_SPEED + (int)u;  // line to the right (error > 0) -> left wheel faster
  int right = BASE_SPEED - (int)u;
  drive(left, right);

#if DEBUG
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print("err="); Serial.print(error);
    Serial.print(" u="); Serial.print(u);
    Serial.print(" L="); Serial.print(left);
    Serial.print(" R="); Serial.println(right);
  }
#endif
}

void setup() {
  ECE3_Init();
  Serial.begin(9600);

  pinMode(LEFT_NSLP, OUTPUT);
  pinMode(LEFT_DIR, OUTPUT);
  pinMode(LEFT_PWM, OUTPUT);
  pinMode(RIGHT_NSLP, OUTPUT);
  pinMode(RIGHT_DIR, OUTPUT);
  pinMode(RIGHT_PWM, OUTPUT);
  digitalWrite(LEFT_NSLP, HIGH);
  digitalWrite(RIGHT_NSLP, HIGH);
  drive(0, 0);

  for (int i = 0; i < 6; i++) pinMode(BUMPERS[i], INPUT_PULLUP);

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  Serial.println("Ready: press a bumper to start.");
}

void loop() {
  switch (mode) {
    case WAIT:     waitStep();     break;
    case ARMED:    armedStep();    break;
    case TRACKING: trackingStep(); break;
    case STOPPED:  break;
  }
}
