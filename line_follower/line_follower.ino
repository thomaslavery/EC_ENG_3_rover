/*
 * PD line follower for the TI-RSLK (MSP-EXP432P401R) using the ECE3 library.
 * Built on line_follower_p.ino: same gains, slowdown in turns, 180 turn and stop logic.
 * Steering is u = KP * error + KD * (filtered change in error). With KD = 0 it drives
 * the same as the P-only sketch.
 *
 * Operation:
 *   1. Power on -> WAIT (red LED blinks, motors off). Uses the fixed calibration below.
 *   2. Press any front bumper -> ARMED (green LED). Car waits ARM_DELAY_MS so your
 *      hand is clear, then starts TRACKING.
 *   3. At the first crossbar (end of track) it spins 180 deg and follows the line back.
 *   4. At the next crossbar (start of track) it stops.
 *
 * LEDs (to tell why it stopped):
 *   red LED1 (P1.0) blinking fast  - tracking loop is running (heartbeat)
 *   RGB blue solid                 - spinning 180 or on the return leg
 *   RGB green solid                - stopped: saw the second crossbar
 *   RGB red blinking               - back in WAIT, i.e. the board reset (brownout?)
 *   heartbeat frozen, no green     - the code hung
 * Once stopped, Serial (9600) prints the reason and the last sensor reading every 1 s.
 *
 * Sign convention: error > 0 means the line is to the RIGHT of center, and the car
 * turns right (left wheel faster).
 */

#include <ECE3.h>

// 1 = print sensor readings / error over Serial (9600). While waiting for the bumper
// the motors are off, so you can slide the car over the line and check the error sign.
#define DEBUG 0

// ---------------- Tunables ----------------
const int   BASE_PENALTY = 50;
const float PENALTY_COEF = 1.6;      // base speed drops by this * |P term| as the error grows
const int   BASE_SPEED   = 100 + PENALTY_COEF * BASE_PENALTY;     // PWM (0-255) on a straight
const float PENALTY_ATTACK  = 0.7;   // penalty rising (slowing down), ~20 ms
const float PENALTY_RELEASE = 0.03;  // penalty falling (speeding up), ~165 ms
const float KP           = 0.03;  // per unit error (error range is -3500..3500)
const float KD           = 0.2;    // per unit change in error per sample (0 = P-only)
const float D_ALPHA      = 0.5;    // EMA weight for the derivative (1 = no filtering)
const int   ERROR_SIGN   = 1;      // flip to -1 if the car steers away from the line

const unsigned long LOOP_PERIOD_US = 5000;  // control loop runs every 5 ms
const unsigned long ARM_DELAY_MS   = 1000;
const unsigned long END_IGNORE_MS  = 300;   // don't look for the crossbar right at start

const int LOST_THRESHOLD   = 200;  // all sensors below this (0-1000) = line lost
const int BLACK_THRESHOLD  = 700;  // sensor above this counts as black for crossbar
const int CROSSBAR_SENSORS = 7;    // this many black sensors = crossbar
const int CROSSBAR_SAMPLES = 3;    // for this many consecutive samples

const int  TURN_SPEED      = 70;   // PWM for each wheel during the 180 deg spin
const long TURN_MIN_COUNTS = 300;  // encoder counts per wheel before looking for the line (~150 deg)
const long TURN_MAX_COUNTS = 450;  // stop looking and go back to tracking (~225 deg)
const unsigned long TURN_PAUSE_MS   = 0;   // settle before and after the spin
const unsigned long TURN_TIMEOUT_MS = 4000;  // in case the wheels stall or an encoder is dead

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
enum Mode { WAIT, ARMED, TRACKING, TURNING, STOPPED };
Mode mode = WAIT;

uint16_t raw[8];
int norm[8];

// Filled in by readLine()
int maxNorm = 0;
int blackCount = 0;

float prevError = 0;
float lastValidError = 0;
float dFilt = 0;
float penalty = 0;  // smoothed base speed reduction
int crossbarCount = 0;
unsigned long lastLoopUs = 0;
unsigned long trackStartMs = 0;
bool returning = false;  // false = heading to the end bar, true = heading back to the start
unsigned long turnStartMs = 0;

enum StopReason { STOP_NONE, STOP_CROSSBAR };
StopReason stopReason = STOP_NONE;
unsigned long stopMs = 0;

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

void stopCar(StopReason reason) {
  drive(0, 0);
  mode = STOPPED;
  stopReason = reason;
  stopMs = millis();
  digitalWrite(LED1, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
  digitalWrite(GREEN_LED, reason == STOP_CROSSBAR ? HIGH : LOW);
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
  prevError = readLine();  // seed so the first D term doesn't spike
  dFilt = 0;
  penalty = 0;
  crossbarCount = 0;
  trackStartMs = millis();
  lastLoopUs = micros();
  mode = TRACKING;
}

// Hit the end bar: settle, then spin left in place.
void startTurn() {
  drive(0, 0);
  digitalWrite(BLUE_LED, HIGH);
  delay(TURN_PAUSE_MS);
  resetEncoderCount_left();
  resetEncoderCount_right();
  drive(-TURN_SPEED, TURN_SPEED);
  turnStartMs = millis();
  lastLoopUs = micros();
  mode = TURNING;
}

void finishTurn() {
  drive(0, 0);
  delay(TURN_PAUSE_MS);
  returning = true;
  lastValidError = -1;  // if the line isn't under us yet, keep turning left to find it
  prevError = readLine();  // seed so the first D term doesn't spike
  dFilt = 0;
  penalty = 0;
  crossbarCount = 0;
  trackStartMs = millis();
  lastLoopUs = micros();
  mode = TRACKING;
}

void turningStep() {
  unsigned long now = micros();
  if (now - lastLoopUs < LOOP_PERIOD_US) return;
  lastLoopUs = now;
  digitalWrite(LED1, (millis() / 100) % 2);  // heartbeat: blinking = loop still running

  readLine();
  long turned = (labs(getEncoderCount_left()) + labs(getEncoderCount_right())) / 2;
  bool lineCentered = norm[3] > BLACK_THRESHOLD || norm[4] > BLACK_THRESHOLD;

  if ((turned >= TURN_MIN_COUNTS && lineCentered) || turned >= TURN_MAX_COUNTS ||
      millis() - turnStartMs > TURN_TIMEOUT_MS) {
    finishTurn();
  }
}

void trackingStep() {
  unsigned long now = micros();
  if (now - lastLoopUs < LOOP_PERIOD_US) return;
  lastLoopUs = now;
  digitalWrite(LED1, (millis() / 100) % 2);  // heartbeat: blinking = loop still running

  float error = readLine();

  // Crossbar: spin around at the end of the track, stop when back at the start
  if (millis() - trackStartMs > END_IGNORE_MS) {
    crossbarCount = (blackCount >= CROSSBAR_SENSORS) ? crossbarCount + 1 : 0;
    if (crossbarCount >= CROSSBAR_SAMPLES) {
      if (returning) stopCar(STOP_CROSSBAR);
      else startTurn();
      return;
    }
  }

  // PD with a lightly filtered derivative (P sees the raw error, no added lag)
  dFilt = D_ALPHA * (error - prevError) + (1 - D_ALPHA) * dFilt;
  prevError = error;
  float uP = KP * error;
  float uD = KD * dFilt;
  float u = uP + uD;

  // Slow down as the error grows. Uses the P term only so D noise can't jitter the speed.
  // Rises quickly but falls slowly, so the speed doesn't jump each time the error
  // crosses zero between curves.
  float target = PENALTY_COEF * fabs(uP);
  penalty += (target > penalty ? PENALTY_ATTACK : PENALTY_RELEASE) * (target - penalty);

  int left  = BASE_SPEED - penalty + (int)u;  // error > 0 -> left wheel faster
  int right = BASE_SPEED - penalty - (int)u;
  drive(left, right);

#if DEBUG
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print("err="); Serial.print(error);
    Serial.print(" uP="); Serial.print(uP);
    Serial.print(" uD="); Serial.print(uD);
    Serial.print(" pen="); Serial.print(penalty);
    Serial.print(" L="); Serial.print(left);
    Serial.print(" R="); Serial.println(right);
  }
#endif
}

// Motors are off; keep reporting why we stopped and what the sensors saw last.
void stoppedStep() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 1000) return;
  lastPrint = millis();
  Serial.print("STOPPED: crossbar");
  Serial.print("  blackCount=");
  Serial.print(blackCount);
  Serial.print("  after ");
  Serial.print(stopMs - trackStartMs);
  Serial.println(" ms of the return leg");
  printSensors(lastValidError);
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
  pinMode(BLUE_LED, OUTPUT);
  pinMode(LED1, OUTPUT);

  Serial.println("Ready: press a bumper to start.");
}

void loop() {
  switch (mode) {
    case WAIT:     waitStep();     break;
    case ARMED:    armedStep();    break;
    case TRACKING: trackingStep(); break;
    case TURNING:  turningStep();  break;
    case STOPPED:  stoppedStep();  break;
  }
}
