/*
 * Split-tuning PD line follower for the TI-RSLK (MSP-EXP432P401R) using the ECE3 library.
 * Same PD controller, slowdown, 180 turn and stop logic as line_follower.ino, but the
 * track is split into two halves and each half has its own base speed, penalty and gains.
 *
 *   start bar ---- HALF1 ---- | ---- HALF2 ---- end bar
 *
 * The switch is timed from the start of each leg (trackStartMs):
 *   outbound: HALF1 until SPLIT_OUT_MS, then HALF2 up to the end bar
 *   return:   HALF2 until the return split after the turn, then HALF1 back to the start bar
 * So each half of the track always uses its own tuning, whichever way the car drives it.
 *
 * Return split: with AUTO_SPLIT_RET = 1 it's worked out at the end bar as
 *   (time spent on HALF2 on the way out) - RET_SPLIT_EARLY_MS
 * since the car drives the same HALF2 stretch back with the same tuning. With
 * AUTO_SPLIT_RET = 0 it uses SPLIT_RET_MS.
 *
 * Return leg on HALF1: uses the RET_HALF1 tuning instead of HALF1, and the slowdown
 * kicks in at RET_HALF1_PENALTY_ATTACK (1.0 = at once) instead of PENALTY_ATTACK, so the
 * car slows and turns tighter right away in the folds.
 *
 * Setting the split times: run once and read Serial after it stops. It prints how long
 * each leg took; start with each split at about half of that leg's time.
 *
 * Bar detection: sensors are read on every pass of loop() (~2.6 ms) so a bar isn't
 * missed at speed. A bar = CROSSBAR_SENSORS sensors darker than BAR_THRESHOLD within the
 * last BAR_WINDOW_MS (they don't all have to see it on the same read). It's only looked
 * for BAR_ARM_OUT_MS / BAR_ARM_RET_MS into each leg; if a sharp turn sets it off, raise
 * those to a bit less than the leg times. After stopping, Serial prints the most sensors
 * that saw the bar on each leg, to tune CROSSBAR_SENSORS.
 *
 * Operation:
 *   1. Power on -> WAIT (red LED blinks, motors off). Uses the fixed calibration below.
 *   2. Exactly START_DELAY_MS (2 s) after power-on / reset it starts TRACKING.
 *      No bumper press needed.
 *   3. At the first crossbar (end of track) it spins 180 deg and follows the line back.
 *   4. At the next crossbar (start of track) it stops.
 *
 * LEDs (to tell why it stopped):
 *   red LED1 (P1.0) blinking fast  - tracking loop is running (heartbeat)
 *   RGB red solid                  - using HALF2 tuning (off = HALF1), to check the split
 *   RGB blue solid                 - spinning 180 or on the return leg (red + blue = magenta)
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

// ---------------- Per-half tuning ----------------
struct Tuning {
  float baseSpeed;    // PWM (0-255) on a straight
  float penaltyCoef;  // base speed drops by this * |P term| as the error grows
  float kp;           // per unit error (error range is -3500..3500)
  float kd;           // per unit change in error per sample (0 = P-only)
};

// Both start as the line_follower.ino tuning: base = 100 + PENALTY_COEF * 50.
//                       baseSpeed          penaltyCoef  kp    kd
const Tuning HALF1 = { 80 + 1.8 * 50,      1.8,         0.032, 0.15 };  // start bar -> middle
const Tuning HALF2 = { 140 + 1.6 * 50,      1.6,         0.04, 0.3 };  // middle -> end bar
// Return leg only: used instead of HALF1 once the return switches back (middle -> start bar)
const Tuning RET_HALF1 = { 60 + 1.8 * 50,  1.8,         0.05, 0.15 };

// ms from the start of each leg to the middle of the track
const unsigned long SPLIT_OUT_MS = 5800;  // outbound: switch HALF1 -> HALF2
const unsigned long SPLIT_RET_MS = 8000;  // return (timed from the end of the 180): HALF2 -> HALF1
                                          // only used when AUTO_SPLIT_RET = 0
#define AUTO_SPLIT_RET 1                  // 1 = return split from the outbound HALF2 time
const unsigned long RET_SPLIT_EARLY_MS = 200;  // switch back to HALF1 this much earlier than
                                               // the outbound HALF2 time (safety margin)


// ---------------- Shared tunables ----------------
const float PENALTY_ATTACK  = 0.7;   // penalty rising (slowing down), ~20 ms
// Return leg, HALF1 only: how fast the slowdown kicks in. Fraction of the way to the
// target per 5 ms step: 1.0 = full slowdown on the very next step (the max). Above 1
// it overshoots and the slowdown flips between none and full every step.
const float RET_HALF1_PENALTY_ATTACK = 1.0;
const float PENALTY_RELEASE = 0.03;  // penalty falling (speeding up), ~165 ms
const float MIN_BASE_SPEED  = 40;    // slowdown never takes the base below this, so the
                                     // outer wheel always drives forward (no backing up)
const float D_ALPHA      = 0.5;    // EMA weight for the derivative (1 = no filtering)
const int   ERROR_SIGN   = 1;      // flip to -1 if the car steers away from the line

const unsigned long LOOP_PERIOD_US = 5000;  // control loop runs every 5 ms
const unsigned long START_DELAY_MS = 2000;  // starts this long after power-on (millis() = 0)
// ---------------- Bar detection ----------------
// More sensitive: lower CROSSBAR_SENSORS or BAR_THRESHOLD, or raise BAR_WINDOW_MS.
// If a sharp turn sets it off, go the other way or raise the BAR_ARM times.
const int CROSSBAR_SENSORS = 6;    // this many sensors on the bar = bar
const int BAR_THRESHOLD    = 450;  // sensor above this (0-1000) counts as on the bar
const unsigned long BAR_WINDOW_MS = 12;  // a sensor counts if it was on the bar this recently
const int CROSSBAR_SAMPLES = 2;    // for this many consecutive reads (~2.6 ms each)
const unsigned long BAR_ARM_OUT_MS = 10000;  // outbound: don't look for the bar before this
const unsigned long BAR_ARM_RET_MS = 12000;  // return (from the end of the 180): same
// Backup: line gone this long after the BAR_ARM time = ran past the bar (0 = off).
// Leave off until the BAR_ARM times are set past the sharp turns, or a turn that
// loses the line will count as the end.
const unsigned long END_LOST_MS = 0;

const int LOST_THRESHOLD   = 200;  // all sensors below this (0-1000) = line lost
const int BLACK_THRESHOLD  = 700;  // sensor above this counts as black (spin only)

const int  TURN_SPEED      = 120;   // PWM for each wheel during the 180 deg spin
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
int barCount = 0;                // sensors on the bar within the last BAR_WINDOW_MS
unsigned long barSeenMs[8];      // last time each sensor read above BAR_THRESHOLD

bool lineLost = false;
unsigned long lostSinceMs = 0;

int peakBar[2] = {0, 0};             // most sensors on the bar at once, after BAR_ARM (0 = out, 1 = return)
const char *legEnd[2] = {"-", "-"};  // what ended each leg

float prevError = 0;
float lastValidError = 0;
float dFilt = 0;
float penalty = 0;  // smoothed base speed reduction
int crossbarCount = 0;
unsigned long lastLoopUs = 0;
unsigned long trackStartMs = 0;
bool returning = false;  // false = heading to the end bar, true = heading back to the start
unsigned long turnStartMs = 0;
unsigned long outLegMs = 0;  // how long the outbound leg took (printed after stopping)
unsigned long splitRetMs = SPLIT_RET_MS;  // return split actually used (see AUTO_SPLIT_RET)


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
  barCount = 0;
  unsigned long nowMs = millis();
  for (int i = 0; i < 8; i++) {
    long range = (long)maxVal[i] - minVal[i];
    long v = ((long)raw[i] - minVal[i]) * 1000 / range;
    norm[i] = constrain(v, 0, 1000);

    weightedSum += (long)WEIGHTS[i] * norm[i];
    total += norm[i];
    if (norm[i] > maxNorm) maxNorm = norm[i];
    if (norm[i] > BLACK_THRESHOLD) blackCount++;
    if (norm[i] > BAR_THRESHOLD) barSeenMs[i] = nowMs;
    if (nowMs - barSeenMs[i] <= BAR_WINDOW_MS) barCount++;
  }

  if (maxNorm < LOST_THRESHOLD) {
    // Line slid off the edge of the array: turn hard toward where we last saw it
    return (lastValidError >= 0) ? 3500 : -3500;
  }
  lastValidError = ERROR_SIGN * (float)weightedSum / total;
  return lastValidError;
}

// True while the car is on the second half of the track (middle <-> end bar).
bool onHalf2() {
  unsigned long legMs = millis() - trackStartMs;
  if (!returning) return legMs >= SPLIT_OUT_MS;  // outbound: HALF1 first
  return legMs < splitRetMs;                     // return: HALF2 first
}

// Forget anything the bar detector saw before this leg (start area, the spin)
void resetBarDetect() {
  for (int i = 0; i < 8; i++) barSeenMs[i] = millis() - 1000;
  lineLost = false;
  crossbarCount = 0;
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

  // Start a fixed time after power-on. millis() counts from reset, so the setup() time
  // is already included and the start is the same every run.
  if (millis() >= START_DELAY_MS) {
    digitalWrite(RED_LED, LOW);
    mode = ARMED;
  }
}

void armedStep() {
  lastValidError = 0;
  resetBarDetect();
  prevError = readLine();  // seed so the first D term doesn't spike
  dFilt = 0;
  penalty = 0;
  trackStartMs = millis();
  lastLoopUs = micros();
  mode = TRACKING;
}

// Hit the end bar: settle, then spin left in place.
void startTurn() {
  drive(0, 0);
  outLegMs = millis() - trackStartMs;
#if AUTO_SPLIT_RET
  // Time spent on HALF2 on the way out = time the return should spend on it
  if (outLegMs > SPLIT_OUT_MS + RET_SPLIT_EARLY_MS) {
    splitRetMs = outLegMs - SPLIT_OUT_MS - RET_SPLIT_EARLY_MS;
  } else {
    splitRetMs = 0;  // never reached HALF2 on the way out: use HALF1 the whole way back
  }
#endif
  digitalWrite(RED_LED, LOW);
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
  resetBarDetect();
  prevError = readLine();  // seed so the first D term doesn't spike
  dFilt = 0;
  penalty = 0;
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
  // Read the sensors every pass (~2.6 ms) so a bar isn't skipped at speed
  float error = readLine();
  unsigned long ms = millis() - trackStartMs;
  int leg = returning ? 1 : 0;

  if (maxNorm < LOST_THRESHOLD) {
    if (!lineLost) { lineLost = true; lostSinceMs = millis(); }
  } else {
    lineLost = false;
  }

  // Crossbar: spin around at the end of the track, stop when back at the start
  if (ms > (returning ? BAR_ARM_RET_MS : BAR_ARM_OUT_MS)) {
    if (barCount > peakBar[leg]) peakBar[leg] = barCount;
    crossbarCount = (barCount >= CROSSBAR_SENSORS) ? crossbarCount + 1 : 0;
    bool barHit = crossbarCount >= CROSSBAR_SAMPLES;
    bool lineEnded = END_LOST_MS > 0 && lineLost && millis() - lostSinceMs >= END_LOST_MS;
    if (barHit || lineEnded) {
      legEnd[leg] = barHit ? "bar" : "line lost (missed the bar)";
      if (returning) stopCar(STOP_CROSSBAR);
      else startTurn();
      return;
    }
  }

  // PD update every LOOP_PERIOD_US, using the latest reading
  unsigned long now = micros();
  if (now - lastLoopUs < LOOP_PERIOD_US) return;
  lastLoopUs = now;
  digitalWrite(LED1, (millis() / 100) % 2);  // heartbeat: blinking = loop still running

  // Pick this half's tuning
  bool half2 = onHalf2();
  const Tuning &t = half2 ? HALF2 : (returning ? RET_HALF1 : HALF1);
  digitalWrite(RED_LED, half2 ? HIGH : LOW);


  // PD with a lightly filtered derivative (P sees the raw error, no added lag)
  dFilt = D_ALPHA * (error - prevError) + (1 - D_ALPHA) * dFilt;
  prevError = error;
  float uP = t.kp * error;
  float uD = t.kd * dFilt;
  float u = uP + uD;

  // Slow down as the error grows. Uses the P term only so D noise can't jitter the speed.
  // Rises quickly but falls slowly, so the speed doesn't jump each time the error
  // crosses zero between curves.
  float target = t.penaltyCoef * fabs(uP);
  float attack = (returning && !half2) ? RET_HALF1_PENALTY_ATTACK : PENALTY_ATTACK;
  penalty += (target > penalty ? attack : PENALTY_RELEASE) * (target - penalty);
  // Without this, penaltyCoef * 105 (the max P term) can exceed baseSpeed: then once D
  // cancels P coming into a sharp turn, both wheels go negative and the car backs up.
  penalty = constrain(penalty, 0, t.baseSpeed - MIN_BASE_SPEED);

  int left  = (int)(t.baseSpeed - penalty + u);  // error > 0 -> left wheel faster
  int right = (int)(t.baseSpeed - penalty - u);
  drive(left, right);

#if DEBUG
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print(half2 ? "H2 " : "H1 ");
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
  Serial.print("STOPPED (need ");
  Serial.print(CROSSBAR_SENSORS);
  Serial.println(" sensors on the bar)");
  for (int i = 0; i < 2; i++) {
    Serial.print(i == 0 ? "  outbound" : "  return  ");
    Serial.print(": ended on ");
    Serial.print(legEnd[i]);
    Serial.print(", most sensors on bar = ");
    Serial.println(peakBar[i]);
  }
  Serial.print("  outbound leg took ");
  Serial.print(outLegMs);
  Serial.print(" ms (SPLIT_OUT_MS=");
  Serial.print(SPLIT_OUT_MS);
  Serial.print("), return leg took ");
  Serial.print(stopMs - trackStartMs);
  Serial.print(" ms (return split used = ");
  Serial.print(splitRetMs);
  Serial.println(" ms)");
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

  Serial.println("Ready: starting 2 s after power-on.");
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
