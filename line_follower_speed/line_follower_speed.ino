/*
 * Speed-tuned PD line follower for a horseshoe track on the TI-RSLK (MSP-EXP432P401R),
 * using the ECE3 library.
 *
 *   start bar ===== straight =====\
 *                                  )  180 deg curve
 *   end bar   ===== straight =====/
 *
 * One PD tuning for the whole track. The speed comes from the error alone:
 *   - line centered (straights) -> small error -> no slowdown -> full BASE_SPEED
 *   - curve -> error grows -> penalty = PENALTY_COEF * |P term| slows it down
 * The penalty rises fast (PENALTY_ATTACK) so it slows as soon as the curve starts, and
 * falls slowly (PENALTY_RELEASE) so it doesn't jump back to full speed mid-curve. It never
 * takes the base below MIN_BASE_SPEED, so the outer wheel always drives forward.
 *
 * Also:
 *   - Base speed ramps up at BASE_ACCEL per 5 ms from LAUNCH_SPEED at the start of each
 *     leg, so the wheels don't slip.
 *   - Sensors are read on every pass of loop() (~2.6 ms) so a thin bar isn't missed at
 *     speed. The PD update still runs every 5 ms.
 *   - A bar = CROSSBAR_SENSORS (default 4) sensors darker than BAR_THRESHOLD within the
 *     last BAR_WINDOW_MS (they don't all have to see it on the same read). It's only
 *     looked for BAR_ARM_OUT_MS / BAR_ARM_RET_MS into each leg.
 *   - Short reverse pulse (BRAKE_PWM for BRAKE_MS) at each bar so it doesn't overshoot.
 *   - After stopping, Serial (9600) prints each leg's time, what ended it, and the most
 *     sensors that saw the bar, to tune the bar settings.
 *
 * Operation:
 *   1. Put the car on the track, power on -> WAIT (red LED blinks, motors off).
 *   2. Exactly START_DELAY_MS (2 s) after power-on / reset it starts TRACKING.
 *   3. At the first bar (end of track) it brakes, spins 180 deg and follows the line back.
 *   4. At the next bar (start of track) it brakes and stops.
 *
 * LEDs:
 *   red LED1 (P1.0) blinking fast  - tracking loop is running (heartbeat)
 *   RGB blue solid                 - spinning 180 or on the return leg
 *   RGB green solid                - stopped: saw the second bar
 *   RGB red blinking               - WAIT (before the start, or the board reset)
 *   heartbeat frozen, no green     - the code hung
 *
 * Sign convention: error > 0 means the line is to the RIGHT of center, and the car
 * turns right (left wheel faster).
 */

#include <ECE3.h>

// 1 = print the control values over Serial (9600) while driving. Printing slows the loop.
#define DEBUG 0

// ---------------- PD tuning ----------------
const float PENALTY_COEF = 1.6;    // base speed drops by this * |P term| as the error grows
const float BASE_SPEED   = 140 + PENALTY_COEF * 50;  // PWM (0-255) with the line centered
const float KP           = 0.04;   // per unit error (error range is -3500..3500)
const float KD           = 0.3;    // per unit change in error per 5 ms (0 = P-only)
const float D_ALPHA      = 0.5;    // EMA weight for the derivative (1 = no filtering)
const int   ERROR_SIGN   = 1;      // flip to -1 if the car steers away from the line

// ---------------- Error-driven slowdown ----------------
const float PENALTY_ATTACK  = 0.7;   // penalty rising (slowing down), ~20 ms (1.0 = at once)
const float PENALTY_RELEASE = 0.03;  // penalty falling (speeding up), ~165 ms
const float MIN_BASE_SPEED  = 40;    // slowdown never takes the base below this, so the
                                     // outer wheel always drives forward (no backing up)

// ---------------- Launch ----------------
const float LAUNCH_SPEED = 100;    // base speed at the start of each leg (ramps up from here)
const float BASE_ACCEL   = 4;      // max base speed increase per 5 ms step

// ---------------- End bar ----------------
// More sensitive: lower CROSSBAR_SENSORS or BAR_THRESHOLD, or raise BAR_WINDOW_MS.
// If it stops or turns early (e.g. in the curve), go the other way or raise BAR_ARM.
const int CROSSBAR_SENSORS = 4;    // this many sensors on the bar = bar
const int BAR_THRESHOLD    = 450;  // sensor above this (0-1000) counts as on the bar
const unsigned long BAR_WINDOW_MS = 12;  // a sensor counts if it was on the bar this recently
const int CROSSBAR_SAMPLES = 1;    // for this many consecutive reads (~2.6 ms each)
const unsigned long BAR_ARM_OUT_MS = 300;  // outbound: don't look for the bar before this
const unsigned long BAR_ARM_RET_MS = 300;  // return (from the end of the 180): same
// Backup: line gone this long after the BAR_ARM time = ran past the bar (0 = off).
// Leave off unless the BAR_ARM times are past the curve, or losing the line in the
// curve will count as the end.
const unsigned long END_LOST_MS = 0;

const int BRAKE_PWM = 150;                // reverse PWM for the brake pulse (0 = coast)
const unsigned long BRAKE_MS = 60;        // brake pulse length

// ---------------- Timing / misc ----------------
const unsigned long LOOP_PERIOD_US = 5000;  // PD update every 5 ms
const unsigned long START_DELAY_MS = 2000;  // starts this long after power-on (millis() = 0)

const int LOST_THRESHOLD  = 200;   // all sensors below this (0-1000) = line lost
const int BLACK_THRESHOLD = 700;   // sensor above this counts as black (spin only)

const int  TURN_SPEED      = 100;  // PWM for each wheel during the 180 deg spin
const long TURN_MIN_COUNTS = 300;  // encoder counts per wheel before looking for the line (~150 deg)
const long TURN_MAX_COUNTS = 450;  // stop looking and go back to tracking (~225 deg)
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

// ---------------- State ----------------
enum Mode { WAIT, TRACKING, TURNING, STOPPED };
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

float prevError = 0;
float lastValidError = 0;
float dFilt = 0;
float penalty = 0;   // smoothed base speed reduction (the error "momentum")
float baseCmd = 0;   // rate-limited base speed
int crossbarCount = 0;
unsigned long lastLoopUs = 0;
unsigned long trackStartMs = 0;
bool returning = false;  // false = heading to the end bar, true = heading back to the start
unsigned long turnStartMs = 0;

// Per-leg log, printed after stopping (index 0 = outbound, 1 = return)
unsigned long legMs[2] = {0, 0};
int peakBar[2] = {0, 0};               // most sensors on the bar at once, after BAR_ARM
const char *legEnd[2] = {"-", "-"};    // what ended each leg

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

// Short reverse pulse to kill forward speed, then motors off
void brake() {
  if (BRAKE_PWM > 0 && BRAKE_MS > 0) {
    drive(-BRAKE_PWM, -BRAKE_PWM);
    delay(BRAKE_MS);
  }
  drive(0, 0);
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

void startLeg() {
  // Forget anything the bar detector saw during the spin or before the start
  for (int i = 0; i < 8; i++) barSeenMs[i] = millis() - 1000;
  lineLost = false;
  crossbarCount = 0;
  prevError = readLine();  // seed so the first D term doesn't spike
  dFilt = 0;
  penalty = 0;
  baseCmd = LAUNCH_SPEED;
  trackStartMs = millis();
  lastLoopUs = micros();
  mode = TRACKING;
}

void stopCar() {
  brake();
  mode = STOPPED;
  legMs[1] = millis() - trackStartMs;
  digitalWrite(LED1, LOW);
  digitalWrite(RED_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
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

void printLeg(const char *name, int i) {
  Serial.print("  ");
  Serial.print(name);
  Serial.print(": took ");
  Serial.print(legMs[i]);
  Serial.print(" ms, ended on ");
  Serial.print(legEnd[i]);
  Serial.print(", most sensors on bar = ");
  Serial.println(peakBar[i]);
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
    returning = false;
    lastValidError = 0;
    startLeg();
  }
}

// Hit the end bar: brake, then spin left in place.
void startTurn() {
  brake();
  legMs[0] = millis() - trackStartMs;
  digitalWrite(BLUE_LED, HIGH);
  resetEncoderCount_left();
  resetEncoderCount_right();
  drive(-TURN_SPEED, TURN_SPEED);
  turnStartMs = millis();
  lastLoopUs = micros();
  mode = TURNING;
}

void finishTurn() {
  drive(0, 0);
  returning = true;
  lastValidError = -1;  // if the line isn't under us yet, keep turning left to find it
  startLeg();
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
  // Read the sensors every pass (~2.6 ms) so a thin bar isn't skipped at speed
  float error = readLine();
  unsigned long ms = millis() - trackStartMs;
  int leg = returning ? 1 : 0;

  if (maxNorm < LOST_THRESHOLD) {
    if (!lineLost) { lineLost = true; lostSinceMs = millis(); }
  } else {
    lineLost = false;
  }

  // End bar: spin around at the end of the track, stop when back at the start
  if (ms > (returning ? BAR_ARM_RET_MS : BAR_ARM_OUT_MS)) {
    if (barCount > peakBar[leg]) peakBar[leg] = barCount;
    crossbarCount = (barCount >= CROSSBAR_SENSORS) ? crossbarCount + 1 : 0;
    bool barHit = crossbarCount >= CROSSBAR_SAMPLES;
    bool lineEnded = END_LOST_MS > 0 && lineLost && millis() - lostSinceMs >= END_LOST_MS;
    if (barHit || lineEnded) {
      legEnd[leg] = barHit ? "bar" : "line lost (missed the bar)";
      if (returning) stopCar();
      else startTurn();
      return;
    }
  }

  // PD update every LOOP_PERIOD_US, using the latest reading
  unsigned long now = micros();
  if (now - lastLoopUs < LOOP_PERIOD_US) return;
  lastLoopUs = now;
  digitalWrite(LED1, (millis() / 100) % 2);  // heartbeat: blinking = loop still running

  // Launch ramp
  baseCmd = min(BASE_SPEED, baseCmd + BASE_ACCEL);

  // PD with a lightly filtered derivative (P sees the raw error, no added lag)
  dFilt = D_ALPHA * (error - prevError) + (1 - D_ALPHA) * dFilt;
  prevError = error;
  float uP = KP * error;
  float uD = KD * dFilt;
  float u = uP + uD;

  // Slow down as the error grows. Uses the P term only so D noise can't jitter the speed.
  // Rises quickly but falls slowly, so the speed doesn't jump back up mid-curve.
  float target = PENALTY_COEF * fabs(uP);
  penalty += (target > penalty ? PENALTY_ATTACK : PENALTY_RELEASE) * (target - penalty);
  // Without this, PENALTY_COEF * |max P term| can exceed the base speed: then once D
  // cancels P coming into a turn, both wheels go negative and the car backs up.
  penalty = constrain(penalty, 0, baseCmd - MIN_BASE_SPEED);

  int left  = (int)(baseCmd - penalty + u);  // error > 0 -> left wheel faster
  int right = (int)(baseCmd - penalty - u);
  drive(left, right);

#if DEBUG
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print("t="); Serial.print(ms);
    Serial.print(" err="); Serial.print(error);
    Serial.print(" uP="); Serial.print(uP);
    Serial.print(" uD="); Serial.print(uD);
    Serial.print(" base="); Serial.print(baseCmd);
    Serial.print(" pen="); Serial.print(penalty);
    Serial.print(" L="); Serial.print(left);
    Serial.print(" R="); Serial.println(right);
  }
#endif
}

// Motors are off; keep reporting the leg results and the last sensor reading.
void stoppedStep() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint < 1000) return;
  lastPrint = millis();
  Serial.print("STOPPED (need ");
  Serial.print(CROSSBAR_SENSORS);
  Serial.println(" sensors on the bar)");
  printLeg("outbound", 0);
  printLeg("return  ", 1);
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

  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(LED1, OUTPUT);

  Serial.println("Ready: starting 2 s after power-on.");
}

void loop() {
  switch (mode) {
    case WAIT:     waitStep();     break;
    case TRACKING: trackingStep(); break;
    case TURNING:  turningStep();  break;
    case STOPPED:  stoppedStep();  break;
  }
}
