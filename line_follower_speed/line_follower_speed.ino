/*
 * Speed-tuned PD line follower for a horseshoe track on the TI-RSLK (MSP-EXP432P401R),
 * using the ECE3 library. Same PD controller, slowdown, 180 turn and stop logic as
 * line_follower_split.ino, adapted for:
 *
 *   start bar ===== straight =====\
 *                                  )  180 deg curve
 *   end bar   ===== straight =====/
 *
 * Each leg (out and back) is split into three timed segments, timed from the start of
 * that leg (trackStartMs):
 *   STRAIGHT until CURVE_START, CURVE until CURVE_END, then STRAIGHT to the bar.
 * The outbound and return legs have their own curve times (OUT_* and RET_*).
 *
 * Changes for speed and for thin end bars:
 *   - Sensors are read on every pass of loop() (~2.6 ms), not just every 5 ms, so a thin
 *     bar isn't missed at full speed. The PD update still runs every 5 ms, so the gains
 *     mean the same as in the other sketches.
 *   - A bar = CROSSBAR_SENSORS (default 4) sensors darker than BAR_THRESHOLD within the
 *     last BAR_WINDOW_MS. The window means the sensors don't all have to see the bar on
 *     the same read (thin bar, crossed at a slight angle, read mid-way across it).
 *     It's only looked for after the curve (CURVE_END), so the curve can't trigger it.
 *   - Backup: if the line disappears for END_LOST_MS after the curve, the car treats that
 *     as the end of the track (it ran past the bar without seeing it).
 *   - After stopping, Serial prints the most sensors that saw the bar on each leg, and
 *     whether the leg ended on the bar or on the backup.
 *   - Base speed ramps up at BASE_ACCEL per 5 ms so the wheels don't slip at launch or
 *     coming out of the curve. It drops at once when the segment changes to CURVE.
 *   - Short reverse pulse (BRAKE_PWM for BRAKE_MS) at each bar so it doesn't overshoot.
 *
 * Setting it up:
 *   1. Leave CALIBRATE at 1 and do a run. It uses CURVE tuning the whole way, and looks
 *      for the bar END_IGNORE_MS after each leg starts.
 *   2. After it stops, Serial (9600) prints for each leg: how long the leg took and when
 *      the car was steering hard (= the curve).
 *   3. Put those times into OUT_/RET_CURVE_START_MS / _END_MS. Set CURVE_START a little
 *      early so the car has slowed by the time it gets there. CURVE_END must be before the
 *      car reaches the bar, or it won't look for the bar.
 *   4. Set CALIBRATE to 0, then raise STRAIGHT.baseSpeed until it starts losing the line.
 *   Changing the speeds changes the times, so check them again after big speed changes.
 *
 * Operation:
 *   1. Power on -> WAIT (red LED blinks, motors off). Uses the fixed calibration below.
 *   2. Exactly START_DELAY_MS (2 s) after power-on / reset it starts TRACKING.
 *      No bumper press needed.
 *   3. At the first bar (end of track) it brakes, spins 180 deg and follows the line back.
 *   4. At the next bar (start of track) it brakes and stops.
 *
 * LEDs:
 *   red LED1 (P1.0) blinking fast  - tracking loop is running (heartbeat)
 *   RGB red solid                  - using CURVE tuning (off = STRAIGHT)
 *   RGB blue solid                 - spinning 180 or on the return leg (red + blue = magenta)
 *   RGB green solid                - stopped: saw the second bar
 *   RGB red blinking               - back in WAIT, i.e. the board reset (brownout?)
 *   heartbeat frozen, no green     - the code hung
 *
 * Sign convention: error > 0 means the line is to the RIGHT of center, and the car
 * turns right (left wheel faster).
 */

#include <ECE3.h>

// 1 = print the control values over Serial (9600) while driving. Printing slows the loop.
#define DEBUG 0

// 1 = calibration run: CURVE tuning everywhere, bar armed after END_IGNORE_MS. See above.
#define CALIBRATE 1

// ---------------- Segment tuning ----------------
struct Tuning {
  float baseSpeed;    // PWM (0-255) when the line is centered
  float penaltyCoef;  // base speed drops by this * |P term| as the error grows
  float kp;           // per unit error (error range is -3500..3500)
  float kd;           // per unit change in error per 5 ms (0 = P-only)
};

// CURVE starts as the line_follower.ino tuning. STRAIGHT is a starting guess:
// faster, less slowdown (small errors on a straight shouldn't cost speed), more D to
// keep it from weaving.
//                         baseSpeed  penaltyCoef  kp     kd
const Tuning STRAIGHT = {  230,       0.8,         0.03,  0.3 };
const Tuning CURVE    = {  180,       1.6,         0.03,  0.2 };

// ms from the start of each leg (return leg: from the end of the 180 spin)
const unsigned long OUT_CURVE_START_MS = 1200;
const unsigned long OUT_CURVE_END_MS   = 2500;
const unsigned long RET_CURVE_START_MS = 1200;
const unsigned long RET_CURVE_END_MS   = 2500;

// ---------------- End bar ----------------
// More sensitive: lower CROSSBAR_SENSORS or BAR_THRESHOLD, or raise BAR_WINDOW_MS.
// If it stops or turns on a plain straight, go the other way.
const int CROSSBAR_SENSORS = 4;    // this many sensors on the bar = bar
const int BAR_THRESHOLD    = 450;  // sensor above this (0-1000) counts as on the bar
const unsigned long BAR_WINDOW_MS = 12;  // a sensor counts if it was on the bar this recently
const int CROSSBAR_SAMPLES = 1;    // for this many consecutive reads (~2.6 ms each)
const unsigned long END_LOST_MS = 120;   // line gone this long after the curve = end (0 = off)

const int BLACK_THRESHOLD  = 700;  // sensor above this (0-1000) counts as black (spin only)

const int BRAKE_PWM = 150;                // reverse PWM for the brake pulse (0 = coast)
const unsigned long BRAKE_MS = 60;        // brake pulse length

// ---------------- Shared tunables ----------------
const float BASE_ACCEL      = 4;     // max base speed increase per 5 ms step (0 -> 230 in ~0.3 s)
const float LAUNCH_SPEED    = 100;   // base speed at the start of each leg (ramps up from here)
const float PENALTY_ATTACK  = 0.7;   // penalty rising (slowing down), ~20 ms
const float PENALTY_RELEASE = 0.03;  // penalty falling (speeding up), ~165 ms
const float MIN_BASE_SPEED  = 40;    // slowdown never takes the base below this, so the
                                     // outer wheel always drives forward (no backing up)
const float D_ALPHA         = 0.5;   // EMA weight for the derivative (1 = no filtering)
const int   ERROR_SIGN      = 1;     // flip to -1 if the car steers away from the line

const unsigned long LOOP_PERIOD_US = 5000;  // PD update every 5 ms
const unsigned long START_DELAY_MS = 2000;  // starts this long after power-on (millis() = 0)
const unsigned long END_IGNORE_MS  = 300;   // CALIBRATE only: don't look for the bar before this

const int LOST_THRESHOLD = 200;    // all sensors below this (0-1000) = line lost
const int HARD_ERROR     = 1500;   // |error| above this is logged as "in the curve"

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

float prevError = 0;
float lastValidError = 0;
float dFilt = 0;
float penalty = 0;   // smoothed base speed reduction
float baseCmd = 0;   // rate-limited base speed
int crossbarCount = 0;
unsigned long lastLoopUs = 0;
unsigned long trackStartMs = 0;
bool returning = false;  // false = heading to the end bar, true = heading back to the start
unsigned long turnStartMs = 0;

// Per-leg log, printed after stopping (index 0 = outbound, 1 = return)
unsigned long legMs[2] = {0, 0};
unsigned long firstHardMs[2] = {0, 0};
unsigned long lastHardMs[2] = {0, 0};
bool sawHard[2] = {false, false};
int peakBar[2] = {0, 0};               // most sensors on the bar at once, after the curve
const char *legEnd[2] = {"-", "-"};    // what ended each leg

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

// Short reverse pulse to kill forward speed, then motors off
void brake() {
  if (BRAKE_PWM > 0 && BRAKE_MS > 0) {
    drive(-BRAKE_PWM, -BRAKE_PWM);
    delay(BRAKE_MS);
  }
  drive(0, 0);
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

// Which segment of the current leg we're in
bool inCurve(unsigned long ms) {
#if CALIBRATE
  return true;
#else
  if (!returning) return ms >= OUT_CURVE_START_MS && ms < OUT_CURVE_END_MS;
  return ms >= RET_CURVE_START_MS && ms < RET_CURVE_END_MS;
#endif
}

// Only look for the bar on the last straight, so the curve can't set it off
bool barArmed(unsigned long ms) {
#if CALIBRATE
  return ms > END_IGNORE_MS;
#else
  return ms >= (returning ? RET_CURVE_END_MS : OUT_CURVE_END_MS);
#endif
}

void startLeg() {
  // Forget anything seen during the spin or before the start
  for (int i = 0; i < 8; i++) barSeenMs[i] = millis() - 1000;
  lineLost = false;
  prevError = readLine();  // seed so the first D term doesn't spike
  dFilt = 0;
  penalty = 0;
  baseCmd = LAUNCH_SPEED;
  crossbarCount = 0;
  trackStartMs = millis();
  lastLoopUs = micros();
  mode = TRACKING;
}

void stopCar(StopReason reason) {
  brake();
  mode = STOPPED;
  stopReason = reason;
  stopMs = millis();
  legMs[1] = stopMs - trackStartMs;
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

void printLeg(const char *name, int i) {
  Serial.print("  ");
  Serial.print(name);
  Serial.print(": took ");
  Serial.print(legMs[i]);
  Serial.print(" ms, steering hard from ");
  if (sawHard[i]) {
    Serial.print(firstHardMs[i]);
    Serial.print(" to ");
    Serial.print(lastHardMs[i]);
    Serial.print(" ms");
  } else {
    Serial.print("(never)");
  }
  Serial.print(", ended on ");
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
    mode = ARMED;
  }
}

void armedStep() {
  returning = false;
  lastValidError = 0;
  startLeg();
}

// Hit the end bar: brake, then spin left in place.
void startTurn() {
  brake();
  legMs[0] = millis() - trackStartMs;
  digitalWrite(RED_LED, LOW);
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

  if (barArmed(ms)) {
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

  // Log when the car is steering hard, to find the curve times
  if (fabs(error) > HARD_ERROR) {
    if (!sawHard[leg]) { sawHard[leg] = true; firstHardMs[leg] = ms; }
    lastHardMs[leg] = ms;
  }

  // PD update every LOOP_PERIOD_US, using the latest reading
  unsigned long now = micros();
  if (now - lastLoopUs < LOOP_PERIOD_US) return;
  lastLoopUs = now;
  digitalWrite(LED1, (millis() / 100) % 2);  // heartbeat: blinking = loop still running

  bool curve = inCurve(ms);
  const Tuning &t = curve ? CURVE : STRAIGHT;
  digitalWrite(RED_LED, curve ? HIGH : LOW);

  // Base speed: ramps up slowly, drops right away
  if (t.baseSpeed > baseCmd) baseCmd = min(t.baseSpeed, baseCmd + BASE_ACCEL);
  else baseCmd = t.baseSpeed;

  // PD with a lightly filtered derivative (P sees the raw error, no added lag)
  dFilt = D_ALPHA * (error - prevError) + (1 - D_ALPHA) * dFilt;
  prevError = error;
  float uP = t.kp * error;
  float uD = t.kd * dFilt;
  float u = uP + uD;

  // Slow down as the error grows. Uses the P term only so D noise can't jitter the speed.
  // Rises quickly but falls slowly, so the speed doesn't jump each time the error
  // crosses zero.
  float target = t.penaltyCoef * fabs(uP);
  penalty += (target > penalty ? PENALTY_ATTACK : PENALTY_RELEASE) * (target - penalty);
  // Without this, penaltyCoef * 105 (the max P term) can exceed the base speed: then once
  // D cancels P coming into a sharp turn, both wheels go negative and the car backs up.
  penalty = constrain(penalty, 0, baseCmd - MIN_BASE_SPEED);

  int left  = (int)(baseCmd - penalty + u);  // error > 0 -> left wheel faster
  int right = (int)(baseCmd - penalty - u);
  drive(left, right);

#if DEBUG
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 100) {
    lastPrint = millis();
    Serial.print(curve ? "C " : "S ");
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

// Motors are off; keep reporting the leg times and the last sensor reading.
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
