/*
 * Hardware test for the TI-RSLK (MSP-EXP432P401R): IR sensors, bumpers, motors, encoders.
 * Open Serial at 9600. Press any bumper to flip between the two stages:
 *
 *   SENSORS (RGB green) - motors off, prints the raw IR readings 5 times a second,
 *                         left to right as seen from above. Higher = darker (max 2500).
 *   TURN    (RGB blue)  - waits 1 s, tank-turns left 90 deg, pauses, turns right 90 deg,
 *                         then stops. Prints the encoder counts after each turn.
 *                         Press a bumper twice (to SENSORS and back) to run it again.
 *
 * Every bumper press prints which bumper it was (0-5), so you can check all six.
 */

#include <ECE3.h>

// ---------------- Tunables ----------------
const int  TURN_SPEED      = 60;    // PWM (0-255) for the turn test
const long COUNTS_PER_90   = 180;   // encoder counts per wheel for a 90 deg tank turn (tune)
const unsigned long TURN_TIMEOUT_MS = 3000;  // give up if the encoders never get there
const int  BLACK_RAW       = 1500;  // only used for the "...##..." picture in SENSORS

// ---------------- Pins ----------------
const int LEFT_NSLP  = 31;  // P3.7
const int LEFT_DIR   = 29;  // P5.4
const int LEFT_PWM   = 40;  // P2.7
const int RIGHT_NSLP = 11;  // P3.6
const int RIGHT_DIR  = 30;  // P5.5
const int RIGHT_PWM  = 39;  // P2.6

const int BUMPERS[6] = {24, 25, 6, 27, 8, 28};  // P4.0, P4.2, P4.3, P4.5, P4.6, P4.7 (active LOW)

// ---------------- State ----------------
enum Stage { SENSORS, TURN };
Stage stage = SENSORS;
bool turnDone = false;

uint16_t raw[8];  // sensor 0 = rightmost, sensor 7 = leftmost

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

// Index (0-5) of a pressed bumper, or -1 if none
int pressedBumper() {
  for (int i = 0; i < 6; i++) {
    if (digitalRead(BUMPERS[i]) == LOW) return i;
  }
  return -1;
}

// Delay that returns false early if a bumper gets pressed
bool waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    if (pressedBumper() >= 0) return false;
  }
  return true;
}

void printPadded(long value, int width) {
  int digits = 1;
  for (long v = labs(value); v >= 10; v /= 10) digits++;
  if (value < 0) digits++;
  for (int i = digits; i < width; i++) Serial.print(' ');
  Serial.print(value);
}

void printSensorHeader() {
  Serial.println("   S7(L)    S6    S5    S4    S3    S2    S1 S0(R)");
}

void printSensors() {
  ECE3_read_IR(raw);
  Serial.print("  ");
  for (int i = 7; i >= 0; i--) printPadded(raw[i], 6);
  Serial.print("   L ");
  for (int i = 7; i >= 0; i--) Serial.print(raw[i] > BLACK_RAW ? '#' : '.');
  Serial.println(" R");
}

// Tank turn 90 deg: dir = +1 for left, -1 for right.
// Returns false if a bumper was pressed partway through.
bool turn90(int dir) {
  resetEncoderCount_left();
  resetEncoderCount_right();
  unsigned long start = millis();
  bool ok = true;

  drive(-dir * TURN_SPEED, dir * TURN_SPEED);
  while (true) {
    long l = labs(getEncoderCount_left());
    long r = labs(getEncoderCount_right());
    if ((l + r) / 2 >= COUNTS_PER_90) break;
    if (pressedBumper() >= 0) { ok = false; break; }
    if (millis() - start > TURN_TIMEOUT_MS) {
      Serial.println("  TIMEOUT: wheels stalled or an encoder isn't counting");
      break;
    }
  }
  unsigned long elapsed = millis() - start;
  drive(0, 0);
  delay(300);  // let it coast to a stop so the counts include any overshoot

  Serial.print(dir > 0 ? "  left 90:  " : "  right 90: ");
  Serial.print("L enc=");
  printPadded(getEncoderCount_left(), 5);
  Serial.print("  R enc=");
  printPadded(getEncoderCount_right(), 5);
  Serial.print("  took ");
  Serial.print(elapsed);
  Serial.println(" ms");
  return ok;
}

void enterStage(Stage s) {
  stage = s;
  drive(0, 0);
  digitalWrite(GREEN_LED, s == SENSORS ? HIGH : LOW);
  digitalWrite(BLUE_LED, s == TURN ? HIGH : LOW);
  if (s == SENSORS) {
    Serial.println("== SENSORS (motors off) ==");
    printSensorHeader();
  } else {
    Serial.println("== TURN: starting in 1 s, hands off ==");
    turnDone = false;
  }
}

// ---------------- Arduino entry points ----------------
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

  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);

  enterStage(SENSORS);
}

void loop() {
  int b = pressedBumper();
  if (b >= 0) {
    drive(0, 0);
    Serial.print("Bumper ");
    Serial.print(b);
    Serial.println(" pressed");
    while (pressedBumper() >= 0) delay(10);
    delay(50);  // debounce
    enterStage(stage == SENSORS ? TURN : SENSORS);
    return;
  }

  if (stage == SENSORS) {
    static unsigned long lastPrint = 0;
    static int rows = 0;
    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      if (++rows % 20 == 0) printSensorHeader();
      printSensors();
    }
  } else if (!turnDone) {
    turnDone = true;
    // Each step stops early if a bumper is pressed; loop() then switches stage
    if (waitMs(1000) && turn90(+1) && waitMs(500) && turn90(-1)) {
      Serial.println("TURN done. Press a bumper for SENSORS.");
    }
  }
}
