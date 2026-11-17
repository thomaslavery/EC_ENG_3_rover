/*
 * Spin test for the TI-RSLK (MSP-EXP432P401R): tank-turns left, then right, forever.
 *
 *   Power on -> RGB red solid, motors off. Press any bumper to start (1 s delay).
 *   RGB blue = spinning left, RGB green = spinning right, no RGB = pause.
 *   Press reset to stop. If red comes back on by itself, the board reset (brownout?).
 */

const int SPIN_SPEED = 70;              // PWM (0-255) for each wheel
const unsigned long SPIN_MS  = 1500;    // how long each spin lasts
const unsigned long PAUSE_MS = 500;     // pause between spins

const int LEFT_NSLP  = 31;  // P3.7
const int LEFT_DIR   = 29;  // P5.4
const int LEFT_PWM   = 40;  // P2.7
const int RIGHT_NSLP = 11;  // P3.6
const int RIGHT_DIR  = 30;  // P5.5
const int RIGHT_PWM  = 39;  // P2.6

const int BUMPERS[6] = {24, 25, 6, 27, 8, 28};  // active LOW

void setMotor(int dirPin, int pwmPin, int speed) {
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

void setup() {
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

  digitalWrite(RED_LED, HIGH);
  while (!anyBumperPressed()) delay(10);
  while (anyBumperPressed()) delay(10);
  digitalWrite(RED_LED, LOW);
  delay(1000);  // get your hand clear
}

void loop() {
  digitalWrite(BLUE_LED, HIGH);
  drive(-SPIN_SPEED, SPIN_SPEED);  // left
  delay(SPIN_MS);
  drive(0, 0);
  digitalWrite(BLUE_LED, LOW);
  delay(PAUSE_MS);

  digitalWrite(GREEN_LED, HIGH);
  drive(SPIN_SPEED, -SPIN_SPEED);  // right
  delay(SPIN_MS);
  drive(0, 0);
  digitalWrite(GREEN_LED, LOW);
  delay(PAUSE_MS);
}
