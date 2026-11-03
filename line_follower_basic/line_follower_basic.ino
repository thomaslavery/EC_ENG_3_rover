/*
 * Baseline line follower for the TI-RSLK (MSP-EXP432P401R) using the ECE3 library.
 * Deliberately simple: each sensor is just black or white, and the car picks one of
 * five fixed wheel-speed pairs. No calibration, no PD, wheels never reverse.
 *
 * Operation:
 *   1. Power on -> motors off. Serial (9600) prints the sensors as a pattern like
 *      "L ...##... R" so you can check the threshold by sliding the car over the line.
 *   2. Press any bumper -> 1 s pause -> follows the line.
 *   3. Stops when (almost) every sensor sees black (end crossbar), or on a bumper hit.
 */

#include <ECE3.h>

// ---------------- Tunables ----------------
const int BLACK_RAW = 1500;   // raw reading above this = black (white ~700-1000, black ~2000-2500)

const int SPEED_STRAIGHT = 50;
const int SPEED_SOFT_FAST = 55, SPEED_SOFT_SLOW = 30;  // line slightly off center
const int SPEED_HARD_FAST = 55, SPEED_HARD_SLOW = 0;   // line near the edge

// ---------------- Pins ----------------
const int LEFT_NSLP  = 31;  // P3.7
const int LEFT_DIR   = 29;  // P5.4
const int LEFT_PWM   = 40;  // P2.7
const int RIGHT_NSLP = 11;  // P3.6
const int RIGHT_DIR  = 30;  // P5.5
const int RIGHT_PWM  = 39;  // P2.6

const int BUMPERS[6] = {24, 25, 6, 27, 8, 28};  // active LOW

// Sensor 0 is on the RIGHT side of the car, sensor 7 on the LEFT.
uint16_t raw[8];
bool black[8];

int leftSpeed = 0, rightSpeed = 0;  // last command, reused when the line is lost
int crossbarCount = 0;

void drive(int left, int right) {
  analogWrite(LEFT_PWM, left);
  analogWrite(RIGHT_PWM, right);
}

bool anyBumperPressed() {
  for (int i = 0; i < 6; i++) {
    if (digitalRead(BUMPERS[i]) == LOW) return true;
  }
  return false;
}

void readSensors() {
  ECE3_read_IR(raw);
  for (int i = 0; i < 8; i++) black[i] = raw[i] > BLACK_RAW;
}

void printPattern() {
  Serial.print("L ");
  for (int i = 7; i >= 0; i--) Serial.print(black[i] ? '#' : '.');
  Serial.println(" R");
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
  digitalWrite(LEFT_DIR, LOW);   // forward, never changed
  digitalWrite(RIGHT_DIR, LOW);
  drive(0, 0);

  for (int i = 0; i < 6; i++) pinMode(BUMPERS[i], INPUT_PULLUP);
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  // Wait for a bumper press, showing the sensor pattern meanwhile
  digitalWrite(RED_LED, HIGH);
  while (!anyBumperPressed()) {
    readSensors();
    printPattern();
    delay(200);
  }
  while (anyBumperPressed()) delay(10);
  digitalWrite(RED_LED, LOW);
  delay(1000);  // get your hand clear
}

void loop() {
  if (anyBumperPressed()) {
    drive(0, 0);
    digitalWrite(GREEN_LED, HIGH);
    while (true) {}  // stopped for good, press reset to run again
  }

  readSensors();

  int count = 0;
  for (int i = 0; i < 8; i++) {
    if (black[i]) count++;
  }

  // End crossbar: 7+ sensors black for 3 reads in a row
  if (count >= 7) crossbarCount++;
  else crossbarCount = 0;
  if (crossbarCount >= 3) {
    drive(0, 0);
    digitalWrite(GREEN_LED, HIGH);
    while (true) {}
  }

  // Pick a steering level from the outermost black sensor.
  // If no sensor sees the line, leftSpeed/rightSpeed keep their last values,
  // so the car keeps turning the way it was going.
  if (black[0] || black[1]) {         // line far right -> hard right
    leftSpeed = SPEED_HARD_FAST;  rightSpeed = SPEED_HARD_SLOW;
  } else if (black[7] || black[6]) {  // line far left -> hard left
    leftSpeed = SPEED_HARD_SLOW;  rightSpeed = SPEED_HARD_FAST;
  } else if (black[2]) {              // slightly right -> soft right
    leftSpeed = SPEED_SOFT_FAST;  rightSpeed = SPEED_SOFT_SLOW;
  } else if (black[5]) {              // slightly left -> soft left
    leftSpeed = SPEED_SOFT_SLOW;  rightSpeed = SPEED_SOFT_FAST;
  } else if (black[3] || black[4]) {  // centered -> straight
    leftSpeed = SPEED_STRAIGHT;   rightSpeed = SPEED_STRAIGHT;
  }

  drive(leftSpeed, rightSpeed);
}
