/*
  ELEC1601 robot - essentials scenarios (based on Ross Hutton's skeleton)
*/

#include <Servo.h>

// ---------- PINS ----------
#define LEFT_SERVO_PIN  13
#define RIGHT_SERVO_PIN 12

#define LEFT_IR_LED  10
#define LEFT_IR_RX   11
#define MID_IR_LED   6
#define MID_IR_RX    7
#define RIGHT_IR_LED 2
#define RIGHT_IR_RX  3

#define LED_RIGHT A0
#define LED_MID   A1
#define LED_LEFT  A2

// ---------- MODES ----------
#define CALIBRATION_MODE 0   // 1 = never move, just print sensor readings
#define DEBUG_PRINT      1   // 1 = print readings + scenario before moving

// ---------- IR FREQUENCIES (change freely) ----------
// Lower frequency = sees further away.
// score = how many of these frequencies saw the wall (higher score = closer).
// If a sensor still sees the wall at 42k from far away, shift its list up (e.g. 39000-43000).
// Every list needs NUM_FREQS values.
#define NUM_FREQS 5
long leftFreqs[NUM_FREQS]  = {38000, 39000, 40000, 41000, 42000};
long midFreqs[NUM_FREQS]   = {38000, 39000, 40000, 41000, 42000};
long rightFreqs[NUM_FREQS] = {38000, 39000, 40000, 41000, 42000};

// ---------- SCORE -> CM TABLES (PLACEHOLDERS - fill these in!) ----------
// Position in the list = score. Score 0 = nothing seen, so 99 means "no wall".
// To fill in: CALIBRATION_MODE 1, hold a wall at 1.5/3/5/10/15 cm, write the
// distance under the score it prints. Every list needs NUM_FREQS + 1 values.
//                        score:  0   1   2   3   4   5
float leftCmTable[NUM_FREQS + 1]  = {99, 15, 10, 5,  3,  1.5};
float midCmTable[NUM_FREQS + 1]   = {99, 15, 10, 5,  3,  1.5};
float rightCmTable[NUM_FREQS + 1] = {99, 15, 10, 5,  3,  1.5};

// ---------- SCENARIO THRESHOLDS (cm) ----------
#define WALL_SEEN      12   // closer than this = "there's a wall"
#define FRONT_CLEAR    10   // spec: corridor front is clear or at least 10 cm away
#define TURN_FRONT_MIN 3    // 2/3/4: front wall must be between MIN and MAX
#define TURN_FRONT_MAX 8    //        (keep MAX below FRONT_CLEAR)
#define SIDE_TOO_CLOSE 3    // 5/6: this close to one wall...
#define SIDE_DIFF      4    //      ...and this much closer than the other side
#define ANGLE_CLOSE    5    // 7/8: side wall and front both this close
#define CORNER_CLOSE   3    // 9: left wall and front both REALLY close (keep below ANGLE_CLOSE)
#define FRONT_STOP     3    // emergency stop while driving

// ---------- SENSOR READING ----------
#define SAMPLES       5     // readings averaged before choosing a scenario
#define QUICK_SAMPLES 3     // readings averaged while moving
#define CAL_TRIES     10    // calibration mode: tries per frequency

// ---------- MOVEMENT (tune in the lab) ----------
#define MS_PER_CM      100   // PLACEHOLDER: drive forward 2000 ms, measure d cm, set to 2000 / d
#define MS_PER_DEG_CW  6.17  // from old code: 555 ms = 90 deg
#define MS_PER_DEG_CCW 6.17
#define PAUSE_MS       150   // short stop after every move so it doesn't drift into the next one

#define SCEN1_FORWARD_CM    5   // 1: spec says 5 cm
#define DEADEND_FORWARD_CM  6   // 4: spec says at least 5 cm
#define SIDESTEP_DEG        30  // 5/6: turn this much...
#define SIDESTEP_FWD_CM     6   //      ...drive forward this far...
#define SIDESTEP_BACK_CM    5   //      ...turn back, then reverse this far
#define ANGLE_STEP_DEG      5   // 7/8: rotate this much at a time...
#define ANGLE_MAX_STEPS     9   //      ...at most this many times (9 x 5 = 45 deg)
#define CORNER_TURN_DEG     30  // 9: turn CCW this much...
#define CORNER_BACK_CM      5   //    ...reverse this far, then turn CW back
#define CORRIDOR_EXTRA_CM   3   // 2/3: keep going this far after both side walls come back
#define CORRIDOR_MAX_CHECKS 30  // 2/3: give up after this many checks (each is about 0.1 s)

// LED modes for showCode()
#define LED_OFF   0
#define LED_ON    1
#define LED_FLASH 2

Servo servoLeft;
Servo servoRight;

int stopped = 0;
float leftCm, midCm, rightCm;

void setup() {
  pinMode(LEFT_IR_LED, OUTPUT);  pinMode(LEFT_IR_RX, INPUT);
  pinMode(MID_IR_LED, OUTPUT);   pinMode(MID_IR_RX, INPUT);
  pinMode(RIGHT_IR_LED, OUTPUT); pinMode(RIGHT_IR_RX, INPUT);

  pinMode(LED_RIGHT, OUTPUT);
  pinMode(LED_MID, OUTPUT);
  pinMode(LED_LEFT, OUTPUT);

  servoLeft.attach(LEFT_SERVO_PIN);
  servoRight.attach(RIGHT_SERVO_PIN);
  stopMotors();

  Serial.begin(9600);

  if (!CALIBRATION_MODE) delay(5000);   // time to place the robot
}

void loop() {
  if (CALIBRATION_MODE) {
    printCalibration();
    delay(500);
    return;
  }

  if (stopped) return;   // already did a scenario - press reset to go again

  // ---- Measure ----
  leftCm  = readCm(LEFT_IR_LED,  LEFT_IR_RX,  leftFreqs,  leftCmTable,  SAMPLES);
  midCm   = readCm(MID_IR_LED,   MID_IR_RX,   midFreqs,   midCmTable,   SAMPLES);
  rightCm = readCm(RIGHT_IR_LED, RIGHT_IR_RX, rightFreqs, rightCmTable, SAMPLES);

  // ---- Detect scenario ----
  // If more than one matches, the LAST one wins, so the special cases go at the bottom.
  int scenario = 0;

  // 1: corridor - walls both sides, front clear
  if (leftCm <= WALL_SEEN && rightCm <= WALL_SEEN && midCm >= FRONT_CLEAR) scenario = 1;

  // 2: right turn - left wall, no right wall, front wall in range
  if (leftCm <= WALL_SEEN && rightCm > WALL_SEEN && midCm >= TURN_FRONT_MIN && midCm <= TURN_FRONT_MAX) scenario = 2;

  // 3: left turn - right wall, no left wall, front wall in range
  if (rightCm <= WALL_SEEN && leftCm > WALL_SEEN && midCm >= TURN_FRONT_MIN && midCm <= TURN_FRONT_MAX) scenario = 3;

  // 4: dead end - walls everywhere
  if (leftCm <= WALL_SEEN && rightCm <= WALL_SEEN && midCm >= TURN_FRONT_MIN && midCm <= TURN_FRONT_MAX) scenario = 4;

  // 5: hugging the LEFT wall, parallel (right wall may be out of range - that's ok)
  if (midCm >= FRONT_CLEAR && leftCm <= SIDE_TOO_CLOSE && rightCm - leftCm >= SIDE_DIFF) scenario = 5;

  // 6: hugging the RIGHT wall, parallel
  if (midCm >= FRONT_CLEAR && rightCm <= SIDE_TOO_CLOSE && leftCm - rightCm >= SIDE_DIFF) scenario = 6;

  // 7: angled into the LEFT wall - left + front close, right much further
  if (leftCm <= ANGLE_CLOSE && midCm <= ANGLE_CLOSE && rightCm - leftCm >= SIDE_DIFF) scenario = 7;

  // 8: angled into the RIGHT wall - right + front close, left much further
  if (rightCm <= ANGLE_CLOSE && midCm <= ANGLE_CLOSE && leftCm - rightCm >= SIDE_DIFF) scenario = 8;

  // 9: stuck in a front-left corner - left + front REALLY close
  if (leftCm <= CORNER_CLOSE && midCm <= CORNER_CLOSE) scenario = 9;

  if (DEBUG_PRINT) {
    Serial.print("L ");   Serial.print(leftCm);
    Serial.print("  M "); Serial.print(midCm);
    Serial.print("  R "); Serial.print(rightCm);
    Serial.print("  -> scenario ");
    Serial.println(scenario);
  }

  // ---- Do the move ----
  // LED order is the same as the spec: Right (A0), Mid (A1), Left (A2)
  switch (scenario) {
    case 0: // unknown - don't move, take a photo, check the Serial Monitor
      digitalWrite(LED_RIGHT, LOW);
      digitalWrite(LED_MID, LOW);
      digitalWrite(LED_LEFT, LOW);
      stopMotors();
      stopped = 1;
      break;

    case 1: // corridor
      showCode(LED_ON, LED_OFF, LED_OFF);
      forwardCm(SCEN1_FORWARD_CM);
      stopped = 1;
      break;

    case 2: // right turn
      showCode(LED_OFF, LED_ON, LED_OFF);
      turnCW(90);
      driveUntilCorridor();
      stopped = 1;
      break;

    case 3: // left turn
      showCode(LED_ON, LED_ON, LED_OFF);
      turnCCW(90);
      driveUntilCorridor();
      stopped = 1;
      break;

    case 4: // dead end
      showCode(LED_OFF, LED_OFF, LED_ON);
      turnCW(180);
      forwardCm(DEADEND_FORWARD_CM);
      stopped = 1;
      break;

    case 5: // too close to left wall
      showCode(LED_ON, LED_OFF, LED_ON);
      sidestepRight();
      stopped = 1;
      break;

    case 6: // too close to right wall
      showCode(LED_OFF, LED_ON, LED_ON);
      sidestepLeft();
      stopped = 1;
      break;

    case 7: // angled into left wall
      showCode(LED_ON, LED_ON, LED_ON);
      rotateUntilFrontClear(1);   // 1 = clockwise
      stopped = 1;                // comment out to let it carry on (it may then see scenario 5)
      break;

    case 8: // angled into right wall
      showCode(LED_FLASH, LED_OFF, LED_OFF);
      rotateUntilFrontClear(0);   // 0 = anticlockwise
      stopped = 1;                // comment out to let it carry on (it may then see scenario 6)
      break;

    case 9: // stuck in front-left corner
      showCode(LED_FLASH, LED_FLASH, LED_OFF);
      escapeCorner();
      stopped = 1;                // comment out to let it carry on (it may then see scenario 5)
      break;

    default:
      stopMotors();
      stopped = 1;
  }
}

// ================== SCENARIO MOVES ==================

// 5: close to left wall -> angle right, forward, straighten, reverse back
void sidestepRight() {
  turnCW(SIDESTEP_DEG);
  forwardCm(SIDESTEP_FWD_CM);
  turnCCW(SIDESTEP_DEG);
  backwardCm(SIDESTEP_BACK_CM);
}

// 6: close to right wall -> mirror of sidestepRight
void sidestepLeft() {
  turnCCW(SIDESTEP_DEG);
  forwardCm(SIDESTEP_FWD_CM);
  turnCW(SIDESTEP_DEG);
  backwardCm(SIDESTEP_BACK_CM);
}

// 7/8: turn a little at a time until the front sensor stops seeing the wall
void rotateUntilFrontClear(int clockwise) {
  for (int i = 0; i < ANGLE_MAX_STEPS; i++) {
    if (clockwise) turnCW(ANGLE_STEP_DEG);
    else turnCCW(ANGLE_STEP_DEG);

    float front = readCm(MID_IR_LED, MID_IR_RX, midFreqs, midCmTable, QUICK_SAMPLES);
    if (front >= FRONT_CLEAR) break;
  }
}

// 9: front-left corner -> turn CCW, reverse (moves back and away from the left wall), turn CW back
void escapeCorner() {
  turnCCW(CORNER_TURN_DEG);
  backwardCm(CORNER_BACK_CM);
  turnCW(CORNER_TURN_DEG);
}

// 2/3: after turning 90, drive until both side walls are back, then a bit more
void driveUntilCorridor() {
  driveForward();
  for (int i = 0; i < CORRIDOR_MAX_CHECKS; i++) {
    float l = readCm(LEFT_IR_LED,  LEFT_IR_RX,  leftFreqs,  leftCmTable,  QUICK_SAMPLES);
    float m = readCm(MID_IR_LED,   MID_IR_RX,   midFreqs,   midCmTable,   QUICK_SAMPLES);
    float r = readCm(RIGHT_IR_LED, RIGHT_IR_RX, rightFreqs, rightCmTable, QUICK_SAMPLES);

    if (m <= FRONT_STOP) break;          // about to hit something
    if (l <= WALL_SEEN && r <= WALL_SEEN) {
      forwardCm(CORRIDOR_EXTRA_CM);      // back in a corridor, go a bit further
      break;
    }
  }
  stopMotors();
}

// ================== LEDS ==================

// Stop and show the code for 5 seconds. Flashing LEDs go on/off every second.
void showCode(int right, int mid, int left) {
  stopMotors();
  for (int i = 0; i < 5; i++) {
    int flashOn = (i % 2 == 0);   // on, off, on, off, on
    setLed(LED_RIGHT, right, flashOn);
    setLed(LED_MID,   mid,   flashOn);
    setLed(LED_LEFT,  left,  flashOn);
    delay(1000);
  }
}

void setLed(int pin, int mode, int flashOn) {
  if (mode == LED_ON || (mode == LED_FLASH && flashOn)) digitalWrite(pin, HIGH);
  else digitalWrite(pin, LOW);
}

// ================== MOVEMENT ==================

void forwardCm(float cm)  { driveForward();         delay(cm * MS_PER_CM);       stopAndPause(); }
void backwardCm(float cm) { driveBackwards();       delay(cm * MS_PER_CM);       stopAndPause(); }
void turnCW(float deg)    { turnClockwise();        delay(deg * MS_PER_DEG_CW);  stopAndPause(); }
void turnCCW(float deg)   { turnCounterClockwise(); delay(deg * MS_PER_DEG_CCW); stopAndPause(); }

void stopAndPause() {
  stopMotors();
  delay(PAUSE_MS);
}

// Servo values (unchanged from your original)
void driveForward() {
  servoLeft.writeMicroseconds(1545);
  servoRight.writeMicroseconds(1450);
}
void driveBackwards() {
  servoLeft.writeMicroseconds(1445);
  servoRight.writeMicroseconds(1550);
}
void turnClockwise() {
  servoLeft.writeMicroseconds(1595);
  servoRight.writeMicroseconds(1600);
}
void turnCounterClockwise() {
  servoLeft.writeMicroseconds(1395);
  servoRight.writeMicroseconds(1400);
}
void stopMotors() {
  servoLeft.writeMicroseconds(1495);
  servoRight.writeMicroseconds(1500);
}

// ================== SENSORS ==================

// Average a few scores, then look up the distance in that sensor's table
float readCm(int ledPin, int rxPin, long freqs[], float cmTable[], int samples) {
  int total = 0;
  for (int i = 0; i < samples; i++) {
    total = total + getScore(ledPin, rxPin, freqs);
  }
  int score = (total + samples / 2) / samples;   // average, rounded
  return cmTable[score];
}

// How many frequencies saw the wall (0 = nothing, NUM_FREQS = touching)
int getScore(int ledPin, int rxPin, long freqs[]) {
  int score = 0;
  for (int i = 0; i < NUM_FREQS; i++) {
    if (irDetect(ledPin, rxPin, freqs[i]) == 0) score++;   // 0 means wall seen
  }
  return score;
}

// Same as your original: returns 0 if it sees the wall, 1 if it doesn't
int irDetect(int ledPin, int rxPin, long frequency) {
  tone(ledPin, frequency);
  delay(1);
  int irState = digitalRead(rxPin);
  noTone(ledPin);
  delay(1);
  return irState;
}

// ================== CALIBRATION PRINTING ==================

void printCalibration() {
  printSensor("LEFT ", LEFT_IR_LED,  LEFT_IR_RX,  leftFreqs,  leftCmTable);
  printSensor("MID  ", MID_IR_LED,   MID_IR_RX,   midFreqs,   midCmTable);
  printSensor("RIGHT", RIGHT_IR_LED, RIGHT_IR_RX, rightFreqs, rightCmTable);
  Serial.println();
}

// Prints e.g.  LEFT   38k:10/10  39k:9/10  40k:2/10 ...  score=2 -> ~10 cm
void printSensor(String name, int ledPin, int rxPin, long freqs[], float cmTable[]) {
  Serial.print(name);
  int score = 0;
  for (int f = 0; f < NUM_FREQS; f++) {
    int hits = 0;
    for (int t = 0; t < CAL_TRIES; t++) {
      if (irDetect(ledPin, rxPin, freqs[f]) == 0) hits++;
    }
    if (hits > CAL_TRIES / 2) score++;   // counts if it saw the wall most of the time
    Serial.print("  ");
    Serial.print(freqs[f] / 1000);
    Serial.print("k:");
    Serial.print(hits);
    Serial.print("/");
    Serial.print(CAL_TRIES);
  }
  Serial.print("   score=");
  Serial.print(score);
  Serial.print(" -> ~");
  Serial.print(cmTable[score]);
  Serial.println(" cm");
}
