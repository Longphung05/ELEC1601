// ELEC1601 robot - essentials scenarios
// press reset -> 5 sec to put the robot down -> shows LED code for 5 sec -> does the move -> stops
// press reset to try again
// set calibrationMode = 1 to just print what the sensors see (serial monitor, 9600)

#include <Servo.h>

Servo servoLeft;
Servo servoRight;

// pins
const int leftLed = 10, leftRx = 11, leftRed = A2;
const int midLed = 6,   midRx = 7,   midRed = A1;
const int rightLed = 2, rightRx = 3, rightRed = A0;

const int calibrationMode = 0;

// frequencies for each sensor - change if a sensor is too sensitive / not sensitive enough
long leftFreqs[5]  = {38000, 39000, 40000, 41000, 42000};
long midFreqs[5]   = {38000, 39000, 40000, 41000, 42000};
long rightFreqs[5] = {38000, 39000, 40000, 41000, 42000};

// distance (cm) for each score. score = how many frequencies saw the wall
// 99 = no wall. THESE ARE PLACEHOLDERS - fill in after calibrating
//              score:   0   1   2   3  4  5
float leftTable[6]  = {99, 15, 10, 5, 3, 1.5};
float midTable[6]   = {99, 15, 10, 5, 3, 1.5};
float rightTable[6] = {99, 15, 10, 5, 3, 1.5};

// distances in cm
const int WALL = 12;          // closer than this = there's a wall
const int FRONT_CLEAR = 10;   // front is clear if further than this
const int TURN_MIN = 3;       // front wall has to be between these for a turn / dead end
const int TURN_MAX = 8;
const int TOO_CLOSE = 3;      // scenario 5/6: this close to one wall
const int DIFF = 4;           // and this much closer than the other side
const int ANGLE_CLOSE = 5;    // scenario 7/8: side and front this close
const int CORNER_CLOSE = 3;   // scenario 9: left and front REALLY close
const int FRONT_STOP = 3;     // stop driving if front gets this close

// times in ms - tune these in the lab
const int FORWARD_5CM = 500;     // PLACEHOLDER
const int DEADEND_FORWARD = 600;
const int TURN_90 = 555;
const int TURN_180 = 1110;
const int TURN_30 = 185;
const int TURN_STEP = 30;        // about 5 degrees, for scenario 7/8
const int SIDESTEP_FORWARD = 600;
const int SIDESTEP_BACK = 500;
const int CORNER_BACK = 500;
const int CORRIDOR_EXTRA = 300;

float leftCm, midCm, rightCm;
int stopped = 0;

void setup() {
  pinMode(leftLed, OUTPUT);  pinMode(leftRx, INPUT);  pinMode(leftRed, OUTPUT);
  pinMode(midLed, OUTPUT);   pinMode(midRx, INPUT);   pinMode(midRed, OUTPUT);
  pinMode(rightLed, OUTPUT); pinMode(rightRx, INPUT); pinMode(rightRed, OUTPUT);

  servoLeft.attach(13);
  servoRight.attach(12);
  stopRobot();

  Serial.begin(9600);

  if (calibrationMode == 0) {
    delay(5000);
  }
}

void loop() {
  if (calibrationMode == 1) {
    printCalibration();
    delay(500);
    return;
  }

  if (stopped == 1) {
    return;
  }

  readSensors();

  // if more than one is true, the last one wins
  int scenario = 0;
  if (leftCm <= WALL && rightCm <= WALL && midCm >= FRONT_CLEAR) scenario = 1;
  if (leftCm <= WALL && rightCm > WALL && midCm >= TURN_MIN && midCm <= TURN_MAX) scenario = 2;
  if (rightCm <= WALL && leftCm > WALL && midCm >= TURN_MIN && midCm <= TURN_MAX) scenario = 3;
  if (leftCm <= WALL && rightCm <= WALL && midCm >= TURN_MIN && midCm <= TURN_MAX) scenario = 4;
  if (midCm >= FRONT_CLEAR && leftCm <= TOO_CLOSE && rightCm - leftCm >= DIFF) scenario = 5;
  if (midCm >= FRONT_CLEAR && rightCm <= TOO_CLOSE && leftCm - rightCm >= DIFF) scenario = 6;
  if (leftCm <= ANGLE_CLOSE && midCm <= ANGLE_CLOSE && rightCm - leftCm >= DIFF) scenario = 7;
  if (rightCm <= ANGLE_CLOSE && midCm <= ANGLE_CLOSE && leftCm - rightCm >= DIFF) scenario = 8;
  if (leftCm <= CORNER_CLOSE && midCm <= CORNER_CLOSE) scenario = 9;

  Serial.print("L ");
  Serial.print(leftCm);
  Serial.print("  M ");
  Serial.print(midCm);
  Serial.print("  R ");
  Serial.print(rightCm);
  Serial.print("  scenario ");
  Serial.println(scenario);

  // showCode(right, mid, left)   0 = off, 1 = on, 2 = flash
  switch (scenario) {
    case 0: // unknown - don't move
      digitalWrite(rightRed, LOW);
      digitalWrite(midRed, LOW);
      digitalWrite(leftRed, LOW);
      stopRobot();
      break;

    case 1: // middle of corridor
      showCode(1, 0, 0);
      forward(FORWARD_5CM);
      break;

    case 2: // right turn
      showCode(0, 1, 0);
      turnRight(TURN_90);
      driveUntilCorridor();
      break;

    case 3: // left turn
      showCode(1, 1, 0);
      turnLeft(TURN_90);
      driveUntilCorridor();
      break;

    case 4: // dead end
      showCode(0, 0, 1);
      turnRight(TURN_180);
      forward(DEADEND_FORWARD);
      break;

    case 5: // too close to left wall
      showCode(1, 0, 1);
      turnRight(TURN_30);
      forward(SIDESTEP_FORWARD);
      turnLeft(TURN_30);
      backward(SIDESTEP_BACK);
      break;

    case 6: // too close to right wall
      showCode(0, 1, 1);
      turnLeft(TURN_30);
      forward(SIDESTEP_FORWARD);
      turnRight(TURN_30);
      backward(SIDESTEP_BACK);
      break;

    case 7: // angled into left wall - turn right bit by bit until front is clear
      showCode(1, 1, 1);
      for (int i = 0; i < 9; i++) {
        turnRight(TURN_STEP);
        readSensors();
        if (midCm >= FRONT_CLEAR) break;
      }
      break;

    case 8: // angled into right wall - turn left bit by bit until front is clear
      showCode(2, 0, 0);
      for (int i = 0; i < 9; i++) {
        turnLeft(TURN_STEP);
        readSensors();
        if (midCm >= FRONT_CLEAR) break;
      }
      break;

    case 9: // stuck in front-left corner - turn left, reverse, turn back
      showCode(2, 2, 0);
      turnLeft(TURN_30);
      backward(CORNER_BACK);
      turnRight(TURN_30);
      break;
  }

  stopped = 1;
}

// reads all 3 sensors 3 times and averages, then turns the score into cm
void readSensors() {
  int leftScore = 0;
  int midScore = 0;
  int rightScore = 0;

  for (int s = 0; s < 3; s++) {
    for (int i = 0; i < 5; i++) {
      tone(leftLed, leftFreqs[i]);
      delay(1);
      if (digitalRead(leftRx) == 0) leftScore++;   // 0 means it sees the wall
      noTone(leftLed);
      delay(1);

      tone(midLed, midFreqs[i]);
      delay(1);
      if (digitalRead(midRx) == 0) midScore++;
      noTone(midLed);
      delay(1);

      tone(rightLed, rightFreqs[i]);
      delay(1);
      if (digitalRead(rightRx) == 0) rightScore++;
      noTone(rightLed);
      delay(1);
    }
  }

  // divide by 3 for the average (the +1 rounds it)
  leftScore = (leftScore + 1) / 3;
  midScore = (midScore + 1) / 3;
  rightScore = (rightScore + 1) / 3;

  leftCm = leftTable[leftScore];
  midCm = midTable[midScore];
  rightCm = rightTable[rightScore];
}

// prints 1 if that frequency sees the wall, 0 if not (38k 39k 40k 41k 42k)
void printCalibration() {
  Serial.print("LEFT: ");
  for (int i = 0; i < 5; i++) {
    tone(leftLed, leftFreqs[i]);
    delay(1);
    if (digitalRead(leftRx) == 0) Serial.print("1 ");
    else Serial.print("0 ");
    noTone(leftLed);
    delay(1);
  }

  Serial.print("   MID: ");
  for (int i = 0; i < 5; i++) {
    tone(midLed, midFreqs[i]);
    delay(1);
    if (digitalRead(midRx) == 0) Serial.print("1 ");
    else Serial.print("0 ");
    noTone(midLed);
    delay(1);
  }

  Serial.print("   RIGHT: ");
  for (int i = 0; i < 5; i++) {
    tone(rightLed, rightFreqs[i]);
    delay(1);
    if (digitalRead(rightRx) == 0) Serial.print("1 ");
    else Serial.print("0 ");
    noTone(rightLed);
    delay(1);
  }

  Serial.println();
}

// stops, shows the LED code for 5 sec. 0 = off, 1 = on, 2 = flash every second
void showCode(int r, int m, int l) {
  stopRobot();
  for (int i = 0; i < 5; i++) {
    int flash = 0;
    if (i % 2 == 0) flash = 1;

    if (r == 1 || (r == 2 && flash == 1)) digitalWrite(rightRed, HIGH);
    else digitalWrite(rightRed, LOW);

    if (m == 1 || (m == 2 && flash == 1)) digitalWrite(midRed, HIGH);
    else digitalWrite(midRed, LOW);

    if (l == 1 || (l == 2 && flash == 1)) digitalWrite(leftRed, HIGH);
    else digitalWrite(leftRed, LOW);

    delay(1000);
  }
}

// for scenario 2/3: keep driving until there's a wall on both sides again
void driveUntilCorridor() {
  servoLeft.writeMicroseconds(1545);
  servoRight.writeMicroseconds(1450);

  for (int i = 0; i < 30; i++) {   // gives up after 30 checks (about 3 sec)
    readSensors();
    if (midCm <= FRONT_STOP) break;
    if (leftCm <= WALL && rightCm <= WALL) {
      forward(CORRIDOR_EXTRA);
      break;
    }
  }
  stopRobot();
}

void forward(int ms) {
  servoLeft.writeMicroseconds(1545);
  servoRight.writeMicroseconds(1450);
  delay(ms);
  stopRobot();
  delay(150);
}

void backward(int ms) {
  servoLeft.writeMicroseconds(1445);
  servoRight.writeMicroseconds(1550);
  delay(ms);
  stopRobot();
  delay(150);
}

void turnRight(int ms) {
  servoLeft.writeMicroseconds(1595);
  servoRight.writeMicroseconds(1600);
  delay(ms);
  stopRobot();
  delay(150);
}

void turnLeft(int ms) {
  servoLeft.writeMicroseconds(1395);
  servoRight.writeMicroseconds(1400);
  delay(ms);
  stopRobot();
  delay(150);
}

void stopRobot() {
  servoLeft.writeMicroseconds(1495);
  servoRight.writeMicroseconds(1500);
}
