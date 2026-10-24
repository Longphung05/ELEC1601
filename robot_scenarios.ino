#include <Servo.h>

// =====================================================================
//  ADVANCED MAZE - Tremaux / depth-first search on a cell grid
//
//  How it works:
//    - The robot moves ONE CELL at a time and stops at each cell centre.
//    - At each new cell it reads left / front / right and stores the walls
//      in a map (relative coordinates - the start cell is the middle).
//    - It goes to an open neighbour it has NOT visited yet
//      (straight first, then right, then left).
//    - Visited cells are treated like walls, so it can't loop around islands.
//    - If there is nothing new here, it uses the map to drive the SHORTEST
//      known route to the nearest cell that still has an unexplored opening
//      (faster than retracing its steps one by one).
//    - Finish = a dead end (3 walls) at the end of a straight run of corridor
//      cells (walls on both sides), FINISH_RUN_CELLS long in total. This is
//      checked on the whole map, so it works even if the robot starts inside
//      the finish corridor - it spots it, then drives back into the dead end.
//
//  Simulated on the 2026 S2 example maze with perfect sensors: finishes from
//  all 144 start cells/headings, about 34 cell moves + 27 turns on average.
//
//  Every value marked  // <<< TUNE  is a guess or was copied from the
//  essentials code and should be checked on the real robot.
//
//  Suggested tuning order (change RUN_MODE, upload, test):
//    1. RUN_MODE 1 -> per-frequency sensor calibration (same as essentials)
//    2. RUN_MODE 5 -> wall / no-wall readings with the robot at a cell centre
//    3. RUN_MODE 3 -> TURN_90  (4 turns should end up facing the same way)
//    4. RUN_MODE 4 -> TURN_180
//    5. RUN_MODE 2 -> CELL_TIME + steering (should move exactly one cell)
//    6. RUN_MODE 0 -> full maze run
// =====================================================================

const int RUN_MODE = 0;   // 0 = maze, 1 = freq calibration, 2 = one-cell test,
                          // 3 = 90 turn test, 4 = 180 turn test, 5 = wall reading test

Servo servoLeft;
Servo servoRight;

// ---------------- pins (same as essentials) ----------------
const int leftLed = 10, leftRx = 11, leftRed = A2;
const int midLed = 6,   midRx = 7,   midRed = A1;
const int rightLed = 2, rightRx = 3, rightRed = A0;

// ---------------- sensors ----------------
long leftFreqs[5]  = {38000, 40500, 43000, 45000, 47000};   // <<< TUNE (calibration)
long midFreqs[5]   = {38000, 40500, 43000, 45000, 47000};   // <<< TUNE
long rightFreqs[5] = {38000, 40500, 43000, 45000, 47000};   // <<< TUNE

// score (0-5) -> distance in cm. 99 = no wall.  PLACEHOLDERS from essentials.
//                        score:  0   1   2  3   4    5
float leftTable[6]  = {99, 10, 5, 3, 1.5, 0.5};   // <<< TUNE (calibration)
float midTable[6]   = {99, 10, 5, 3, 1.5, 0.5};   // <<< TUNE
float rightTable[6] = {99, 10, 5, 3, 1.5, 0.5};   // <<< TUNE

const int SAMPLES = 3;          // reads averaged per readSensors() (~30 ms each)  // <<< TUNE
const int SENSE_READS = 1;      // votes when deciding walls at a cell centre (odd number) // <<< TUNE

// wall detection AT A CELL CENTRE. A wall must read <= this to count.
// Side: must see the wall of THIS cell. Front: must see the far edge of THIS
// cell but NOT the wall of the next cell.
const float SIDE_WALL_CM  = 10;   // <<< TUNE (depends on cell size)
const float FRONT_WALL_CM = 10;   // <<< TUNE (depends on cell size)

// ---------------- servo speeds (from essentials) ----------------
const int LEFT_STOP  = 1495;   // <<< TUNE (servo trim)
const int RIGHT_STOP = 1500;   // <<< TUNE
const int LEFT_FWD   = 1545;   // <<< TUNE (faster = less time, but less accurate)
const int RIGHT_FWD  = 1450;   // <<< TUNE
const int LEFT_BACK  = 1445;
const int RIGHT_BACK = 1550;
const int LEFT_TURN_R = 1595, RIGHT_TURN_R = 1600;   // turning right on the spot
const int LEFT_TURN_L = 1395, RIGHT_TURN_L = 1400;   // turning left on the spot

// ---------------- timing (ms) ----------------
const int TURN_90  = 555;      // <<< TUNE (from essentials)
const int TURN_180 = 1110;     // <<< TUNE (from essentials)
const int SETTLE_MS = 150;     // pause after each move/turn   // <<< TUNE (smaller = faster)
const unsigned long CELL_TIME = 1500;  // drive time for one cell - GUESS, measure it! // <<< TUNE
const unsigned long START_DELAY = 2000; // time to put the robot down // <<< TUNE

// ---------------- driving corrections ----------------
const float FRONT_STOP_CM = 3;     // stop a move early if the front gets this close // <<< TUNE
const float SIDE_TARGET_CM = 5;    // distance to hold when only ONE side wall is seen // <<< TUNE
const float KP = 2.0;              // steering strength (us per cm of error) // <<< TUNE
const int STEER_MAX = 15;          // max steering change in us        // <<< TUNE
const int STEADY_READS = 2;        // a side wall must be seen this many reads in a row
                                   // before steering on it (ignores pillars) // <<< TUNE

// front snap: after a move, if there is a wall ahead, creep until it is this close.
// Resets position along the corridor. Turn on once the front sensor is calibrated.
const int USE_FRONT_SNAP = 0;            // <<< TUNE (0 = off, 1 = on)
const float FRONT_SNAP_CM = 5;           // <<< TUNE
const unsigned long SNAP_TIMEOUT = 600;  // <<< TUNE

// ---------------- finish detection ----------------
const int FINISH_RUN_CELLS = 2;   // straight corridor cells (incl. the dead end) // <<< TUNE (2 or 3)

// ---------------- maze map ----------------
// 6x6 maze, unknown start -> 11x11 lets any start fit with the start in the middle.
const int MAP_SIZE = 11;
const int START_X = 5, START_Y = 5;

// directions: 0 = N, 1 = E, 2 = S, 3 = W  (N = the way the robot faces at the start)
const int dx[4] = {0, 1, 0, -1};
const int dy[4] = {-1, 0, 1, 0};

// one byte per cell:
//   bits 0-3 : wall on side N/E/S/W (1 = wall)
//   bit 4    : visited (all 4 sides of this cell are known)
const byte VISITED = 0x10;
byte mapCells[MAP_SIZE][MAP_SIZE];

// route planning (breadth-first search over the known map)
const byte UNREACHED = 255;
byte dist[MAP_SIZE][MAP_SIZE];          // steps to the nearest target cell
byte bfsQueue[MAP_SIZE * MAP_SIZE];     // cells stored as y * MAP_SIZE + x

// exploration preference, relative to the robot: 0 = straight, 1 = right,
// 3 = left, 2 = back. The best order depends on the maze - in simulation
// {0,3,1,2} was faster on the worst starts, {0,1,3,2} on average. // <<< TUNE
const int ORDER[4] = {0, 1, 3, 2};

int px = START_X, py = START_Y;   // robot position
int heading = 0;                  // robot direction (0-3)
int targetX = -1, targetY = -1;   // finish cell once it has been spotted
bool finished = false;
bool failed = false;
unsigned long runStart;

float leftCm, midCm, rightCm;
bool wallL, wallF, wallR;         // last reading at a cell centre

// =====================================================================

void setup() {
  pinMode(leftLed, OUTPUT);  pinMode(leftRx, INPUT);  pinMode(leftRed, OUTPUT);
  pinMode(midLed, OUTPUT);   pinMode(midRx, INPUT);   pinMode(midRed, OUTPUT);
  pinMode(rightLed, OUTPUT); pinMode(rightRx, INPUT); pinMode(rightRed, OUTPUT);

  servoLeft.attach(13);
  servoRight.attach(12);
  stopRobot();
  Serial.begin(9600);

  if (RUN_MODE != 0) return;

  delay(START_DELAY);
  runStart = millis();

  // start cell: read L/F/R, then turn right once so the right sensor sees what
  // was behind us. Now all 4 walls of the start cell are known.
  mapCells[py][px] |= VISITED;
  senseWalls();
  turnRight90();
  senseWalls();
  Serial.println(F("start cell scanned"));
}

void loop() {
  if (RUN_MODE == 1) { stopRobot(); printCalibration(); delay(500); return; }
  if (RUN_MODE == 2) { moveOneCell(); delay(3000); return; }
  if (RUN_MODE == 3) { turnRight90(); delay(2000); return; }
  if (RUN_MODE == 4) { turnAround();  delay(2000); return; }
  if (RUN_MODE == 5) { senseWalls(); printWallTest(); delay(500); return; }

  if (finished) { stopRobot(); showCode(2, 2, 2); return; }   // flash all = done
  if (failed)   { stopRobot(); showCode(0, 0, 1); return; }   // left LED = gave up

  // ---------- 1. arrived at a cell: sense it (first visit only) ----------
  if (!isVisited(px, py)) {
    mapCells[py][px] |= VISITED;
    senseWalls();
  }

  // ---------- 2. finish check (anywhere on the map) ----------
  if (targetX < 0) findFinish();
  if (px == targetX && py == targetY) {
    finished = true;
    stopRobot();
    Serial.print(F("FINISH! time (s): "));
    Serial.println((millis() - runStart) / 1000.0);
    printMap();
    return;
  }

  // ---------- 3. choose where to go ----------
  // a) still exploring: an open, unvisited neighbour right here?
  int goDir = -1;
  if (targetX < 0) {
    for (int i = 0; i < 4; i++) {
      int d = (heading + ORDER[i]) % 4;
      int nx = px + dx[d], ny = py + dy[d];
      if (!hasWall(px, py, d) && inBounds(nx, ny) && !isVisited(nx, ny)) {
        goDir = d;
        break;
      }
    }
  }

  // b) nothing new here (or finish known): follow the shortest known route
  //    to the finish, or to the nearest cell that still has an unexplored opening
  bool planned = false;
  if (goDir == -1) {
    int sources = buildDistances();
    if (sources == 0 || dist[py][px] == UNREACHED) {
      // nothing left to explore (or the map is inconsistent) - give up
      // TODO: if this happens on the real maze, a sensor misread probably
      // closed off a corridor. Could fall back to wall-following here.
      failed = true;
      stopRobot();
      Serial.println(F("no route left - no finish found"));
      printMap();
      return;
    }
    goDir = stepTowardTarget();
    planned = true;
  }

  if (targetX >= 0) showCode(1, 1, 0);   // right + mid = heading to the finish
  else if (planned) showCode(0, 1, 0);   // mid = travelling through known cells
  else showCode(1, 0, 0);                // right = exploring a new cell

  // ---------- 4. turn and move ----------
  faceDir(goDir);
  moveOneCell();
  px += dx[goDir];
  py += dy[goDir];

  Serial.print(planned ? F("ROUTE ") : F("NEW   "));
  Serial.print(F("cell ")); Serial.print(px); Serial.print(','); Serial.print(py);
  Serial.print(F("  heading ")); Serial.print(heading);
  Serial.print(F("  t=")); Serial.println((millis() - runStart) / 1000.0);
}

// =====================================================================
//  MAP HELPERS
// =====================================================================

bool inBounds(int x, int y) {
  return x >= 0 && x < MAP_SIZE && y >= 0 && y < MAP_SIZE;
}

bool isVisited(int x, int y) {
  if (!inBounds(x, y)) return true;
  return mapCells[y][x] & VISITED;
}

bool hasWall(int x, int y, int d) {
  if (!inBounds(x, y)) return true;
  return mapCells[y][x] & (1 << d);
}

// sets/clears a wall on this cell AND the matching side of the neighbour
void setWall(int x, int y, int d, bool w) {
  if (w) mapCells[y][x] |= (1 << d);
  else   mapCells[y][x] &= ~(1 << d);

  int nx = x + dx[d], ny = y + dy[d];
  int od = (d + 2) % 4;
  if (inBounds(nx, ny)) {
    if (w) mapCells[ny][nx] |= (1 << od);
    else   mapCells[ny][nx] &= ~(1 << od);
  }
}

// true if cell (x,y) is a visited dead end with a straight corridor behind it
// (corridor cells = walls on both sides), FINISH_RUN_CELLS long including itself
bool isFinish(int x, int y) {
  if (!inBounds(x, y) || !(mapCells[y][x] & VISITED)) return false;

  int walls = 0, open = -1;
  for (int d = 0; d < 4; d++) {
    if (mapCells[y][x] & (1 << d)) walls++;
    else open = d;
  }
  if (walls != 3) return false;

  int count = 1;   // the dead end itself
  int cx = x + dx[open], cy = y + dy[open];
  while (inBounds(cx, cy)) {
    byte m = mapCells[cy][cx];
    bool sideA = m & (1 << ((open + 1) % 4));
    bool sideB = m & (1 << ((open + 3) % 4));
    if (!(sideA && sideB)) break;
    count++;
    cx += dx[open];
    cy += dy[open];
  }
  return count >= FINISH_RUN_CELLS;
}

// checks the whole map; sets targetX/targetY if the finish has been spotted
void findFinish() {
  for (int y = 0; y < MAP_SIZE; y++) {
    for (int x = 0; x < MAP_SIZE; x++) {
      if (isFinish(x, y)) {
        targetX = x;
        targetY = y;
        Serial.print(F("finish spotted at "));
        Serial.print(x); Serial.print(','); Serial.println(y);
        return;
      }
    }
  }
}

// a visited cell with an open side leading to a cell we haven't been to
bool isFrontier(int x, int y) {
  if (!(mapCells[y][x] & VISITED)) return false;
  for (int d = 0; d < 4; d++) {
    int nx = x + dx[d], ny = y + dy[d];
    if (!hasWall(x, y, d) && inBounds(nx, ny) && !isVisited(nx, ny)) return true;
  }
  return false;
}

// breadth-first search through visited cells. Fills dist[][] with the number of
// moves to the nearest target: the finish if known, otherwise every frontier cell.
// Returns how many target cells there were.
int buildDistances() {
  int head = 0, tail = 0, sources = 0;

  for (int y = 0; y < MAP_SIZE; y++) {
    for (int x = 0; x < MAP_SIZE; x++) {
      dist[y][x] = UNREACHED;
      bool isSource = (targetX >= 0) ? (x == targetX && y == targetY) : isFrontier(x, y);
      if (isSource) {
        dist[y][x] = 0;
        bfsQueue[tail++] = y * MAP_SIZE + x;
        sources++;
      }
    }
  }

  while (head < tail) {
    int x = bfsQueue[head] % MAP_SIZE;
    int y = bfsQueue[head] / MAP_SIZE;
    head++;
    for (int d = 0; d < 4; d++) {
      int nx = x + dx[d], ny = y + dy[d];
      if (hasWall(x, y, d) || !inBounds(nx, ny)) continue;
      if (!(mapCells[ny][nx] & VISITED)) continue;   // only plan through known cells
      if (dist[ny][nx] != UNREACHED) continue;
      dist[ny][nx] = dist[y][x] + 1;
      bfsQueue[tail++] = ny * MAP_SIZE + nx;
    }
  }
  return sources;
}

// picks the neighbour one step closer to the target (straight preferred = fewer turns)
int stepTowardTarget() {
  for (int i = 0; i < 4; i++) {
    int d = (heading + ORDER[i]) % 4;
    int nx = px + dx[d], ny = py + dy[d];
    if (hasWall(px, py, d) || !inBounds(nx, ny)) continue;
    if (dist[ny][nx] == dist[py][px] - 1) return d;
  }
  return heading;   // should never happen
}

// reads the sensors at the cell centre and writes L/F/R walls into the map
void senseWalls() {
  int votesL = 0, votesF = 0, votesR = 0;
  for (int i = 0; i < SENSE_READS; i++) {
    readSensors();
    if (leftCm  <= SIDE_WALL_CM)  votesL++;
    if (midCm   <= FRONT_WALL_CM) votesF++;
    if (rightCm <= SIDE_WALL_CM)  votesR++;
  }
  wallL = votesL * 2 > SENSE_READS;
  wallF = votesF * 2 > SENSE_READS;
  wallR = votesR * 2 > SENSE_READS;

  setWall(px, py, heading, wallF);
  setWall(px, py, (heading + 1) % 4, wallR);
  setWall(px, py, (heading + 3) % 4, wallL);

  Serial.print(F("  walls L")); Serial.print(wallL);
  Serial.print(F(" F")); Serial.print(wallF);
  Serial.print(F(" R")); Serial.print(wallR);
  Serial.print(F("   cm ")); Serial.print(leftCm);
  Serial.print(' '); Serial.print(midCm);
  Serial.print(' '); Serial.println(rightCm);
}

// prints the map: S = start, R = robot, . = visited
void printMap() {
  for (int y = 0; y < MAP_SIZE; y++) {
    for (int x = 0; x < MAP_SIZE; x++) {
      Serial.print('+');
      Serial.print((mapCells[y][x] & 1) ? F("---") : F("   "));
    }
    Serial.println('+');
    for (int x = 0; x < MAP_SIZE; x++) {
      Serial.print((mapCells[y][x] & 8) ? '|' : ' ');
      if (x == px && y == py) Serial.print(F(" R "));
      else if (x == START_X && y == START_Y) Serial.print(F(" S "));
      else if (mapCells[y][x] & VISITED) Serial.print(F(" . "));
      else Serial.print(F("   "));
    }
    Serial.println((mapCells[y][MAP_SIZE - 1] & 2) ? '|' : ' ');
  }
}

// =====================================================================
//  MOTION
// =====================================================================

void stopRobot() {
  servoLeft.writeMicroseconds(LEFT_STOP);
  servoRight.writeMicroseconds(RIGHT_STOP);
}

// forward with steering. steer > 0 veers RIGHT, steer < 0 veers LEFT
void driveSteer(int steer) {
  steer = constrain(steer, -STEER_MAX, STEER_MAX);
  servoLeft.writeMicroseconds(LEFT_FWD + steer);    // left wheel faster -> veer right
  servoRight.writeMicroseconds(RIGHT_FWD + steer);  // right wheel slower -> veer right
}

void turnRight(int ms) {
  servoLeft.writeMicroseconds(LEFT_TURN_R);
  servoRight.writeMicroseconds(RIGHT_TURN_R);
  delay(ms);
  stopRobot();
  delay(SETTLE_MS);
}

void turnLeft(int ms) {
  servoLeft.writeMicroseconds(LEFT_TURN_L);
  servoRight.writeMicroseconds(RIGHT_TURN_L);
  delay(ms);
  stopRobot();
  delay(SETTLE_MS);
}

void turnRight90() { turnRight(TURN_90);  heading = (heading + 1) % 4; }
void turnLeft90()  { turnLeft(TURN_90);   heading = (heading + 3) % 4; }
void turnAround()  { turnRight(TURN_180); heading = (heading + 2) % 4; }

// turns the shortest way to face direction d
void faceDir(int d) {
  int diff = (d - heading + 4) % 4;
  if (diff == 1) turnRight90();
  else if (diff == 3) turnLeft90();
  else if (diff == 2) turnAround();
}

// drives one cell forward, keeping centred using the side walls
// (this replaces scenarios 1 and 5-8 from the essentials)
void moveOneCell() {
  int leftSeen = 0, rightSeen = 0;
  unsigned long start = millis();

  driveSteer(0);
  while (millis() - start < CELL_TIME) {
    readSensors();   // ~90 ms, robot keeps driving meanwhile

    if (midCm <= FRONT_STOP_CM) break;   // about to hit the front wall

    // a side wall must be seen several reads in a row (so a pillar flashing
    // past doesn't yank the steering in open spaces)
    leftSeen  = (leftCm  <= SIDE_WALL_CM) ? leftSeen + 1  : 0;
    rightSeen = (rightCm <= SIDE_WALL_CM) ? rightSeen + 1 : 0;

    float steer = 0;
    if (leftSeen >= STEADY_READS && rightSeen >= STEADY_READS) {
      steer = KP * (rightCm - leftCm);            // both walls: stay in the middle
    } else if (leftSeen >= STEADY_READS) {
      steer = KP * (SIDE_TARGET_CM - leftCm);     // only left wall: hold distance
    } else if (rightSeen >= STEADY_READS) {
      steer = -KP * (SIDE_TARGET_CM - rightCm);   // only right wall: hold distance
    }
    driveSteer((int)steer);
  }

  // optional: line up on a front wall so position errors don't build up
  if (USE_FRONT_SNAP && midCm <= FRONT_WALL_CM) {
    unsigned long snapStart = millis();
    driveSteer(0);
    while (millis() - snapStart < SNAP_TIMEOUT) {
      readSensors();
      if (midCm <= FRONT_SNAP_CM) break;
    }
  }

  stopRobot();
  delay(SETTLE_MS);
}

// =====================================================================
//  SENSORS (same idea as essentials)
// =====================================================================

// reads all 3 sensors SAMPLES times, averages the score, converts to cm
void readSensors() {
  int leftScore = 0, midScore = 0, rightScore = 0;

  for (int s = 0; s < SAMPLES; s++) {
    for (int i = 0; i < 5; i++) {
      tone(leftLed, leftFreqs[i]);
      delay(1);
      if (digitalRead(leftRx) == 0) leftScore++;
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

  // average with rounding -> 0-5
  leftScore  = (leftScore  + SAMPLES / 2) / SAMPLES;
  midScore   = (midScore   + SAMPLES / 2) / SAMPLES;
  rightScore = (rightScore + SAMPLES / 2) / SAMPLES;

  leftCm  = leftTable[leftScore];
  midCm   = midTable[midScore];
  rightCm = rightTable[rightScore];
}

// RUN_MODE 1: prints 1 if that frequency sees the wall (38k 40.5k 43k 45k 47k)
void printCalibration() {
  Serial.print(F("LEFT: "));
  for (int i = 0; i < 5; i++) {
    tone(leftLed, leftFreqs[i]); delay(1);
    Serial.print(digitalRead(leftRx) == 0 ? F("1 ") : F("0 "));
    noTone(leftLed); delay(1);
  }
  Serial.print(F("   MID: "));
  for (int i = 0; i < 5; i++) {
    tone(midLed, midFreqs[i]); delay(1);
    Serial.print(digitalRead(midRx) == 0 ? F("1 ") : F("0 "));
    noTone(midLed); delay(1);
  }
  Serial.print(F("   RIGHT: "));
  for (int i = 0; i < 5; i++) {
    tone(rightLed, rightFreqs[i]); delay(1);
    Serial.print(digitalRead(rightRx) == 0 ? F("1 ") : F("0 "));
    noTone(rightLed); delay(1);
  }
  Serial.println();
}

// RUN_MODE 5: put the robot at a cell centre and check L/F/R match reality.
// Also lights the LEDs: right LED = right wall, mid = front, left = left wall
void printWallTest() {
  showCode(wallR ? 1 : 0, wallF ? 1 : 0, wallL ? 1 : 0);
}

// LEDs: 0 = off, 1 = on, 2 = flash
void showCode(int r, int m, int l) {
  int flashOn = (millis() / 500) % 2;
  digitalWrite(rightRed, (r == 1 || (r == 2 && flashOn)) ? HIGH : LOW);
  digitalWrite(midRed,   (m == 1 || (m == 2 && flashOn)) ? HIGH : LOW);
  digitalWrite(leftRed,  (l == 1 || (l == 2 && flashOn)) ? HIGH : LOW);
}
