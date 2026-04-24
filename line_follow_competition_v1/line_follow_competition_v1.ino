// ============================================================
//  LINE FOLLOWING COMPETITION CAR
//  version 1.0
//
//  YOUR SETTINGS:
//  - Base speed: 120 (medium)
//  - ToF sensor: ~2cm from front of car
//  - Start position: bottom-left square, car faces RIGHT (toward the line)
//
//  MAP LAYOUT (from the image):
//  - Start square (bottom-left) → car faces RIGHT along dashed bottom line
//  - Main path curves up and travels the outer loop
//  - 3 branch junctions along the way (each branch has a cube at the end)
//  - Charging bay on the LEFT side (bracket-shaped lines on sides)
//  - Final end square at the TOP-RIGHT
//
//  HOW THE CODE WORKS (State Machine):
//  Think of it like a list of tasks. The car does one task at a time.
//  When it finishes a task it moves to the next one automatically.
//
//  Task order:
//  START → FOLLOW_LINE → APPROACH_JUNCTION → TURNING → BRANCH_FOLLOW
//       → SCAN_LINE_TYPE → APPROACH_CUBE
//       → PUSH_CUBE (dashed)  OR  AVOID_CUBE (solid)
//       → RETURN_TO_JUNCTION → REJOIN_MAIN → FOLLOW_LINE
//       (repeat for all 3 branches, then)
//       → CHARGING → FOLLOW_LINE → DONE
// ============================================================

#include <Wire.h>
#include <VL53L0X.h>  //VL53L0X by Pololu library

#define LM1 5   // Left  motor pin 1 (forward)
#define LM2 6   // Left  motor pin 2 (backward)
#define RM1 9   // Right motor pin 1
#define RM2 10  // Right motor pin 2

#define S1 4  // IR sensor — far LEFT
#define S2 3  // IR sensor — center-left
#define S3 2  // IR sensor — CENTER
#define S4 7  // IR sensor — center-right
#define S5 8  // IR sensor — far RIGHT

#define XSHUT_PIN 11  // VL53L0X hard-reset pin

//  TUNE PARAMETERS
// --- Motor speeds (0 to 255) ---
#define BASE_SPEED 95  // Normal driving speed
#define SLOW_SPEED 60   // Careful zones (near junctions / cubes)
#define TURN_SPEED 90  // Spinning on the spot for 90 degree turns

// --- PID line following gains ---
#define KP 29  // How hard to correct left/right (lower = smoother)
#define KD 15  // Reduces wobble (raise if car oscillates)

// --- ToF distance sensor --- 2cm from the front of the car. detect the cube 90mm ahead.
#define CUBE_DETECT_MM 90 

// --- Timing (milliseconds) --- distance estimates based on time.
#define CROSS_JUNCTION_MS 500       // go forward to centre branch
#define TURNING_DURATION 500       // Turning duration for 90 degree

#define BRANCH_TRAVEL_MS 500        // Travel ~10cm down branch before scanning
#define SCAN_DURATION_MS 1000       // Time spent crawling to detect dashed/solid
#define PUSH_FORWARD_MS 900         // Push cube into square (~15cm forward)
#define PUSH_REVERSE_MS 1050        // Reverse back out (a little extra to clear cube)
#define AVOID_BACKUP_MS 400         // Back up before U-turn (solid line case)
#define UTURN_MS 850                // Spin duration for 180 degree U-turn — TUNE THIS!
#define CHARGE_ENTER_MS 500         // Drive time to get fully inside the charging bay
#define CHARGE_WAIT_MS 5500         // Stay inside 5.5 seconds (rules say 5 minimum)
#define CHARGE_EXIT_SEARCH_MS 2500  // Max time looking for exit line after charging

// --- Dashed line detection ---
// A dashed line causes several black->white->black sensor flips as the car moves.
// A solid line stays black the whole time (0 flips).
#define DASHED_FLIP_THRESHOLD 4  // 4 or more transitions = dashed line

// ============================================================
//  STATE MACHINE — the "chapters" of the car's journey
// ============================================================
enum State {
  START,               // Leave the starting square and find the line
  FOLLOW_LINE,         // Normal PID line following
  APPROACH_JUNCTION,   // Slow down and centre over the T-junction
  TURNING,             // Spin 90 degrees left or right
  BRANCH_FOLLOW,       // Follow the branch ~10cm to reach the scan zone
  SCAN_LINE_TYPE,      // Detect whether line ahead of cube is dashed or solid
  APPROACH_CUBE,       // Creep toward cube until ToF detects it
  PUSH_CUBE,           // Push cube into square, then reverse out  (dashed)
  AVOID_CUBE,          // Back up and U-turn without touching cube  (solid)
  RETURN_TO_JUNCTION,  // Follow branch line back to T-junction
  REJOIN_MAIN,         // Turn back onto the main path
  CHARGING,            // Enter bay, wait 5+ s, exit and find next line
  DONE                 // Final square reached — stop!
};

State currentState = START;

// ============================================================
//  GLOBAL VARIABLES
// ============================================================
byte s1, s2, s3, s4, s5;  // Sensor readings: 0 = black (line), 1 = white

int pidError = 0;
int lastPidError = 0;

byte branchCount = 0;        // Number of branches completed so far (0 to 3)
bool turnLeft = false;      // Which way to turn at the current junction
bool isDashedLine = false;  // Was the line before the cube dashed?

unsigned long stateStartTime = 0;  // Time when current state began

VL53L0X tof;

void setup() {
  Serial.begin(115200);
  Serial.println(F(" Starting ................"));

  // Motor output pins
  pinMode(LM1, OUTPUT);
  pinMode(LM2, OUTPUT);
  pinMode(RM1, OUTPUT);
  pinMode(RM2, OUTPUT);

  // IR sensor input pins
  pinMode(S1, INPUT);
  pinMode(S2, INPUT);
  pinMode(S3, INPUT);
  pinMode(S4, INPUT);
  pinMode(S5, INPUT);


  // distance sensor
  Wire.begin();
  tof.setTimeout(500);
  if (!tof.init()) {
    Serial.println(F("Failed to detect and initialize tof!"));
    while (1) {}
  }
  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. tof.startContinuous(100)).
  tof.startContinuous(50);


  Serial.println(F("Place car on start square. Running in 3 seconds..."));
  delay(3000);

  enterState(START);
  Serial.println(F("GO!"));
}

void loop() {
  readSensors();  // Always read sensors first every loop

  switch (currentState) {
    case START: doStart(); break;
    case FOLLOW_LINE: doFollowLine(); break;
    case APPROACH_JUNCTION: doApproachJunction(); break;
    case TURNING: doTurning(); break;
    case BRANCH_FOLLOW: doBranchFollow(); break;
    case SCAN_LINE_TYPE: doScanLineType(); break;
    case APPROACH_CUBE: doApproachCube(); break;
    case PUSH_CUBE: doPushCube(); break;
    case AVOID_CUBE: doAvoidCube(); break;
    case RETURN_TO_JUNCTION: doReturnToJunction(); break;
    case REJOIN_MAIN: doRejoinMain(); break;
    case CHARGING: doCharging(); break;
    case DONE: doDone(); break;
  }
}

//  START
//  move forward until meet the line from black square.
void doStart() {
  motorDrive(SLOW_SPEED, SLOW_SPEED);  // go forward

  // When both corner sensors sees white, we have cleared the square edge
  if (s1 == 1 || s2 == 1 || s3 == 1 || s4 == 1 || s5 == 1) {
    Serial.println(F("[START] Left starting square — finding the line..."));
    driveTime(SLOW_SPEED, SLOW_SPEED, 200);  //  bit more to fully clear
    motorStop();
    delay(100);
    enterState(FOLLOW_LINE);
  }
}

// ----------------------------------------------------------
//  FOLLOW LINE
//  Normal PID driving. Watches for three triggers:
//  1. T-junction (both outer sensors hit wide cross-line)
//  2. Line disappears > 150ms after all branches = charging gap
//  3. All sensors black after all branches + charging = end square
// ----------------------------------------------------------
void doFollowLine() {

   // --- End square check (all atleast 4 black, all branches and charging done) ---
   bool allBlack = (s2 == 0 && s3 == 0 && s4 == 0) && (s1 == 0 || s5 == 0);
   bool isLeftBranchDetected = (s1 == 0 && s2 == 0 && s5 == 1 );
   bool isRightBranchDetected = (s4 == 0 && s5 == 0 && s1 == 1 );

  if (allBlack && branchCount > 3) {
    Serial.println(F("[FOLLOW] All sensors black = END SQUARE. Finished!"));
    motorStop();
    delay(300);
    enterState(DONE);
    return;
  }

//take left branch
if (isLeftBranchDetected && branchCount < 3) {
    Serial.print(F("[FOLLOW] Left branch detected! Branch #"));
    branchCount++;
    Serial.println(branchCount);
    turnLeft = true; // set turn direction for this branch
    motorStop();
    delay(100); //100
    enterState(APPROACH_JUNCTION);
    return;
  }

//take right branch
  if (isRightBranchDetected && branchCount < 3) {
    Serial.print(F("[FOLLOW] Right branch detected! Branch #"));
    branchCount++;
    Serial.println(branchCount);
    turnLeft = false; // set turn direction for this branch
    motorStop();
    delay(100);
    enterState(APPROACH_JUNCTION);
    return;
  }

  //take charging branch
  if ((isRightBranchDetected || isLeftBranchDetected) && branchCount <=3) {
    Serial.print(F("[FOLLOW] Charging branch detected! Branch #"));
    branchCount++;
    Serial.println(branchCount);
    turnLeft = false; // set turn direction for this branch
    motorStop();
    delay(100);
    enterState(APPROACH_JUNCTION);
    return;
  }




//   // --- T-junction check (outer sensors both black, still have branches left) ---
// bool tJunction = (s1 == 0 && s5 == 0 && branchCount < 3);
// if (tJunction) {
//     // Determine direction from which side sees the branch
//     if (s1 == 0 && s2 == 0) turnLeft = true;
//     else if (s4 == 0 && s5 == 0) turnLeft = false;
    
//     Serial.print(F("[FOLLOW] T-junction detected! Branch #"));
//     Serial.println(branchCount + 1);
//     motorStop();
//     delay(100);
//     enterState(APPROACH_JUNCTION);
//     return;
// }

//   // --- Charging bay gap (all sensors white for > 150ms, after 3 branches) ---
//   bool noLine = (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1);
//   if (noLine && branchCount >= 3) {
//     static unsigned long lineLostAt = 0;
//     if (lineLostAt == 0) lineLostAt = millis();
//     if (millis() - lineLostAt > 150) {
//       lineLostAt = 0;
//       Serial.println(F("[FOLLOW] Line gap detected = CHARGING area!"));
//       enterState(CHARGING);
//       return;
//     }
//     // Gap too short — keep going straight
//     motorDrive(SLOW_SPEED, SLOW_SPEED);
//     return;
//   }

  // --- Normal PID line following ---
  doPID(BASE_SPEED);
}


//  APPROACH JUNCTION --- go forward for CROSS_JUNCTION_MS to place the car's center exactly over the junction cross-point before turning.

void doApproachJunction() {
  if (millis() - stateStartTime < CROSS_JUNCTION_MS) {
    motorDrive(SLOW_SPEED, SLOW_SPEED);
  } else {
    motorStop();
    delay(100);
    // turnLeft = pickTurnDirection();
    Serial.print(F("[JUNCTION] Turn direction: "));
    Serial.println(turnLeft ? F("LEFT") : F("RIGHT"));
    enterState(TURNING);
  }
}

//  TURNING --- Spin in place until TURNING_DURATION.
void doTurning() {

  //   if (turnLeft) {
  //   motorDrive(-TURN_SPEED, TURN_SPEED);  // Spin left
  // } else {
  //   motorDrive(TURN_SPEED, -TURN_SPEED);  // Spin right
  // }

  // if (s3 == 0) {  // Center sensor found the branch line
  //   Serial.println(F("[TURNING] Found branch line!"));
  //   motorStop();
  //   delay(150);
  //   enterState(BRANCH_FOLLOW);
  //   enterState(DONE);
  // }


  if (turnLeft) {
    driveTime(-TURN_SPEED, TURN_SPEED, TURNING_DURATION);  // Spin left
    Serial.println(F("[TURNING] left turning complete!"));
    motorStop();
    delay(150);
    enterState(BRANCH_FOLLOW);
    // enterState(DONE);
  } else {
    driveTime(TURN_SPEED, -TURN_SPEED, TURNING_DURATION);  // Spin right
    Serial.println(F("[TURNING] right turning complete!"));
    motorStop();
    delay(150);
    enterState(BRANCH_FOLLOW);
    // enterState(DONE);
  }
} 


//  BRANCH FOLLOW
//  Follow the branch for BRANCH_TRAVEL_MS (~10cm).

void doBranchFollow() {
  if (millis() - stateStartTime < BRANCH_TRAVEL_MS) {
    doPID(SLOW_SPEED);
  } else {
    motorStop();
    delay(100);
    Serial.println(F("[BRANCH] ~10cm done — scanning line type..."));
    enterState(SCAN_LINE_TYPE);
  }
}

// ----------------------------------------------------------
//  SCAN LINE TYPE
//  Crawl forward for SCAN_DURATION_MS and count how many times
//  the center sensor flips between black and white.
//  Dashed = many flips. Solid = zero or very few flips.
// ----------------------------------------------------------
void doScanLineType() {
  Serial.println(F("[SCAN] going forward and counting sensor flips..."));

  int flipCount = 0;
  int lastReading = digitalRead(S3);
  unsigned long scanEnd = millis() + SCAN_DURATION_MS;

  while (millis() < scanEnd) {
    motorDrive(SLOW_SPEED, SLOW_SPEED);
    delay(15);
    int now = digitalRead(S3);
    if (now != lastReading) {
      flipCount++;
      lastReading = now;
    }
  }
  motorStop();
  delay(100);

  isDashedLine = (flipCount >= DASHED_FLIP_THRESHOLD);

  Serial.print(F("[SCAN] Flips: "));
  Serial.print(flipCount);
  Serial.print(F("  =>  "));
  Serial.println(isDashedLine ? F("DASHED line (will PUSH cube)") : F("SOLID line (will AVOID cube)"));

  enterState(APPROACH_CUBE);
}

//  APPROACH CUBE
//  go forward using PID until the ToF sensor sees the cube
//  closer than CUBE_DETECT_MM.
void doApproachCube() {
  int dist = readTof();

  // Print distance to Serial every ~200ms to monitor
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 200) {
    Serial.print(F("[APPROACH] ToF distance: "));
    if (dist < 0) Serial.println(F("out of range"));
    else {
      Serial.print(dist);
      Serial.println(F(" mm"));
    }
    lastPrint = millis();
  }

  if (dist > 0 && dist < CUBE_DETECT_MM) {
    Serial.println(F("[APPROACH] Cube detected! Stopping approach."));
    motorStop();
    delay(200);
    enterState(isDashedLine ? PUSH_CUBE : AVOID_CUBE);
    return;
  }

  doPID(SLOW_SPEED);  // Keep creeping toward cube
}

// ----------------------------------------------------------
//  PUSH CUBE  (dashed line)
//  Drive forward to push the cube fully inside the black square,
//  then reverse back to roughly where we started.
// ----------------------------------------------------------
void doPushCube() {
  Serial.println(F("[PUSH] Pushing cube into square..."));
  driveTime(SLOW_SPEED, SLOW_SPEED, PUSH_FORWARD_MS);

  Serial.println(F("[PUSH] Reversing out of square..."));
  driveTime(-SLOW_SPEED, -SLOW_SPEED, PUSH_REVERSE_MS);

  motorStop();
  delay(200);
  Serial.println(F("[PUSH] Done. Returning to junction."));
  enterState(RETURN_TO_JUNCTION);
}

// ----------------------------------------------------------
//  AVOID CUBE  (solid line)
//  Do NOT touch the cube. Back up, do a 180 U-turn, return.
// ----------------------------------------------------------
void doAvoidCube() {
  Serial.println(F("[AVOID] Solid line — must NOT touch cube!"));

  // Back away from cube first
  driveTime(-SLOW_SPEED, -SLOW_SPEED, AVOID_BACKUP_MS);
  motorStop();
  delay(100);

  // Spin 180 degrees (U-turn)
  Serial.println(F("[AVOID] Doing 180 U-turn..."));
  motorDrive(TURN_SPEED, -TURN_SPEED);  // Spin right
  delay(UTURN_MS);
  motorStop();
  delay(100);

  Serial.println(F("[AVOID] Done. Returning to junction."));
  enterState(RETURN_TO_JUNCTION);
}

// ----------------------------------------------------------
//  RETURN TO JUNCTION
//  Follow the branch line back. Stop when S1 AND S5 both go
//  black again (we're back at the wide T-junction crossing).
// ----------------------------------------------------------
void doReturnToJunction() {
  bool atJunction = (s1 == 0 && s5 == 0);

  if (atJunction) {
    Serial.println(F("[RETURN] Reached junction again!"));
    motorStop();
    delay(200);
    enterState(REJOIN_MAIN);
    return;
  }

  doPID(SLOW_SPEED);
}

// ----------------------------------------------------------
//  REJOIN MAIN
//  Spin back onto the main path (opposite of how we entered).
//  Count this branch as done, then resume line following.
// ----------------------------------------------------------
void doRejoinMain() {
  // Turn SAME direction as entry to get back onto main path
  if (turnLeft) {
    motorDrive(-TURN_SPEED, TURN_SPEED);  // Turn left to exit
  } else {
    motorDrive(TURN_SPEED, -TURN_SPEED);  // Turn right to exit
  }

  unsigned long start = millis();
  while (millis() - start < 1500) {
    readSensors();
    if (s3 == 0) break;
  }
  motorStop();
  delay(150);

  branchCount++;
  Serial.print(F("[REJOIN] Back on main path. Branches done: "));
  Serial.println(branchCount);

  enterState(FOLLOW_LINE);
}

// ----------------------------------------------------------
//  CHARGING
//  Three phases:
//  Phase 0 — Drive straight into the bay (ignore side bracket lines)
//  Phase 1 — Sit still for CHARGE_WAIT_MS (5.5 seconds)
//  Phase 2 — Drive forward until center sensor finds the exit line
// ----------------------------------------------------------
void doCharging() {
  static int chargePhase = 0;
  static unsigned long phaseTimer = 0;
  static unsigned long lastSecPrint = 0;

  // --- Phase 0: Enter ---
  if (chargePhase == 0) {
    Serial.println(F("[CHARGE] Entering charging bay straight..."));
    // Drive straight — NO PID correction, ignore bracket lines on sides!
    driveTime(SLOW_SPEED, SLOW_SPEED, CHARGE_ENTER_MS);
    motorStop();
    chargePhase = 1;
    phaseTimer = millis();
    lastSecPrint = millis();
    Serial.println(F("[CHARGE] Inside bay. Timer started."));
  }

  // --- Phase 1: Wait ---
  else if (chargePhase == 1) {
    motorStop();
    unsigned long elapsed = millis() - phaseTimer;

    if (millis() - lastSecPrint > 1000) {
      Serial.print(F("[CHARGE] Waiting... "));
      Serial.print(elapsed / 1000);
      Serial.println(F(" s elapsed"));
      lastSecPrint = millis();
    }

    if (elapsed >= CHARGE_WAIT_MS) {
      Serial.println(F("[CHARGE] 5+ seconds done! Exiting bay."));
      chargePhase = 2;
      phaseTimer = millis();
    }
  }

  // --- Phase 2: Exit and find line ---
  else if (chargePhase == 2) {
    // Drive straight forward — do NOT use PID here.
    // The bracket lines on the sides must NOT be followed.
    // Only stop when the CENTER sensor (S3) hits the real exit line.
    motorDrive(SLOW_SPEED, SLOW_SPEED);

    if (s3 == 0) {
      Serial.println(F("[CHARGE] Exit line found! Continuing main path."));
      motorStop();
      delay(200);
      chargePhase = 0;  // Reset for safety
      enterState(FOLLOW_LINE);
    } else if (millis() - phaseTimer > CHARGE_EXIT_SEARCH_MS) {
      Serial.println(F("[CHARGE] WARNING: Exit line not found after 2.5s. Stopping!"));
      motorStop();
      chargePhase = 0;
      enterState(DONE);
    }
  }
}

// ----------------------------------------------------------
//  DONE
//  Final end square reached. Stop forever.
// ----------------------------------------------------------
void doDone() {
  motorStop();
  static unsigned long lastMsg = 0;
  if (millis() - lastMsg > 2000) {
    Serial.println(F("[DONE] Competition run complete! Car stopped."));
    lastMsg = millis();
  }
}


// --- PID driving ---
void doPID(int baseSpd) {
  pidError = getLineError();
  int correction = KP * pidError + KD * (pidError - lastPidError);
  lastPidError = pidError;
  motorDrive(baseSpd + correction, baseSpd - correction);
}

// --- Calculate line error for PID ---
// Returns: 0 = perfect center, negative = line left, positive = line right
int getLineError() {
  if (s3 == 0 && s2 == 1 && s4 == 1) return 0;        // Perfect center
  else if (s3 == 0 && s2 == 0 && s4 == 1) return -1;  // Slightly left
  else if (s3 == 0 && s2 == 1 && s4 == 0) return 1;   // Slightly right
  else if (s2 == 0 && s3 == 1) return -2;             // Center-left
  else if (s4 == 0 && s3 == 1) return 2;              // Center-right
  else if (s1 == 0 && s2 == 1) return -4;             // Far left
  else if (s5 == 0 && s4 == 1) return 4;              // Far right
  else return lastPidError;                           // Lost — hold direction
}

// --- Read all 5 IR sensors ---
void readSensors() {
  s1 = digitalRead(S1);
  s2 = digitalRead(S2);
  s3 = digitalRead(S3);
  s4 = digitalRead(S4);
  s5 = digitalRead(S5);
}

// --- Drive both motors. Positive = forward, negative = backward. ---
void motorDrive(int L, int R) {
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);

  // Left motor
  if (L >= 0) {
    analogWrite(LM1, L);
    analogWrite(LM2, 0);
  } else {
    analogWrite(LM1, 0);
    analogWrite(LM2, -L);
  }

  // Right motor (wiring reversed — same as your original code)
  if (R >= 0) {
    analogWrite(RM1, 0);
    analogWrite(RM2, R);
  } else {
    analogWrite(RM1, -R);
    analogWrite(RM2, 0);
  }
}

// --- Stop all motors ---
void motorStop() {
  analogWrite(LM1, 0);
  analogWrite(LM2, 0);
  analogWrite(RM1, 0);
  analogWrite(RM2, 0);
}

// --- Drive at speed L, R for exactly 'ms' milliseconds then stop ---
void driveTime(int L, int R, int ms) {
  unsigned long endTime = millis() + ms;
  while (millis() < endTime) motorDrive(L, R);
  motorStop();
}

// --- Read distance from ToF sensor (mm). Returns -1 if out of range or timeout. ---
int readTof() {
  int dist = tof.readRangeContinuousMillimeters();
  if (tof.timeoutOccurred()) return -1;
  return dist;
}

// --- Change to a new state and print it to Serial Monitor ---
void enterState(State s) {
  currentState = s;
  stateStartTime = millis();
  const char* names[] = {
    "START", "FOLLOW_LINE", "APPROACH_JUNCTION", "TURNING", "BRANCH_FOLLOW",
    "SCAN_LINE_TYPE", "APPROACH_CUBE", "PUSH_CUBE", "AVOID_CUBE",
    "RETURN_TO_JUNCTION", "REJOIN_MAIN", "CHARGING", "DONE"
  };
  Serial.print(F(">>> STATE: "));
  Serial.println(names[(int)s]);
}

// --- Decide which way to turn at a junction ---
// Reads which outer sensor has more black detections.
// TIP: Once you know your map, you can hardcode the directions:
//   if (branchCount == 0) return false;  // 1st branch = RIGHT
//   if (branchCount == 1) return false;  // 2nd branch = RIGHT
//   if (branchCount == 2) return true;   // 3rd branch = LEFT
// bool pickTurnDirection() {
//   int leftHits = (s1 == 0 ? 1 : 0) + (s2 == 0 ? 1 : 0);
//   int rightHits = (s4 == 0 ? 1 : 0) + (s5 == 0 ? 1 : 0);
//   return (leftHits > rightHits);  // true = turn left
// }

// ============================================================
//  TUNING GUIDE FOR STUDENTS
// ============================================================
//
//  Do these tests ONE AT A TIME on the real floor:
//
//  1. START — does the car leave the square cleanly and find the line?
//     If it overshoots, reduce the 200ms driveTime in doStart().
//
//  2. FOLLOW_LINE PID — does it stay on the line smoothly?
//     Wobbling too much? Lower KP (try 18).
//     Too slow to react to curves? Raise KP (try 30).
//
//  3. JUNCTION detection — put two pieces of tape wide apart.
//     Does S1 and S5 both go black at the same time? Good.
//     Misses junction? The car may be going too fast — lower BASE_SPEED.
//
//  4. TURN accuracy — does the 90-degree turn end up aligned with the branch?
//     The car stops when S3 sees the branch line, so it should self-correct.
//     If it overshoots, lower TURN_SPEED.
//
//  5. SCAN line type — put the car on a dashed segment. Serial Monitor should
//     show 4+ flips. On a solid line it should show 0-2 flips.
//     Adjust DASHED_FLIP_THRESHOLD if needed.
//
//  6. CUBE detection — run proximity_test_code.ino and hold cube 9cm away.
//     Note the mm reading. Set CUBE_DETECT_MM to slightly above that value.
//     Remember: sensor is 2cm from front, so actual gap = reading - 20mm.
//
//  7. PUSH distance — measure with a ruler how far the car needs to travel
//     to push the cube fully inside. Convert to ms at SLOW_SPEED.
//     Example: if 15cm takes 700ms at your speed, set PUSH_FORWARD_MS = 700.
//
//  8. U-TURN (UTURN_MS) — spin the car in place and time how long 180 degrees
//     takes at TURN_SPEED. Set UTURN_MS to that value.
//
//  9. CHARGING — CHARGE_ENTER_MS should get the robot fully inside the bay.
//     Test by placing car at bay entrance and checking it goes fully in.
// ============================================================
