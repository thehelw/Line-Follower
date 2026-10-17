//Functional FSM based code
/*
  SIX-SENSOR LINE-FOLLOWING MAZE ROBOT
  Board: Arduino Mega 2560

  Sensor order:
  Lmax, L, CL, CR, R, Rmax

  Sensor pins:
  Lmax = 17
  L    = 18
  CL   = 19
  CR   = 14
  R    = 15
  Rmax = 16

  Motor driver:
  ENA = 5, IN1 = 6, IN2 = 7
  IN3 = 8, IN4 = 9, ENB = 10

  Sensor polarity:
  LOW  = black
  HIGH = white

  Features:
  - PD line following
  - Left-priority junction handling
  - 90-degree pivot turns
  - Search for lost line
  - U-turn if SEARCH_LINE lasts 500 ms
  - Stop at the second sustained all-black marker
*/

// =====================================================
// 1. PIN CONFIGURATION
// =====================================================

const int CR   = 14;
const int R    = 15;
const int Rmax = 16;

const int Lmax = 17;
const int L    = 18;
const int CL   = 19;

const bool BLACK_IS_HIGH = false;

const int ENB = 10;
const int IN4 = 9;
const int IN3 = 8;
const int IN2 = 7;
const int IN1 = 6;
const int ENA = 5;

// =====================================================
// 2. TUNING PARAMETERS
// =====================================================

const int x   = 30;
const int BASE_SPEED   = 100+x;
const int MIN_SPEED    = 60+x;
const int MAX_SPEED    = 130+x;
const int TURN_SPEED   = 100+x;
const int SEARCH_SPEED = 85+x;

const float KP = 18.0;
const float KD = 8.0;

const unsigned long TURN_TIMEOUT_MS    = 1800;
const unsigned long SEARCH_TIMEOUT_MS  = 3000;
const unsigned long SEARCH_CONFIRM_MS  = 600;
const unsigned long UTURN_TIME_MS      = 700; // Calibrate
const unsigned long MARKER_TIME_MS     = 1000;
const unsigned long NODE_COOLDOWN_MS   = 600;
const unsigned long PRINT_INTERVAL_MS  = 150;
const unsigned long NODE_ADVANCE_MS    = 50;

// =====================================================
// 3. SENSOR CONFIGURATION
// =====================================================

// Ordered from leftmost to rightmost.
const int sensorPins[6] = {
  Lmax, L, CL, CR, R, Rmax
};

// Sensor position weights, prioritization is applied to avoid random decisions at junctions.
int weights[6] = {
  -5, -3, -1, 1, 4, 6
};

bool sensorBlack[6];

int blackCount = 0;
float lineError = 0.0;
float previousError = 0.0;

// -1 = line last detected on the left
// +1 = line last detected on the right
int lastDirection = -1;

// =====================================================
// 4. ROBOT STATES
// =====================================================

enum RobotState {
  FOLLOW_LINE,   // State 0
  ADVANCE_NODE,  // State 1
  TURNING,       // State 2
  SEARCH_LINE,   // State 3
  FINISHED,      // State 4 in this enum
  UTURN          // State 5 in this enum
};

// Note: enum values start at zero.
// SEARCH_LINE is State 3 numerically.
// The 500 ms rule applies to SEARCH_LINE by name.

enum TurnDirection {
  TURN_NONE,
  TURN_LEFT,
  TURN_RIGHT
};

RobotState state = FOLLOW_LINE;
TurnDirection turnDirection = TURN_NONE;

unsigned long stateStartTime = 0;
unsigned long lastNodeTime = 0;
unsigned long markerStartTime = 0;
unsigned long lastPrintTime = 0;

bool nodeLatched = false;
bool markerLatched = false;
bool turnSawLineClear = false;
bool finished = false;

int markerCount = 0;

// =====================================================
// 5. SETUP
// =====================================================

void setup() {
  Serial.begin(9600);

  for (int i = 0; i < 6; i++) {
    pinMode(sensorPins[i], INPUT);
  }

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  stopMotors();

  Serial.println("Six-sensor maze robot ready.");
  Serial.println("Sensor order: Lmax L CL CR R Rmax");
  Serial.println("0 = black, 1 = white");
}

// =====================================================
// 6. MAIN LOOP
// =====================================================

void loop() {
  readSensors();
  updateMarkerDetection();

  if (finished) {
    state = FINISHED;
  }

  // Release the junction latch after leaving the junction.
  if (!sensorBlack[0] && !sensorBlack[5]) {
    nodeLatched = false;
  }

  switch (state) {
    case FOLLOW_LINE:
      followLine();
      break;

    case ADVANCE_NODE:
      advanceNode();
      break;

    case TURNING:
      executeTurn();
      break;

    case SEARCH_LINE:
      searchForLine();
      break;

    case UTURN:
      performUTurn();
      break;

    case FINISHED:
      stopMotors();
      break;
  }

  printDebug();
}

// =====================================================
// 7. READ THE SIX SENSORS
// =====================================================

void readSensors() {
  blackCount = 0;
  int weightedSum = 0;

  for (int i = 0; i < 6; i++) {
    int reading = digitalRead(sensorPins[i]);

    sensorBlack[i] = BLACK_IS_HIGH
        ? (reading == HIGH)
        : (reading == LOW);

    if (sensorBlack[i]) {
      blackCount++;
      weightedSum += weights[i];
    }
  }

  // Keep the last error when no sensor sees black.
  if (blackCount > 0) {
    lineError = (float)weightedSum / blackCount;
  }
}

// =====================================================
// 8. FOLLOW THE LINE
// =====================================================

void followLine() {
  // Wide black area: continue forward.
  if (blackCount == 6) {
    drive(BASE_SPEED, BASE_SPEED);
    return;
  }

  // All white: line lost. Begin the search timer.
  if (blackCount == 0) {
    state = SEARCH_LINE;
    stateStartTime = millis();
    return;
  }

  // Update the last known line direction.
  if (lineError < -0.3) {
    lastDirection = -1;
  } else if (lineError > 0.3) {
    lastDirection = 1;
  }

  // Detect possible junction branches.
  bool leftBranch  = sensorBlack[0];
  bool rightBranch = sensorBlack[5];

  if ((leftBranch || rightBranch) &&
      !nodeLatched &&
      millis() - lastNodeTime >= NODE_COOLDOWN_MS) {

    // Left-hand priority.
    if (leftBranch) {
      turnDirection = TURN_LEFT;
    } else {
      turnDirection = TURN_RIGHT;
    }

    nodeLatched = true;
    lastNodeTime = millis();

    state = ADVANCE_NODE;
    stateStartTime = millis();

    drive(BASE_SPEED, BASE_SPEED);
    return;
  }

  // PD controller.
  float derivative = lineError - previousError;
  float correction = KP * lineError + KD * derivative;

  previousError = lineError;

  int leftSpeed = constrain(
    (int)(BASE_SPEED + correction),
    MIN_SPEED, MAX_SPEED
  );

  int rightSpeed = constrain(
    (int)(BASE_SPEED - correction),
    MIN_SPEED, MAX_SPEED
  );

  drive(leftSpeed, rightSpeed);
}

// =====================================================
// 9. APPROACH A JUNCTION
// =====================================================

void advanceNode() {
  drive(BASE_SPEED, BASE_SPEED);

  if (millis() - stateStartTime >= NODE_ADVANCE_MS) {
    state = TURNING;
    stateStartTime = millis();

    // The center sensors must lose the old line first.
    turnSawLineClear = false;
  }
}

// =====================================================
// 10. EXECUTE A 90-DEGREE TURN
// =====================================================

void executeTurn() {
  bool centerOnLine = sensorBlack[2] || sensorBlack[3];

  if (!centerOnLine) {
    turnSawLineClear = true;
  }

  if (turnDirection == TURN_LEFT) {
    drive(-TURN_SPEED, TURN_SPEED);
  } else if (turnDirection == TURN_RIGHT) {
    drive(TURN_SPEED, -TURN_SPEED);
  } else {
    state = FOLLOW_LINE;
    return;
  }

  // Finish after the center sensors reacquire the line.
  if (turnSawLineClear && centerOnLine) {
    stopMotors();

    state = FOLLOW_LINE;
    turnDirection = TURN_NONE;
    previousError = lineError;
    return;
  }

  // Safety timeout.
  if (millis() - stateStartTime >= TURN_TIMEOUT_MS) {
    stopMotors();

    state = SEARCH_LINE;
    stateStartTime = millis();
  }
}

// =====================================================
// 11. SEARCH FOR A LOST LINE
// =====================================================

void searchForLine() {
  // If black is detected, resume normal line following.
  if (blackCount > 0) {
    state = FOLLOW_LINE;
    previousError = lineError;
    return;
  }

  // Search toward the last known line position.
  if (lastDirection < 0) {
    drive(-SEARCH_SPEED, SEARCH_SPEED);
  } else {
    drive(SEARCH_SPEED, -SEARCH_SPEED);
  }

  // If SEARCH_LINE lasts 500 ms, start a U-turn.
  if (millis() - stateStartTime >= SEARCH_CONFIRM_MS) {
    stopMotors();

    Serial.println(
      "SEARCH_LINE lasted 500 ms: starting U-turn"
    );

    state = UTURN;
    stateStartTime = millis();
  }

  // SEARCH_TIMEOUT_MS is retained as a tuning parameter.
  // The 500 ms confirmation rule takes priority here.
}

// =====================================================
// 12. PERFORM A 180-DEGREE U-TURN
// =====================================================

void performUTurn() {
  // Pivot in place.
  drive(TURN_SPEED, -TURN_SPEED);

  if (millis() - stateStartTime >= UTURN_TIME_MS) {
    stopMotors();

    Serial.println("Timed U-turn completed.");

    // Reset the PD controller.
    previousError = 0;

    // Reverse the preferred search direction.
    lastDirection = -lastDirection;

    // Search for and reacquire the line.
    state = SEARCH_LINE;
    stateStartTime = millis();
  }
}

// =====================================================
// 13. CIRCLE MARKER DETECTION
// =====================================================

void updateMarkerDetection() {
  bool allBlack = (blackCount == 6);

  if (!allBlack) {
    markerStartTime = 0;
    markerLatched = false;
    return;
  }

  if (markerStartTime == 0) {
    markerStartTime = millis();
  }

  if (millis() - markerStartTime >= MARKER_TIME_MS &&
      !markerLatched) {

    markerLatched = true;
    markerCount++;

    Serial.print("Marker detected: ");
    Serial.println(markerCount);

    // Assumes the first circle is the start
    // and the second circle is the finish.
    if (markerCount >= 2) {
      finished = true;
      stopMotors();
      Serial.println("Finish marker reached.");
    }
  }
}

// =====================================================
// 14. MOTOR CONTROL
// =====================================================

// Assumes ENA controls the left motor and ENB the right.
// Positive speed = forward; negative speed = backward.

void drive(int leftSpeed, int rightSpeed) {
  setMotor(ENA, IN1, IN2, leftSpeed);
  setMotor(ENB, IN3, IN4, rightSpeed);
}

void setMotor(
  int enablePin,
  int inA,
  int inB,
  int motorSpeed
) {
  motorSpeed = constrain(motorSpeed, -255, 255);

  if (motorSpeed > 0) {
    digitalWrite(inA, LOW);
    digitalWrite(inB, HIGH);
  } else if (motorSpeed < 0) {
    digitalWrite(inA, HIGH);
    digitalWrite(inB, LOW);
  } else {
    digitalWrite(inA, LOW);
    digitalWrite(inB, LOW);
  }

  analogWrite(enablePin, abs(motorSpeed));
}

void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// =====================================================
// 15. SERIAL DEBUG OUTPUT
// =====================================================

void printDebug() {
  if (millis() - lastPrintTime < PRINT_INTERVAL_MS) {
    return;
  }

  lastPrintTime = millis();

  // Print 0 for black and 1 for white.
  for (int i = 0; i < 6; i++) {
    Serial.print(sensorBlack[i] ? 0 : 1);
    Serial.print(i == 5 ? " | " : " ");
  }

  Serial.print("Error: ");
  Serial.print(lineError);

  Serial.print(" | State: ");
  Serial.println((int)state);
}
