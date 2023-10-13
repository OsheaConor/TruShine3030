#include <AccelStepper.h>

// Eine rev des Stepper motors braucht 800 steps 

#define X_STEP_COUNT 800
#define Y_STEP_COUNT 400

#define X_STEP_CHAR_COUNT 50
#define Y_STEP_CHAR_COUNT 100
// Fiktive Werte!

#define MOTOR_STEP_TIME 200
#define MOTOR_SLOW_STEP_TIME 800
#define MOTOR_SLOW_STEP_FACTOR 2
// Zahlen müssen nochmal überarbeitet werden!
// MOTOR_SLOW_STEP_FACTOR besagt, wie viel langsamer der Motor sich bewegen muss, wenn er beim Kalibrieren von dem Schalter weg fährt.

#define DRILL_STEP_SPEED 600
// Speed with which the drill will move, while drilling.
// Speed in steps/s

#define X_MOTOR_DIR_PIN 19
#define X_MOTOR_STEP_PIN 21
#define Y_MOTOR_DIR_PIN 5
#define Y_MOTOR_STEP_PIN 18
#define Z_MOTOR_DIR_PIN 15
#define Z_MOTOR_STEP_PIN 17
#define DRILL_MOTOR_POWER_PIN 4

#define X_SENSOR 34
#define Y_SENSOR 35
#define Z_SENSOR 32

#define CHAR_AMOUNT 26
#define COORD_AMOUNT 144

AccelStepper X_STEPPER_MOTOR(AccelStepper::DRIVER, X_MOTOR_STEP_PIN, X_MOTOR_DIR_PIN);
AccelStepper Y_STEPPER_MOTOR(AccelStepper::DRIVER, Y_MOTOR_STEP_PIN, Y_MOTOR_DIR_PIN);

static uint CHAR_INDEXES[CHAR_AMOUNT] = {0, 6, 14, 18, 27, 34, 40, 45, 51, 53, 58, 65, 69, 75, 81, 86, 91, 100, 106, 112, 116, 120, 123, 130, 135, 140};

static float CHAR_MAP[COORD_AMOUNT][2] = {
  {0.0, 0.125}, {0.0, 0.875}, {1.0, 0.875},  {1.0, 0.125},  {1.0, 0.5},    {0.0, 0.5},                                              //A [6] (0-5)
  {0.0, 0.125}, {0.0, 0.875}, {1.0, 0.875},  {1.0, 0.5},    {0.0, 0.5},    {1.0, 0.5},   {1.0, 0.125}, {0.0, 0.125},                //B[8](6-13)
  {1.0, 0.125}, {0.0, 0.125}, {0.0, 0.875},  {1.0, 0.875},                                                                          //C[4](14-17)
  {0.0, 0.125}, {0.0, 0.875}, {0.75, 0.875}, {0.75, 0.75},  {1.0, 0.75},   {1.0, 0.25},  {0.75, 0.25}, {0.75, 0.125}, {0.0, 0.125}, //D[9](18-26)
  {1.0, 0.125}, {0.0, 0.125}, {0.0, 0.5},    {0.75, 0.5},   {0.0, 0.5},    {0.0, 0.875}, {1.0, 0.875},                              //E[7](27-33)
  {0.0, 0.125}, {0.0, 0.5},   {0.75, 0.5},   {0.0, 0.5},    {0.0, 0.875},  {1.0, 0.875},                                            //F[6](34-39)
  {0.5, 0.5},   {1.0, 0.5},   {1.0, 0.125},  {0.0, 0.125},  {0.0, 0.875},  {1.0, 0.875},                                            //G[6](40-45)
  {0.0, 0.125}, {0.0, 0.875}, {0.0, 0.5},    {1.0, 0.5},    {1.0, 0.875},  {1.0, 0.125},                                            //H[6](46-51)
  {0.5, 0.125}, {0.5, 0.875},                                                                                                       //I[2](52-53)
  {0.0, 0.25},  {0.0, 0.125}, {1.0, 0.125},  {1.0, 0.875},  {0.0, 0.875},                                                           //J[5](54-58)
  {0.0, 0.125}, {0.0, 0.875}, {0.0, 0.5},    {0.75, 0.875}, {0.0, 0.5}, {0.75, 0.125},                                              //K[6](59-64)
  {1.0, 0.125}, {0.0, 0.125}, {0.0, 0.875},                                                                                         //L[3](65-67)
  {0.0, 0.125}, {0.0, 0.875}, {0.5, 0.875},  {0.5, 0.5},    {0.5, 0.875},  {1.0, 0.875}, {1.0, 0.125},                              //M[7](68-74)
  {0.0, 0.125}, {0.0, 0.875}, {0.5, 0.875},  {0.5, 0.125},  {1.0, 0.125},  {1.0, 0.875},                                            //N[6](75-80)
  {0.0, 0.125}, {0.0, 0.875}, {1.0, 0.875},  {1.0, 0.125},  {0.0, 0.125},                                                           //O[5](81-85)
  {0.0, 0.125}, {0.0, 0.875}, {1.0, 0.875},  {1.0, 0.5},    {0.0, 0.5},                                                             //P[5](86-90)
  {0.0, 0.125}, {0.0, 0.875}, {1.0, 0.875},  {1.0, 0.125},  {0.75, 0.125}, {0.75, 0.25}, {0.75, 0.0}, {0.75, 0.125}, {0.0, 0.125},  //Q[9](91-99)
  {0.0, 0.125}, {0.0, 0.875}, {1.0, 0.875},  {1.0, 0.5},    {0.0, 0.5},    {1.0, 0.125},                                            //R[6](100-105)
  {0.0, 0.125}, {1.0, 0.125}, {1.0, 0.5},    {0.0, 0.5},    {0.0, 0.875},  {1.0, 0.875},                                            //S[6](106-111)
  {0.5, 0.125}, {0.5, 0.875}, {0.0, 0.875},  {1.0, 0.875},                                                                          //T[4](112-115)
  {0.0, 0.875}, {0.0, 0.125}, {1.0, 0.125},  {1.0, 0.875},                                                                          //U[4](116-119)
  {0.0, 0.875}, {0.5, 0.125}, {1.0, 0.875},                                                                                         //V[3](120-122)
  {0.0, 0.875}, {0.0, 0.125}, {0.5, 0.125},  {0.5, 0.5},    {0.5, 0.125},  {1.0, 0.125}, {1.0, 0.875},                              //W[7](123-129)
  {0.0, 0.875}, {1.0, 0.125}, {0.5, 0.5},    {0.0, 0.125},  {1.0, 0.875},                                                           //X[5](130-134)
  {0.0, 0.875}, {0.5, 0.5},   {1.0, 0.875},  {0.5, 0.5},    {0.5, 0.125},                                                           //Y[5](135-139)
  {1.0, 0.125}, {0.0, 0.125}, {1.0, 0.875},  {0.0, 0.875}                                                                           //Z[4](140-143)
};


static float positionX = 0.0f;
static float positionY = 0.0f;
static bool drillLowered = false;

void setup()
{
  pinMode(X_MOTOR_STEP_PIN, OUTPUT);
  pinMode(X_MOTOR_DIR_PIN, OUTPUT);
  pinMode(Y_MOTOR_STEP_PIN, OUTPUT);
  pinMode(Y_MOTOR_DIR_PIN, OUTPUT);
  pinMode(Z_MOTOR_STEP_PIN, OUTPUT);
  pinMode(Z_MOTOR_DIR_PIN, OUTPUT);

  pinMode(DRILL_MOTOR_POWER_PIN, OUTPUT);

  pinMode(X_SENSOR, INPUT);
  pinMode(Y_SENSOR, INPUT);
  pinMode(Z_SENSOR, INPUT);

  X_STEPPER_MOTOR.setSpeed(DRILL_STEP_SPEED);
  Y_STEPPER_MOTOR.setSpeed(DRILL_STEP_SPEED);
  X_STEPPER_MOTOR.setAcceleration(DRILL_STEP_SPEED);
  Y_STEPPER_MOTOR.setAcceleration(DRILL_STEP_SPEED);

  Serial.begin(115200);
}

void loop()
{
}

// ========
/* Utils */
// ========

float** charTooCoords(char c) {
  int asciiChar = (int) c;
  uint charIndex = asciiChar - 65;  // 'A' -> Index: 0

  uint startIndex = CHAR_INDEXES[charIndex];
  uint endIndex = startIndex + getCoordLength(c);
  float** charCoords = new float* [endIndex - startIndex];

  Serial.println(startIndex);
  Serial.println(endIndex);
  for (int i = startIndex; i < endIndex; i++) {
    charCoords[i - startIndex] = CHAR_MAP[i];
  }

  return charCoords;
}

uint getCoordLength(char c) {
  int asciiChar = (int) c;
  uint charIndex = asciiChar - 65;  // 'A' -> Index: 0

  uint startIndex = CHAR_INDEXES[charIndex];
  uint endIndex;
  if(charIndex >= (CHAR_AMOUNT - 1)) {
    endIndex = COORD_AMOUNT;
  }
  else {
    endIndex = CHAR_INDEXES[charIndex + 1];
  }

  return (endIndex - startIndex);
}

// Val kann nicht kleiner als lowerBounds oder höher als topBounds werden
float clamp(float val, float lowerBounds, float topBounds) {
  if (val < lowerBounds) return lowerBounds;
  else if (val > topBounds) return topBounds;
  else return val;
}

bool isLettersOnly(String txt) {
  for (int i = 0; i < txt.length(); i++) {
  	if (!isAlpha(txt[i])) return false;
  }
  
	return true;
}

// ===================
/* Drill Char Logic */
// ===================

void drillChar(char c, uint charIndex) {
  // Get the (0 0) coords of the char field.
  // Christophs function

  // Assuming that the drill is at (0 0) of the char field:
  float** charCoords = charTooCoords(c);
  uint coordLen = getCoordLength(c);

  moveDrillInCharField(charCoords, coordLen);
}

void moveDrillInCharField(float** coords, uint len) {
  stopDrill();

  float charPosX = 0.0;
  float charPosY = 0.0;

  for(int i = 0; i < len; i++) {
    float* charCoord = coords[i];

    // Values between 0 and 1
    float xOffset = (charCoord[0] - charPosX);
    float yOffset = (charCoord[1] - charPosY);

    int xSteps = (X_STEP_CHAR_COUNT * xOffset);
    int ySteps = (Y_STEP_CHAR_COUNT * yOffset);

    float xSpeed = DRILL_STEP_SPEED;
    float ySpeed = DRILL_STEP_SPEED * (ySteps / xSteps);

    X_STEPPER_MOTOR.move(xSteps);
    Y_STEPPER_MOTOR.move(ySteps);

    Y_STEPPER_MOTOR.setSpeed(ySpeed);

    AccelStepper checkStepper = (xSteps >= ySteps) ? X_STEPPER_MOTOR : Y_STEPPER_MOTOR;

    // Blocking loop, da #run / #runSpeed nicht den Thread blockiert
    while(checkStepper.distanceToGo() > 0) {
      X_STEPPER_MOTOR.runSpeed();
      Y_STEPPER_MOTOR.runSpeed();

      X_STEPPER_MOTOR.setSpeed(xSpeed);
      Y_STEPPER_MOTOR.setSpeed(ySpeed);
    }
  }

  Y_STEPPER_MOTOR.setSpeed(DRILL_STEP_SPEED);
  stopDrill();
}

// ======================
/* CONSOLE Input Logic */
/* Muss zu USB oder anderem umgeändert werden! */
// ==============================================

String getUserNameFromConsole() {
  while (Serial.available() == 0) { }
  String name = Serial.readString();
  name.toUpperCase();
  
  return name;
}

// =================
/* Movement Logic */
// =================

void stopMove() {
  digitalWrite(X_MOTOR_STEP_PIN, LOW);
  digitalWrite(Z_MOTOR_STEP_PIN, LOW);
}

void moveSteps(int steps, int stepPin, int dirPin, bool reverse) {
  if (reverse) {
    digitalWrite(dirPin, HIGH);
  }

  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(MOTOR_STEP_TIME);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(MOTOR_STEP_TIME);
  }

  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);
}

// coord: Range 0 too 1
void moveX(float coord) {
  moveXRelative((coord - positionX));
  positionX = coord;
}

// If the offset exceeds the bounds of the plate, ie. coords > 1, it will move to the bounds of the plate.
// Input can be negative to move in -x direction.
void moveXRelative(float offset) {
  offset = clamp((positionX + offset), 0, 1);
  int steps = X_STEP_COUNT * offset;
  bool reverse = (steps < 0);

  moveSteps(abs(steps), X_MOTOR_STEP_PIN, X_MOTOR_DIR_PIN, reverse);
  positionX += offset;
}

void moveY(float coord) {
  moveYRelative((coord - positionY));
  positionY = coord;
}

void moveYRelative(float offset) {
  offset = clamp((positionY + offset), 0, 1);
  int steps = Y_STEP_COUNT * offset;
  bool reverse = (steps < 0);

  moveSteps(abs(steps), Y_MOTOR_STEP_PIN, Y_MOTOR_DIR_PIN, reverse);
  positionY += offset;
}

// =====================
/* Drillhead Movement */
// =====================

void stopDrill() {
  stopMove();
  digitalWrite(DRILL_MOTOR_POWER_PIN, LOW);

  // Weird workaround
  // Bewegt sich gleich hoch
  calibrateYAxis();
  drillLowered = false;
}

void startDrill() {
  // Insert drive down logic
  drillLowered = true;
}

// =============
/* Home Logic */
// =============

void gotoHome() {
  stopDrill();

  calibrateXAxis();
  calibrateYAxis();
  // calibrateZAxis();
}

void calibrateXAxis() {
  // Move to sensor; Move away; Move towards... slower; Move away... slower
  moveToSensor(X_MOTOR_STEP_PIN, X_SENSOR,X_MOTOR_DIR_PIN, MOTOR_SLOW_STEP_TIME);
  moveFromSensor(X_MOTOR_STEP_PIN, X_MOTOR_DIR_PIN, X_SENSOR, MOTOR_SLOW_STEP_TIME);
  moveToSensor(X_MOTOR_STEP_PIN, X_SENSOR,X_MOTOR_DIR_PIN, MOTOR_SLOW_STEP_TIME * MOTOR_SLOW_STEP_FACTOR);
  moveFromSensor(X_MOTOR_STEP_PIN, X_MOTOR_DIR_PIN, X_SENSOR, MOTOR_SLOW_STEP_TIME * MOTOR_SLOW_STEP_FACTOR);
}

void calibrateZAxis() {
  // Definitely didn't copy paste it
  moveToSensor(Z_MOTOR_STEP_PIN, Z_SENSOR,Z_MOTOR_DIR_PIN, MOTOR_SLOW_STEP_TIME);
  moveFromSensor(Z_MOTOR_STEP_PIN, Z_MOTOR_DIR_PIN, Z_SENSOR, MOTOR_SLOW_STEP_TIME);
  moveToSensor(Z_MOTOR_STEP_PIN, Z_SENSOR,Z_MOTOR_DIR_PIN, MOTOR_SLOW_STEP_TIME * 2);
  moveFromSensor(Z_MOTOR_STEP_PIN, Z_MOTOR_DIR_PIN, Z_SENSOR, MOTOR_SLOW_STEP_TIME * 2);
}

void calibrateYAxis() {
  // Definitely didn't copy paste it... I would never!
  moveToSensor(Y_MOTOR_STEP_PIN, Y_SENSOR,Y_MOTOR_DIR_PIN, MOTOR_SLOW_STEP_TIME);
  moveFromSensor(Y_MOTOR_STEP_PIN, Y_MOTOR_DIR_PIN, Y_SENSOR, MOTOR_SLOW_STEP_TIME);
  moveToSensor(Y_MOTOR_STEP_PIN, Y_SENSOR,Y_MOTOR_DIR_PIN, MOTOR_SLOW_STEP_TIME * MOTOR_SLOW_STEP_FACTOR);
  moveFromSensor(Y_MOTOR_STEP_PIN, Y_MOTOR_DIR_PIN, Y_SENSOR, MOTOR_SLOW_STEP_TIME * MOTOR_SLOW_STEP_FACTOR);
}

void moveToSensor(int motorPin, int sensorPin,int directionPin, int speed) {
  digitalWrite(directionPin, LOW);
  while(digitalRead(sensorPin) == 0) {
    moveSlowly(motorPin, speed);
  }
}

void moveFromSensor(int motorPin, int directionPin, int sensorPin, int speed) {
  digitalWrite(directionPin, HIGH);

  while(digitalRead(sensorPin) == 1) {
    moveSlowly(motorPin, speed);
  }

  digitalWrite(directionPin, LOW);
}

void moveSlowly(int motorPin, int stepSpeed) {
    digitalWrite(motorPin, HIGH);
    delayMicroseconds(stepSpeed);
    digitalWrite(motorPin, LOW);
    delayMicroseconds(stepSpeed);
}
