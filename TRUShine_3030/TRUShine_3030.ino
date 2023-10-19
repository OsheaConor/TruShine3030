#include <AccelStepper.h>

// Eine rev des Stepper motors braucht 800 steps 

#define X_STEP_COUNT 800
#define Y_STEP_COUNT 400

#define X_STEP_CHAR_COUNT 50
#define Y_STEP_CHAR_COUNT 100
// Fiktive Werte!

#define MOTOR_SLOW_STEP_SPEED 100
#define MOTOR_SLOW_STEP_FACTOR 2
// Zahlen müssen nochmal überarbeitet werden!
// MOTOR_SLOW_STEP_FACTOR besagt, wie viel langsamer der Motor sich bewegen muss, wenn er beim Kalibrieren von dem Schalter weg fährt.
// Werte in steps/s

#define MOVE_STEP_SPEED 900
#define DRILL_STEP_SPEED 800
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
AccelStepper Z_STEPPER_MOTOR(AccelStepper::DRIVER, Z_MOTOR_STEP_PIN, Z_MOTOR_DIR_PIN);

static uint CHAR_INDEXES[CHAR_AMOUNT] = {0, 6, 14, 18, 27, 34, 40, 45, 51, 53, 58, 65, 69, 75, 81, 86, 91, 100, 106, 112, 116, 120, 123, 130, 135, 140};

static float CHAR_MAP[COORD_AMOUNT][2] = {
  {0.0, 0.125}, {0.0, 0.875}, {1.0, 0.875},  {1.0, 0.125},  {1.0, 0.5},    {0.0, 0.5},                                              //A[6] (0-5)
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
  Serial.begin(115200);

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

  X_STEPPER_MOTOR.setAcceleration(DRILL_STEP_SPEED);
  Y_STEPPER_MOTOR.setAcceleration(DRILL_STEP_SPEED);
  X_STEPPER_MOTOR.setMaxSpeed(DRILL_STEP_SPEED);
  Y_STEPPER_MOTOR.setMaxSpeed(DRILL_STEP_SPEED);
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

void drillChar(char c) {
  // Get the (0 0) coords of the char field.
  // Get scaling factor of char fields
  // Christophs functions

  // Move to 0 0 of the char field
  // moveToOnBoard(getCharOrigin(charIndex), someConstYValue);  <Pseudo>

  // Assuming that the drill is at (0 0) of the char field:
  float** charCoords = charTooCoords(c);
  uint coordLen = getCoordLength(c);

  float posCharX = 0.0f;
  float posCharY = 0.0f;

  startDrill();
  for (int i = 0; i < coordLen; i++) {
    float xCoord = charCoords[i][0];
    float yCoord = charCoords[i][1];

    moveRelativeInCharField((xCoord - posCharX), (yCoord - posCharY));

    posCharX = xCoord;
    posCharY = yCoord;
  }

  stopMove();
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
  X_STEPPER_MOTOR.stop();
  Y_STEPPER_MOTOR.stop();
}

void moveSteps(int xSteps, int ySteps) {
  float* speeds = configureMotors(xSteps, ySteps);

  float xSpeed = speeds[0];
  float ySpeed = speeds[1];

  X_STEPPER_MOTOR.move(abs(xSteps));
  Y_STEPPER_MOTOR.move(abs(ySteps));

  X_STEPPER_MOTOR.setPinsInverted((xSteps < 0), false, false);
  Y_STEPPER_MOTOR.setPinsInverted((ySteps < 0), false, false);

  AccelStepper* controlMotorPtr = (xSteps == 0) ? &Y_STEPPER_MOTOR : &X_STEPPER_MOTOR;

  while(controlMotorPtr->distanceToGo() > 0) {
    X_STEPPER_MOTOR.setSpeed(xSpeed);
    Y_STEPPER_MOTOR.setSpeed(ySpeed);
    
    X_STEPPER_MOTOR.runSpeed();
    Y_STEPPER_MOTOR.runSpeed();
  }

  X_STEPPER_MOTOR.setPinsInverted(false, false, false);
  Y_STEPPER_MOTOR.setPinsInverted(false, false, false);
}

float* configureMotors(int xSteps, int ySteps) {
  bool xHigh = (abs(xSteps) > abs(ySteps)) ? true : false;
  AccelStepper* throttledMotor = (xHigh) ? &Y_STEPPER_MOTOR : &X_STEPPER_MOTOR;
  AccelStepper* nonThrottledMotor = (xHigh) ? &X_STEPPER_MOTOR : &Y_STEPPER_MOTOR;
  float throttle = (xHigh) ? ((float) ySteps / (float) xSteps) : ((float) xSteps / (float) ySteps);
  throttle = abs(throttle);

  float throttledMotorSpeed = (DRILL_STEP_SPEED * throttle);
  throttledMotor->setSpeed(throttledMotorSpeed);
  throttledMotor->setAcceleration(throttledMotorSpeed);

  nonThrottledMotor->setSpeed(DRILL_STEP_SPEED);
  
  float xSpeed = (xHigh) ? DRILL_STEP_SPEED : throttledMotorSpeed;
  float ySpeed = (xHigh) ? throttledMotorSpeed : DRILL_STEP_SPEED;

  return new float[2] {xSpeed, ySpeed};
}

void moveRelativeOnBoard(float x, float y) {
  int xSteps = X_STEP_COUNT * x;
  int ySteps = Y_STEP_COUNT * y;

  positionX += x;
  positionY += y;

  moveSteps(xSteps, ySteps);
}

void moveToOnBoard(float x, float y) {
  moveRelativeOnBoard((x - positionX), (y - positionY));
}

void moveRelativeInCharField(float x, float y) {
  int xSteps = X_STEP_CHAR_COUNT * x;
  int ySteps = Y_STEP_CHAR_COUNT * y;

  positionX += (x * (X_STEP_COUNT / X_STEP_CHAR_COUNT));
  positionY += (y * (Y_STEP_COUNT / Y_STEP_CHAR_COUNT));

  moveSteps(xSteps, ySteps);
}

// =====================
/* Drillhead Movement */
// =====================

void stopDrill() {
  stopMove();
  // Rewrite with lib
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
  delay(1000);    // Just in case

  calibrateXAxis();
  calibrateYAxis();
  calibrateZAxis();
}

void calibrateXAxis() {
  // Move to sensor; Move away; Move towards... slower; Move away... slower
  moveToSensor(&X_STEPPER_MOTOR, X_SENSOR, MOTOR_SLOW_STEP_SPEED);
  moveSteps(0, 800);
  moveToSensor(&X_STEPPER_MOTOR, X_SENSOR, MOTOR_SLOW_STEP_SPEED / MOTOR_SLOW_STEP_FACTOR);
  moveFromSensor(&X_STEPPER_MOTOR, X_SENSOR, MOTOR_SLOW_STEP_SPEED / MOTOR_SLOW_STEP_FACTOR);
}

void calibrateZAxis() {
  // Definitely didn't copy paste it
  moveToSensor(&Z_STEPPER_MOTOR, Z_SENSOR, MOTOR_SLOW_STEP_SPEED);
  moveFromSensor(&Z_STEPPER_MOTOR, Z_SENSOR, MOTOR_SLOW_STEP_SPEED);
  moveToSensor(&Z_STEPPER_MOTOR, Z_SENSOR, MOTOR_SLOW_STEP_SPEED);
  moveFromSensor(&Z_STEPPER_MOTOR, Z_SENSOR, MOTOR_SLOW_STEP_SPEED / MOTOR_SLOW_STEP_FACTOR);
}

void calibrateYAxis() {
  // Definitely didn't copy paste it... I would never!
  moveToSensor(&Y_STEPPER_MOTOR, Y_SENSOR, MOTOR_SLOW_STEP_SPEED);
  moveSteps(800, 0);
  moveToSensor(&Y_STEPPER_MOTOR, Y_SENSOR, MOTOR_SLOW_STEP_SPEED / MOTOR_SLOW_STEP_FACTOR);
  moveFromSensor(&Y_STEPPER_MOTOR, Y_SENSOR, MOTOR_SLOW_STEP_SPEED / MOTOR_SLOW_STEP_FACTOR);
}

void moveToSensor(AccelStepper *motorPtr, int sensorPin, int speed) {
  motorPtr->setSpeed(speed);
  motorPtr->setPinsInverted(true, false, false);

  while(digitalRead(sensorPin) == 0) {
    motorPtr->runSpeed();
    motorPtr->setSpeed(speed);
  }

  motorPtr->setPinsInverted(false, false, false);
}

void moveFromSensor(AccelStepper *motorPtr, int sensorPin, int speed) {
  motorPtr->setSpeed(speed);
  motorPtr->setPinsInverted(false, false, false);

  while(digitalRead(sensorPin) == 1) {
    motorPtr->runSpeed();
    motorPtr->setSpeed(speed);
  }
}
