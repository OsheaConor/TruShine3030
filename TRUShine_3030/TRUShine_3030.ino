#define X_STEP_COUNT 800
#define Z_STEP_COUNT 400
// Fiktive Werte!

#define MOTOR_STEP_TIME 200
#define MOTOR_SLOW_STEP_TIME 800
#define MOTOR_SLOW_STEP_FACTOR 2
// Zahlen müssen nochmal überarbeitet werden!
// MOTOR_SLOW_STEP_FACTOR besagt, wie viel langsamer der Motor sich bewegen muss, wenn er beim Kalibrieren von dem Schalter weg fährt.

#define X_MOTOR_DIR_PIN 19
#define X_MOTOR_STEP_PIN 21
#define Y_MOTOR_DIR_PIN 15
#define Y_MOTOR_STEP_PIN 17
#define Z_MOTOR_DIR_PIN 5
#define Z_MOTOR_STEP_PIN 18
#define DRILL_MOTOR_POWER_PIN 4

#define X_SENSOR 34
#define Z_SENSOR 35
#define Y_SENSOR 32

static float positionX = 0.0f;
static float positionZ = 0.0f;
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

  Serial.begin(115200);
}

void loop()
{
  gotoHome();
  delay(200);

  /*
  Serial.println("Bitte gebe einen namen ein:");
  String name = getUserNameFromConsole();
  if (!isLettersOnly(name)) {
    Serial.println("Der Name darf nur Buchstaben enthalten!");
    return;
  }
  */
}

// ========
/* Utils */
// ========

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

void moveZ(float coord) {
  moveZRelative((coord - positionZ));
  positionZ = coord;
}

void moveZRelative(float offset) {
  offset = clamp((positionZ + offset), 0, 1);
  int steps = Z_STEP_COUNT * offset;
  bool reverse = (steps < 0);

  moveSteps(abs(steps), Z_MOTOR_STEP_PIN, Z_MOTOR_DIR_PIN, reverse);
  positionZ += offset;
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
  calibrateZAxis();
  calibrateYAxis();
}

void calibrateXAxis() {
  // Move to sensor; Move away; Move towards... slower; Move away... slower
  moveToSensor(X_MOTOR_STEP_PIN, X_SENSOR, MOTOR_SLOW_STEP_TIME);
  moveFromSensor(X_MOTOR_STEP_PIN, X_MOTOR_DIR_PIN, X_SENSOR, MOTOR_SLOW_STEP_TIME);
  moveToSensor(X_MOTOR_STEP_PIN, X_SENSOR, MOTOR_SLOW_STEP_TIME * 2);
  moveFromSensor(X_MOTOR_STEP_PIN, X_MOTOR_DIR_PIN, X_SENSOR, MOTOR_SLOW_STEP_TIME * 2);
}

void calibrateZAxis() {
  // Definitely didn't copy paste it
  moveToSensor(Z_MOTOR_STEP_PIN, Z_SENSOR, MOTOR_SLOW_STEP_TIME);
  moveFromSensor(Z_MOTOR_STEP_PIN, Z_MOTOR_DIR_PIN, Z_SENSOR, MOTOR_SLOW_STEP_TIME);
  moveToSensor(Z_MOTOR_STEP_PIN, Z_SENSOR, MOTOR_SLOW_STEP_TIME * 2);
  moveFromSensor(Z_MOTOR_STEP_PIN, Z_MOTOR_DIR_PIN, Z_SENSOR, MOTOR_SLOW_STEP_TIME * 2);
}

void calibrateYAxis() {
  // Definitely didn't copy paste it... I would never!
  moveToSensor(Y_MOTOR_STEP_PIN, Y_SENSOR, MOTOR_SLOW_STEP_TIME);
  moveFromSensor(Y_MOTOR_STEP_PIN, Y_MOTOR_DIR_PIN, Y_SENSOR, MOTOR_SLOW_STEP_TIME);
  moveToSensor(Y_MOTOR_STEP_PIN, Y_SENSOR, MOTOR_SLOW_STEP_TIME * MOTOR_SLOW_STEP_FACTOR);
  moveFromSensor(Y_MOTOR_STEP_PIN, Y_MOTOR_DIR_PIN, Y_SENSOR, MOTOR_SLOW_STEP_TIME * MOTOR_SLOW_STEP_FACTOR);
}

void moveToSensor(int motorPin, int sensorPin, int speed) {
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



