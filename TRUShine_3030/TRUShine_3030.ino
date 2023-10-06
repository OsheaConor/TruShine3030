#define X_STEP_COUNT 800
#define Y_STEP_COUNT 400
// Fiktive Werte!

#define MOTOR_STEP_TIME 200
#define MOTOR_SLOW_STEP_TIME 800
// Zahlen müssen nochmal überarbeitet werden!

#define X__MOTOR_DIR_PIN 19
#define X_MOTOR_STEP_PIN 21
#define Y_MOTOR_DIR_PIN 5
#define Y_MOTOR_STEP_PIN 18

#define X_SENSOR 34
#define Y_SENSOR 35

static float positionX = 0.0f;
static float positionZ = 0.0f;
static bool drillLowered = false;

void setup()
{
  pinMode(X_MOTOR_STEP_PIN, OUTPUT);
  pinMode(Y_MOTOR_STEP_PIN, OUTPUT);
  pinMode(X_SENSOR, INPUT);
  pinMode(Y_SENSOR, INPUT);

  Serial.begin(115200);
}

void loop()
{
  gotoHome();

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

void moveSteps(int steps, int stepPin, int dirPin, bool reverse) {
  if (reverse) {
    digitalWrite(dirPing, HIGH);
  }

  for (int i = 0; i < steps; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(MOTOR_STEP_TIME);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(MOTOR_STEP_TIME);
  }

  digitalWrite(stepPin, low);
  digitalWrite(dirPin, low);
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

// =============
/* Home Logic */
// =============

void gotoHome() {
  calibrateXAxis();
  calibrateYAxis();
}

void calibrateXAxis() {
  // Move to sensor; Move away; Move towards... slower; Move away... slower
  moveToSensor(X_MOTOR_STEP_PIN, X_SENSOR, MOTOR_SLOW_STEP_TIME);
  moveFromSensor(X_MOTOR_STEP_PIN, X__MOTOR_DIR_PIN, X_SENSOR, MOTOR_SLOW_STEP_TIME);
  moveToSensor(X_MOTOR_STEP_PIN, X_SENSOR, MOTOR_SLOW_STEP_TIME * 2);
  moveFromSensor(X_MOTOR_STEP_PIN, X__MOTOR_DIR_PIN, X_SENSOR, MOTOR_SLOW_STEP_TIME * 2);
}

void calibrateYAxis() {
  // Definitely didn't copy paste it... I would never!
  moveToSensor(Y_MOTOR_STEP_PIN, Y_SENSOR, MOTOR_SLOW_STEP_TIME);
  moveFromSensor(Y_MOTOR_STEP_PIN, Y__MOTOR_DIR_PIN, Y_SENSOR, MOTOR_SLOW_STEP_TIME);
  moveToSensor(Y_MOTOR_STEP_PIN, Y_SENSOR, MOTOR_SLOW_STEP_TIME * 2);
  moveFromSensor(Y_MOTOR_STEP_PIN, Y__MOTOR_DIR_PIN, Y_SENSOR, MOTOR_SLOW_STEP_TIME * 2);
}

void moveToSensor(int motorPin, int sensorPin, int speed) {
  while(digitalRead(sensorPin) == 0) {
    moveSlowly(motorPin, speed);
  }
}

void moveFromSensor(int motorPin, int directionPin, int sensorPin, int speed) {
  digitalWrite(directionPin, HIGH);

  while(digitalRead(sensorPin) == 0) {
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



