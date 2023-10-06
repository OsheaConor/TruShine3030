#define X_STEP_COUNT 800
#define Y_STEP_COUNT 400

#define MOTOR_STEP_TIME 200
#define X__MOTOR_DIR_PIN 19
#define X_MOTOR_STEP_PIN 21
#define Y_MOTOR_DIR_PIN 5
#define Y_MOTOR_STEP_PIN 18

static float positionX = 0.0f;
static float positionZ = 0.0f;
static bool drillLowered = false;

void setup()
{
  pinMode(X_MOTOR_STEP_PIN, OUTPUT);
  pinMode(Y_MOTOR_STEP_PIN, OUTPUT);

  Serial.begin(115200);
}

void loop()
{
  Serial.println("Bitte gebe einen namen ein:");
  String name = getUserNameFromConsole();
  if (!isLettersOnly(name)) {
    Serial.println("Der Name darf nur Buchstaben enthalten!");
    return;
  }
  
}

/* CONSOLE Input Logic */
/* Muss zu USB oder anderem umgeändert werden! */

String getUserNameFromConsole() {
  while (Serial.available() == 0) { }
  String name = Serial.readString();
  name.toUpperCase();
  
  return name;
}

bool isLettersOnly(String txt) {
  for (int i = 0; i < txt.length(); i++) {
  	if (!isAlpha(txt[i])) return false;
  }
  
	return true;
}

/* Movement Logic */

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

}

// If the offset exceeds the bounds of the plate, ie. coords > 1, it will move to the bounds of the plate.
// Input can be negative to move in -x direction.
void moveXRelative(float offset) {
  int steps = X_STEP_COUNT * offset;
  bool reverse = (steps < 0);

  moveSteps(abs(steps), X_MOTOR_STEP_PIN, X_MOTOR_DIR_PIN, reverse);
}

void moveY(float coord) {

}

void moveYRelative(float offset) {
  int steps = Y_STEP_COUNT * offset;
  bool reverse = (steps < 0);

  moveSteps(abs(steps), Y_MOTOR_STEP_PIN, Y_MOTOR_DIR_PIN, reverse);
}

/* Home Logic */

void gotoHome() {

}