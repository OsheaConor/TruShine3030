#include <AccelStepper.h>
#include "BluetoothSerial.h"

static const String DEVICE_NAME = "ESP32-BT-Slave";

static const char BT_USER_NAME_END_TOKEN = '~';
static const char BT_RESET_TOKEN = '!';
static const char BT_OUTPUT_TOKEN = '>';

static const char BT_CHAR_FINISHED = '&';
static const char BT_NAME_FINISHED = '/';


// Check if Bluetooth is available
#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
  #error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

// Check Serial Port Profile
#if !defined(CONFIG_BT_SPP_ENABLED)
  #error Serial Port Profile for Bluetooth is not available or not enabled. It is only available for the ESP32 chip.
#endif

// Eine rev des Stepper motors braucht 800 steps 

#define X_STEP_COUNT 6000
#define Y_STEP_COUNT 4000

#define X_STEP_CHAR_COUNT 700
#define Y_STEP_CHAR_COUNT 1400
// Reale Werte

#define MOTOR_SLOW_STEP_SPEED 800
#define MOTOR_SLOW_STEP_FACTOR 8
// MOTOR_SLOW_STEP_FACTOR besagt, wie viel langsamer der Motor sich bewegen muss, wenn er beim Kalibrieren von dem Schalter weg fährt.
// Werte in steps/s

#define MOVE_STEP_SPEED 900
#define DRILL_STEP_SPEED 800
// Speed with which the drill will move, while drilling.
// Speed in steps/s

#define DRILL_STARTUP_TIME_MS 10000
#define DRILL_STARTUP_RAISE_STEPS 800
#define DRILL_CALIBRATE_STEP_TIME_us 500
// Amount of steps too move up, when calibrating the drills distance too the plate
#define DRILL_SENSOR_X_STEPS 100
#define DRILL_SENSOR_Y_STEPS 100
#define STEPS_TO_PLATE 200
// Fiktive Werte!

#define X_MOTOR_DIR_PIN 19
#define X_MOTOR_STEP_PIN 21
#define Y_MOTOR_DIR_PIN 5
#define Y_MOTOR_STEP_PIN 18
#define Z_MOTOR_DIR_PIN 15
#define Z_MOTOR_STEP_PIN 17

#define DRILL_ENABLE_PIN 2
#define DRILL_MOTOR_POWER_PIN 4

#define X_SENSOR 34
#define Y_SENSOR 35
#define Z_SENSOR 32

#define CHAR_AMOUNT 26
#define COORD_AMOUNT 144

#define MAX_LETTERS 12

BluetoothSerial SerialBT;
AccelStepper X_STEPPER_MOTOR(AccelStepper::DRIVER, X_MOTOR_STEP_PIN, X_MOTOR_DIR_PIN);
AccelStepper Y_STEPPER_MOTOR(AccelStepper::DRIVER, Y_MOTOR_STEP_PIN, Y_MOTOR_DIR_PIN);
AccelStepper Z_STEPPER_MOTOR(AccelStepper::DRIVER, Z_MOTOR_STEP_PIN, Z_MOTOR_DIR_PIN);

                                      // A  B  C   D   E   F   G   H   I   J   K   L   M   N   O   P   Q   R    S    T    U    V    W    X    Y    Z
static uint CHAR_INDEXES[CHAR_AMOUNT] = {0, 5, 15, 19, 26, 33, 39, 45, 51, 53, 58, 64, 67, 72, 76, 81, 86, 95, 101, 107, 111, 115, 118, 123, 128, 133};

static float CHAR_MAP[COORD_AMOUNT][2] = {
  {0.0, 0.125}, {0.5, 0.875}, {1.0, 0.125},  {0.75, 0.5},   {0.25, 0.5},                                                                                          //A[6] (0-4)
  {0.0, 0.125}, {0.0, 0.875}, {0.75, 0.875}, {1.0, 0.75},   {1.0, 0.5},    {0.0, 0.5},    {0.75, 0.5}, {1.0, 0.375},{1.0, 0.125}, {0.0, 0.125},                   //B[10] (5-14)
  {1.0, 0.125}, {0.0, 0.125}, {0.0, 0.875},  {1.0, 0.875},                                                                                                        //C[4] (15-18)
  {0.0, 0.125}, {0.0, 0.875}, {0.75, 0.875}, {1.0, 0.75},   {1.0, 0.25},   {0.75, 0.125}, {0.0, 0.125},                                                           //D[7] (19-25)
  {1.0, 0.125}, {0.0, 0.125}, {0.0, 0.5},    {0.75, 0.5},   {0.0, 0.5},    {0.0, 0.875},  {1.0, 0.875},                                                           //E[7] (26-32)
  {0.0, 0.125}, {0.0, 0.5},   {0.75, 0.5},   {0.0, 0.5},    {0.0, 0.875},  {1.0, 0.875},                                                                          //F[6] (33-38)
  {0.5, 0.5},   {1.0, 0.5},   {1.0, 0.125},  {0.0, 0.125},  {0.0, 0.875},  {1.0, 0.875},                                                                          //G[6] (39-44)
  {0.0, 0.125}, {0.0, 0.875}, {0.0, 0.5},    {1.0, 0.5},    {1.0, 0.875},  {1.0, 0.125},                                                                          //H[6] (45-50)
  {0.5, 0.125}, {0.5, 0.875},                                                                                                                                     //I[2] (51-52)
  {0.0, 0.25},  {0.0, 0.125}, {1.0, 0.125},  {1.0, 1.0},    {0.0, 1.0},                                                                                           //J[5] (53-57)
  {0.0, 0.125}, {0.0, 0.875}, {0.0, 0.5},    {0.75, 0.875}, {0.0, 0.5}, {0.75, 0.125},                                                                            //K[6] (58-63)
  {1.0, 0.125}, {0.0, 0.125}, {0.0, 0.875},                                                                                                                       //L[3] (64-66)
  {0.0, 0.125}, {0.0, 0.875}, {0.5, 0.5},    {1.0, 0.875},  {1.0, 0.125},                                                                                         //M[5] (67-71)
  {0.0, 0.125}, {0.0, 0.875}, {1.0, 0.125},  {1.0, 0.875},                                                                                                        //N[4] (72-75)
  {0.0, 0.125}, {0.0, 0.875}, {1.0, 0.875},  {1.0, 0.125},  {0.0, 0.125},                                                                                         //O[5] (76-80)
  {0.0, 0.125}, {0.0, 0.875}, {1.0, 0.875},  {1.0, 0.5},    {0.0, 0.5},                                                                                           //P[5] (81-85)
  {0.0, 0.125}, {0.0, 0.875}, {1.0, 0.875},  {1.0, 0.125},  {0.75, 0.125}, {0.75, 0.25}, {0.75, 0.0}, {0.75, 0.125}, {0.0, 0.125},                                //Q[9] (86-94)
  {0.0, 0.125}, {0.0, 0.875}, {1.0, 0.875},  {1.0, 0.5},    {0.0, 0.5},    {1.0, 0.125},                                                                          //R[6] (95-100)
  {0.0, 0.125}, {1.0, 0.125}, {1.0, 0.5},    {0.0, 0.5},    {0.0, 0.875},  {1.0, 0.875},                                                                          //S[6] (101-106)
  {0.5, 0.125}, {0.5, 0.875}, {0.0, 0.875},  {1.0, 0.875},                                                                                                        //T[4] (107-110)
  {0.0, 0.875}, {0.0, 0.125}, {1.0, 0.125},  {1.0, 0.875},                                                                                                        //U[4] (111-114)
  {0.0, 0.875}, {0.5, 0.125}, {1.0, 0.875},                                                                                                                       //V[3] (115-117)
  {0.0, 0.875}, {0.25, 0.125},{0.5, 0.5},    {0.75, 0.125}, {1.0, 0.875},                                                                                         //W[5] (118-122)
  {0.0, 0.875}, {1.0, 0.125}, {0.5, 0.5},    {0.0, 0.125},  {1.0, 0.875},                                                                                         //X[5] (123-127)
  {0.0, 0.875}, {0.5, 0.5},   {1.0, 0.875},  {0.5, 0.5},    {0.5, 0.125},                                                                                         //Y[5] (128-132)
  {1.0, 0.125}, {0.0, 0.125}, {1.0, 0.875},  {0.0, 0.875}                                                                                                         //Z[4] (133-136)
};


const float einheit = 1.0/60.0;     //1mm in Koordinate umgerechnet

static float positionX = 0.0f;
static float positionY = 0.0f;
static bool drillLowered = false;

void setup()
{
  
  Serial.begin(115200);
  SerialBT.begin(DEVICE_NAME);

  pinMode(X_MOTOR_STEP_PIN, OUTPUT);
  pinMode(X_MOTOR_DIR_PIN, OUTPUT);
  pinMode(Y_MOTOR_STEP_PIN, OUTPUT);
  pinMode(Y_MOTOR_DIR_PIN, OUTPUT);
  pinMode(Z_MOTOR_STEP_PIN, OUTPUT);
  pinMode(Z_MOTOR_DIR_PIN, OUTPUT);

  pinMode(DRILL_MOTOR_POWER_PIN, OUTPUT);
  pinMode(DRILL_ENABLE_PIN, OUTPUT);

  pinMode(X_SENSOR, INPUT);
  pinMode(Y_SENSOR, INPUT);
  pinMode(Z_SENSOR, INPUT);

  X_STEPPER_MOTOR.setMaxSpeed(MOVE_STEP_SPEED);
  Y_STEPPER_MOTOR.setMaxSpeed(MOVE_STEP_SPEED);

  // startCommandInputListener();
  // calibrateDrillDistance();
}
int i;
void loop()
  {
//   if(Serial.available()) {
//     SerialBT.write(Serial.readString().charAt(0));
//   }
 
//   if(SerialBT.available()) {
//     Serial.println(SerialBT.readString());
//   }
  // Setup
  String s = getUserName();
  char* chars = new char[s.length() + 1];
  strcpy(chars, s.c_str());
  
  moveToPlexiStart();
  
  // // startDrill();
  // // delay(10000);
  // // stopDrill();
    

  //Main
  drillUserName(chars, s.length());

  // Finish
  delay(2000);
  moveToAusgabe();
  delay(2000);
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

float breiteBox(int il) {
  float breite = ((float)(60.0 - (il + 1))/ (float) il );
  breite = min(breite, (float)7.0);
  return breite;
}

float* firstboxGerade(int il) {
  float einheit = (1.0 / 60.0);
  float breite = breiteBox(il);

  float b1 = ((float)breite * ((float)il/2)+(((float)(il + 1)/2)-1));//Breite die es zu ersten Box gehe muss
  float c1 = (((float)30-b1) * einheit);//Cord der ersten 0.0 Box
  //Neue Box
  float NeueBox = ((float) breite + 1)* einheit;// Cord neue Box (Breite Box+ Abstand) vorrausetzung immer 0.0  
  float* Werte = new float[il];//verusch des arrays
  float box2 = c1;

  for (int b = 0; b < il; b++) {
    Werte[b] = (float)box2;
    box2 = ((float)c1 + NeueBox);//rechnet alle andern Boxen aus (geht, wegen dem array aber muss man programm immer neustarten)
    c1 = box2;
  }

  return Werte;
}

void reset() {
  stopAll();  // Also raises the drill! Might cause issues due to it though...
  delay(500);
  gotoHome();
}

void stopAll() {
  stopMove();
  delay(2000);
  stopDrill();
}


// =======================
/* BT Web Command input */
// =======================

  void startCommandInputListener() {
    // Missing the functionality too run this code async via Interrupts
    // Don't know if that's possible with BT, since there's no pin state change
    // Also this will probably run over an interrupt. So it'll continue after the interrupt...
    // which is bad. It should go back to the "loop"
    
    String cmdToken = SerialBT.readString();
    if(cmdToken.length() > 1) return;   // A cmdToken should only be a single char!

    char token = cmdToken.charAt(0);
    switch (token) {
      case BT_RESET_TOKEN:
        reset();
        break;
      
      case BT_OUTPUT_TOKEN:
        moveToAusgabe();
        break;
    }
  }


// ===================
/* Drill Char Logic */
// ===================

void drillUserName(char* name, int nameLen) {
  float* charStartingPoints = firstboxGerade(nameLen);
  for (int i = 0; i < nameLen; i++) {
    char c = name[i];
    float charStart = charStartingPoints[i];

    moveToOnBoard(charStart, 0.05f);
    delay(200);
    drillChar(c);
    SerialBT.write(BT_CHAR_FINISHED);
    delay(500);
  }

  SerialBT.write(BT_NAME_FINISHED);
}

void drillChar(char c) {
  // Get the (0 0) coords of the char field.
  // Get scaling factor of char fields
  // Christophs functions

  // Assuming that the drill is at (0 0) of the char field:
  float** charCoords = charTooCoords(c);
  uint coordLen = getCoordLength(c);

  float posCharX = 0.0f;
  float posCharY = 0.0f;

  // startDrill();
  for (int i = 0; i < coordLen; i++) {
    float xCoord = charCoords[i][0];
    float yCoord = charCoords[i][1];

    moveRelativeInCharField((xCoord - posCharX), (yCoord - posCharY));

    posCharX = xCoord;
    posCharY = yCoord;
  }

  stopMove();
  delay(200);
  stopDrill();
  delay(200);

  // Reset to bottom right corner
  moveRelativeInCharField((1.0 - posCharX), -posCharY);
}


// ======================
/* CONSOLE Input Logic */
/* Muss zu USB oder anderem umgeändert werden! */
// ==============================================

String getUserName() {
  while (true) {
    if (!SerialBT.available()) continue;
    
    String s = SerialBT.readString();
    if (s.endsWith(String(BT_USER_NAME_END_TOKEN))) {
      s.remove(s.length() - 1);
      return s;
    }
  }

  return "";
}

String getUserNameFromConsole() {
  while (Serial.available() == 0) { }
  String name = Serial.readString();
  name.toUpperCase();
  name.trim();                                 // Enter und co. löschen / remove any \r \n whitespace at the end of the String
  
  return name;
}


// =================
/* Movement Logic */
// =================

void stopMove() {
  X_STEPPER_MOTOR.stop();
  Y_STEPPER_MOTOR.stop();
}

void moveSteps(int xSteps, int ySteps, int maxSpeed) {
  float* speeds = configureMotors(xSteps, ySteps, maxSpeed);

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

float* configureMotors(int xSteps, int ySteps, int maxSpeed) {
  bool xHigh = (abs(xSteps) > abs(ySteps)) ? true : false;
  AccelStepper* throttledMotor = (xHigh) ? &Y_STEPPER_MOTOR: &X_STEPPER_MOTOR;
  AccelStepper* nonThrottledMotor = (xHigh) ? &X_STEPPER_MOTOR: &Y_STEPPER_MOTOR;
  float throttle = (xHigh) ? ((float) ySteps / (float) xSteps) : ((float) xSteps / (float) ySteps);
  throttle = abs(throttle);

  float throttledMotorSpeed = (maxSpeed * throttle);
  throttledMotor->setSpeed(throttledMotorSpeed);
  throttledMotor->setAcceleration(throttledMotorSpeed);

  nonThrottledMotor->setSpeed(maxSpeed);
  
  float xSpeed = (xHigh) ? maxSpeed : throttledMotorSpeed;
  float ySpeed = (xHigh) ? throttledMotorSpeed : maxSpeed;

  return new float[2] {xSpeed, ySpeed};
}

void moveRelativeOnBoard(float x, float y) {
  int xSteps = X_STEP_COUNT * x;
  int ySteps = Y_STEP_COUNT * y;

  positionX += x;
  positionY += y;

  moveSteps(xSteps, ySteps, MOVE_STEP_SPEED);
}

void moveToOnBoard(float x, float y) {
  moveRelativeOnBoard((x - positionX), (y - positionY));
}

void moveRelativeInCharField(float x, float y) {
  int xSteps = X_STEP_CHAR_COUNT * x;
  int ySteps = Y_STEP_CHAR_COUNT * y;

  positionX += ((float) xSteps / (float) X_STEP_COUNT);
  positionY += ((float) ySteps / (float) Y_STEP_COUNT);

  moveSteps(xSteps, ySteps, DRILL_STEP_SPEED);
}

// Moves the drill too 0 0 on the glass board
void moveToPlexiStart() {
  gotoHome();
  delay(200);

  moveToPlexiX();
  moveToPlexiY();

  positionX = 0.0f;
  positionY = 0.0f;
}

void moveToPlexiX() {
                    // (V-------V) Magic values
  float stepszuplexiX = 2.05 * 100;
  moveSteps(stepszuplexiX, 0, MOVE_STEP_SPEED);
}

void moveToPlexiY() {
                    // (V------V) Magic values
  float stepszuplexiY = 52.2 * 100; // Weg * Schritte pro mm
  moveSteps(0, stepszuplexiY, MOVE_STEP_SPEED);
}

void moveToAusgabe() {
  stopAll();
  delay(1000);
  // stopDrill();
  delay(1000);
  gotoHome();
  delay(1000);

              // (V-------V) Magic values
  float ausgabe = 140 * 100;
  moveSteps(0, ausgabe, MOVE_STEP_SPEED);
}


// =====================
/* Drillhead Movement */
// =====================

void stopDrill() {
  stopMove();

  
  long startMillis = millis();
  long runMillis = 0;

  while (runMillis < DRILL_STARTUP_TIME_MS) {
    analogWrite(DRILL_MOTOR_POWER_PIN, 255 - (255 * ((float) runMillis / (float) DRILL_STARTUP_TIME_MS)));
    runMillis = (millis() - startMillis);
  }

  digitalWrite(DRILL_ENABLE_PIN, LOW);
  analogWrite(DRILL_MOTOR_POWER_PIN, 0);

 // raiseDrill();
  
}

void startDrill() {
  
  digitalWrite(DRILL_ENABLE_PIN, HIGH);

  long startMillis = millis();
  long runMillis = 0;

 // while (runMillis < DRILL_STARTUP_TIME_MS) {
    //analogWrite(DRILL_MOTOR_POWER_PIN, (255 * ((float) runMillis / (float) DRILL_STARTUP_TIME_MS)));
    //runMillis = (millis() - startMillis);
 // }
 
 int n = 0;
while(n < 255){
  analogWrite(DRILL_MOTOR_POWER_PIN, n);
  delay(39);
  n++;
}




  // moveDrillHeight(-STEPS_TO_PLATE);
  // drillLowered = true;
  
}

void raiseDrill() {
  // There is no check here...
  // I just believe... that it won't break :)
  moveDrillHeight(STEPS_TO_PLATE);
  drillLowered = false;
}

// This method should only ever have too be run once
// Calibrate distance from the drill head too the glass
void calibrateDrillDistance() {
  moveDrillHeight(DRILL_STARTUP_RAISE_STEPS);

  // Hopefully the drill will be far enough up, as too not snap when moving around ._.
  gotoHome();

  // Move to the Z Sensor placed on the plate
  // Note; I'm moving the plate here using "#moveSteps", having a precise home-point is crucial!
  moveSteps(DRILL_SENSOR_X_STEPS, DRILL_SENSOR_Y_STEPS, MOVE_STEP_SPEED);
  delay(500);

  moveToSensor(&Z_STEPPER_MOTOR, Z_SENSOR, MOTOR_SLOW_STEP_SPEED);
  moveFromSensor(&Z_STEPPER_MOTOR, Z_SENSOR, MOTOR_SLOW_STEP_SPEED);
  moveToSensor(&Z_STEPPER_MOTOR, Z_SENSOR, MOTOR_SLOW_STEP_SPEED / MOTOR_SLOW_STEP_FACTOR);
  moveFromSensor(&Z_STEPPER_MOTOR, Z_SENSOR, MOTOR_SLOW_STEP_SPEED / MOTOR_SLOW_STEP_FACTOR);
}

void moveDrillHeight(uint steps) {
  Z_STEPPER_MOTOR.move(steps);
  Z_STEPPER_MOTOR.setPinsInverted((steps < 0), false, false);
  Z_STEPPER_MOTOR.setSpeed(DRILL_STEP_SPEED);

  while (Z_STEPPER_MOTOR.distanceToGo() > 0) {
    Z_STEPPER_MOTOR.runSpeed();
    Z_STEPPER_MOTOR.setSpeed(DRILL_STEP_SPEED);
  }

  Z_STEPPER_MOTOR.setPinsInverted(false, false, false);
}



// =============
/* Home Logic */
// =============

void gotoHome() {
  stopDrill();
  delay(1000);    // Just in case

  calibrateXAxis();
  calibrateYAxis();
}

void calibrateXAxis() {
  // Move to sensor; Move away; Move towards... slower; Move away... slower
  moveToSensor(&X_STEPPER_MOTOR, X_SENSOR, MOTOR_SLOW_STEP_SPEED);
  moveSteps(200, 0, MOTOR_SLOW_STEP_SPEED);
  moveToSensor(&X_STEPPER_MOTOR, X_SENSOR, MOTOR_SLOW_STEP_SPEED / MOTOR_SLOW_STEP_FACTOR);
  moveFromSensor(&X_STEPPER_MOTOR, X_SENSOR, MOTOR_SLOW_STEP_SPEED / MOTOR_SLOW_STEP_FACTOR);
}

void calibrateYAxis() {
  // Definitely didn't copy paste it... I would never!
  moveToSensor(&Y_STEPPER_MOTOR, Y_SENSOR, MOTOR_SLOW_STEP_SPEED);
  moveSteps(0, 200, MOTOR_SLOW_STEP_SPEED);
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