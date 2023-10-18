#define X_MOTOR_DIR_PIN 19
#define X_MOTOR_STEP_PIN 21
void setup() {
// put your setup code here, to run once:
Serial.begin(115200);
pinMode(X_MOTOR_STEP_PIN, OUTPUT);
pinMode(X_MOTOR_DIR_PIN, OUTPUT);
}
void loop() {
// put your main code here, to run repeatedly: 

delayMicroseconds(200);
int il = letters();
int abz = abstand(il);
float breite =breiteBox(il, abz);
float einheit =moveToM();
firstbox(il,abz,breite,einheit);
//test();
}
int letters(){
while(Serial.available() == 0){}//Auf User Input warten
String input = Serial.readString(); //User Input
input.trim(); // Enter und co. löschen / remove any \r \n whitespace at the end of the String
Serial.println(input);
int il = input.length();
Serial.println("Boxen");
Serial.println(il);
return il;
}
int abstand(int il)
{
int abz = il + 1; 
Serial.println("Abstände");
Serial.println(abz);
return abz;
}
float breiteBox(int il, int abz)
{
float breite = ((float)(60.0 - abz)/ (float) il );
breite = min(breite, (float)7.0);
Serial.println("BReite");
Serial.println(breite);
return breite;
}

int s = 0;
float stepszubox1 = 0;
float moveToM() {
float einheit = 1.0/60.0; //1mm in Koordinate umgerechnet
float Mitte = einheit * 30; // Abstand nach links u rechts an die Wand
float stepszubox1 = Mitte * 6000; // 0,5 Punkt erster Box
return einheit;
Serial.println(stepszubox1);
while(s < stepszubox1)
  {
  digitalWrite(X_MOTOR_STEP_PIN, HIGH);
  delayMicroseconds(300);
  digitalWrite(X_MOTOR_STEP_PIN, LOW);
  delayMicroseconds(300);
  s++;
  }
}

void firstbox(int il, int abz, float breite, float einheit)
{

if( il % 2 == 0) //Gerade Anzahl
  {
    float b1 = ((float)breite * (il/2)+(((float)abz/2)-1));
     //Serial.println("Breite");
     //Serial.println(b1);
    // Serial.println("Die Zahl ist gerade");
    float c1 = ((float)30-b1)* einheit;
    Serial.println("Cord");
    Serial.println(c1);
  }
else //Ungerde Zahl
  {
    float b2 = ((float)breite * ((float)il/2)+(((float)abz/2)-1));
    Serial.println("Zu b2");
    Serial.println(b2);
    Serial.println("Die Zahl ist ungerade");
    float c2 = ((float)30-1)* einheit;
    Serial.println("Cord");
    Serial.println(c2);
  }
}

