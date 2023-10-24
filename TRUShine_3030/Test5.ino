//////////////////////
//Motoren definiert//
////////////////////
#define X_MOTOR_DIR_PIN 19
#define X_MOTOR_STEP_PIN 21
#define Y_MOTOR_DIR_PIN 15
#define Y_MOTOR_STEP_PIN 17
#define Z_MOTOR_DIR_PIN 5
#define Z_MOTOR_STEP_PIN 18
//////////////
//Variablen//
////////////
float einheit = 1.0/60.0;     //1mm in Koordinate umgerechnet
int i;                        // nur zum einmaligen teste. Wird bei fertigem Programm nicht mehr benötigt
///////////////////////////
//Variablen für MoveToMP//
/////////////////////////
int s = 0;                    //Schritte
float stepszubox1 = 0;        //Anazhl der Schritte
void setup() 
{
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(X_MOTOR_STEP_PIN, OUTPUT);
  pinMode(X_MOTOR_DIR_PIN, OUTPUT);
  pinMode(Y_MOTOR_STEP_PIN, OUTPUT);
  pinMode(Y_MOTOR_DIR_PIN, OUTPUT);
  pinMode(Z_MOTOR_STEP_PIN, OUTPUT);
  pinMode(Z_MOTOR_DIR_PIN, OUTPUT);
}
//////////////////
//Main Programm//
////////////////
void loop() 
{
  // put your main code here, to run repeatedly: 
  delayMicroseconds(200);
  int il = letters();
  int abz = abstand(il);
  float breite = breiteBox(il, abz);
  if(il <= 12){
  firstboxGerade(il,abz,breite,einheit);
  }
  // float NeueBox =nextBoxRechnerG(breite, einheit, abz, c1, il);
  // nextBoxesG(NeueBox,c1,breite,einheit,abz,il );
  //moveToPlexiX();
  // if(i == 0){
  // moveToPlexiY();
  // i++;
  // }
  // if(i==0){
  //   Ausgabe();
  //   i++;
  // }
}
//////////////////////////////////
//Anzahl an Buchstaben erkennen//
////////////////////////////////
int letters()
{
  while(Serial.available() == 0){}              //Auf User Input warten
  String input = Serial.readString();           //User Input
  input.trim();                                 // Enter und co. löschen / remove any \r \n whitespace at the end of the String
  Serial.println(input);                        //Input printen
  int il = input.length();                      //Variable für die Anzahl an Buchstaben
  if(il >= 12)
  {
    Serial.println("Dein Name ist zu lang. Versuch es mit einem Spitznamen");
  }
  // Serial.println("Boxen");                   //War zum tesen ob rechnungen stimmen
  // Serial.println(il);
  return il;                                    //il zurückgeben, damit andere Funktionen auf die Variable zugreifen können
}
////////////////////////
//Abstände ausrechnen//
//////////////////////
int abstand(int il)
{
  int abz = il + 1;                       //Abstand ausrechnen
  // Serial.println("Abstände");          //War zum tesen ob rechnungen stimmen
  // Serial.println(abz);
  return abz;                             //Varbaible zurückgeben um darauf zugreifen zu können
}
////////////////////////////
//Boxen skalierbar machen//
//////////////////////////
float breiteBox(int il, int abz)
{
  float breite = ((float)(60.0 - abz)/ (float) il );
  breite = min(breite, (float)7.0);
  //Serial.println("BReite");//War zum tesen ob rechnungen stimmen
  //Serial.println(breite);
  return breite;
}
/////////////////////////////////////////
//Zum Mittelpunkt der Plexiglasscheibe//
///////////////////////////////////////
float moveToMP() 
{
  float Mitte = einheit * 30; // Abstand nach links u rechts an die Wand
  float stepszubox1 = Mitte * 6000; // 0,5 Punkt erster Box
  return einheit;
  //Serial.println(stepszubox1);
  while(s < stepszubox1)
    {
    digitalWrite(X_MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(300);
    digitalWrite(X_MOTOR_STEP_PIN, LOW);
    delayMicroseconds(300);
    s++;
    }
}
//////////////////////////////
//Berechnung der ersten Box//
////////////////////////////

float* firstboxGerade(int il, int abz, float breite, float einheit)
{
  float b1 = ((float)breite * ((float)il/2)+(((float)abz/2)-1));//Breite die es zu ersten Box gehe muss
  //Serial.println("Breite");//War zum tesen ob rechnungen stimmen
  //Serial.println(b1);
  float c1 = ((float)30-b1)* einheit;//Cord der ersten 0.0 Box
  //Neue Box
  float NeueBox = ((float) breite + 1)* einheit;// Cord neue Box (Breite Box+ Abstand) vorrausetzung immer 0.0  
  float Werte[il] = {};//verusch des arrays
  float box2 = c1;

  for (int b = 0; b < il; b++) {
    Werte[b] = (float)box2;
    box2 = ((float)c1 + NeueBox);//rechnet alle andern Boxen aus (geht, wegen dem array aber muss man programm immer neustarten)
    c1 = box2;
    Serial.println(Werte[b]);
  }

  return Werte;
}
// ///////////////////////////////////////////
// //Berechnen der Wegstrecke zur neuen Box//
// /////////////////////////////////////////
// float nextBoxRechnerG(float breite,float einheit, int abz, float c1,int il)
//   {
//     float NeueBox = ((float) breite + 1)* einheit;
//     //Serial.println(NeueBox);// Cord neue Box (Breite Box+ Abstand) vorrausetzung immer 0.0  
//     //float box2 = ((float)c1 + NeueBox);
//     //Serial.println("Cord Box 2");
//     //Serial.println(box2);
//     return NeueBox;
  
// }
// ////////////////////////////////////
// //Berechnung aller weiteren Boxen//
// //////////////////////////////////

// void nextBoxesG(float NeueBox, float c1, float breite,float einheit, float abz, int il ){
//    float Werte[il] = {};
//    while(b < (il-1))
//    {
//     float box2 = ((float)c1 + NeueBox);
//     // Serial.println("Cord Box 3");
//     // Serial.println(box2);
//     c1 = box2;
//     b++;
//     for(int c=0; c < il;c++)
//     {
//       Werte[c] = box2;
//       Serial.println(Werte[c]); 
//     }
    
//     }
    
   
   

// }

////////////////////////////////////////////////////////////////////////////////////////
//X Motor zentrieren, so dass die Platte an der linken Kante unter dem Fräskopf liegt//
//////////////////////////////////////////////////////////////////////////////////////
void moveToPlexiX()
{
  int x;
  float stepszuplexiX = 0.6 * 100;
  while(x < stepszuplexiX)
  {  
  digitalWrite(X_MOTOR_STEP_PIN, HIGH);
  delayMicroseconds(300);
  digitalWrite(X_MOTOR_STEP_PIN, LOW);
  delayMicroseconds(300);
  x++;
  }
  delay(5000);
  float einheit = moveToMP();
}
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Y Motor zentrieren, so dass die Platte an der unteren Kante unter dem Fräskpf liet ( sollte nun unten links in der Ecke sein)//
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void moveToPlexiY()
{
  int y;
  float stepszuplexiY = 40 * 100; // Weg * Schritte pro mm
  while(y <= stepszuplexiY)
  {
  //Serial.println(y);
  digitalWrite(Y_MOTOR_STEP_PIN, HIGH);
  delayMicroseconds(300);
  digitalWrite(Y_MOTOR_STEP_PIN, LOW);
  delayMicroseconds(300);
  y++;
  }
}
///////////////////////////////////////////////////////////////////////
//Wenn die Scheibe fertig ist soll die Platte nach ganz vorne fahren//
/////////////////////////////////////////////////////////////////////
void Ausgabe()
{
  float ausgabe = 158 * 100;
  int a;
  while(a <= ausgabe)
  {
    Serial.println(a);
    digitalWrite(Y_MOTOR_STEP_PIN, HIGH);
    delayMicroseconds(300);
    digitalWrite(Y_MOTOR_STEP_PIN, LOW);
    delayMicroseconds(300);
    a++;
  }
}

