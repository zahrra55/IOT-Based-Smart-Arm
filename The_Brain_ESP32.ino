#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h> 

// =====================================================
// 1. PIN MAPPING
// =====================================================
#define PIN_WAIST    13
#define PIN_SHOULDER 12 
#define PIN_ELBOW    14 
#define PIN_GRIPPER  27

#define PIN_LED_RED   4
#define PIN_LED_GREEN 5
#define PIN_LED_BLUE  18

#define S0 32
#define S1 33
#define S2 25
#define S3 26
#define S_OUT 35
// =====================================================
// 2. CALIBRATED ANGLES
// =====================================================
// (Home)
const int HOME_W = 50;
const int HOME_S = 90;
const int HOME_E = 130;
const int HOME_G = 0; // closed

// Getting ready to grab
const int PREP_W = 150;
const int PREP_S = 90;
const int PREP_E = 180;
const int PREP_G = 180; //open

// (Grab)
const int GRAB_W = 150; 
const int GRAB_S = 150; 
const int GRAB_E = 160; 
const int GRAB_G = 0;   

// color vars
int redVal = 0, greenVal = 0, blueVal = 0;
char detectedColorChar = 'N'; 
unsigned long colorDetectionStartTime = 0;

// =====================================================
// 3. OBJECTS
// =====================================================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
RoboEyes<Adafruit_SSD1306> roboEyes(display);

BluetoothSerial SerialBT;
Servo servoWAIST, servoSHOULDER, servoELBOW, servoGRIPPER;

int curW = HOME_W, curS = HOME_S, curE = HOME_E, curG = HOME_G;

// =====================================================
// 4. SETUP
// =====================================================
void setup() {
  Serial.begin(115200);
  SerialBT.begin("Smart_Arm_Master");
  
  servoWAIST.setPeriodHertz(50);
  servoSHOULDER.setPeriodHertz(50);
  servoELBOW.setPeriodHertz(50);
  servoGRIPPER.setPeriodHertz(50);

  servoWAIST.attach(PIN_WAIST, 500, 2400);
  servoSHOULDER.attach(PIN_SHOULDER, 500, 2400);
  servoELBOW.attach(PIN_ELBOW, 500, 2400);
  servoGRIPPER.attach(PIN_GRIPPER, 500, 2400);

  pinMode(PIN_LED_RED, OUTPUT); pinMode(PIN_LED_GREEN, OUTPUT); pinMode(PIN_LED_BLUE, OUTPUT);
  setLedColor(LOW, LOW, LOW);

  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(S_OUT, INPUT);
  digitalWrite(S0, HIGH); digitalWrite(S1, LOW); 

  Wire.begin(21, 22);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) for(;;);
  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 100);
  roboEyes.setAutoblinker(ON, 3, 2);
  roboEyes.setIdleMode(ON, 2, 2);
  roboEyes.setMood(DEFAULT);

  Serial.println("Moving to HOME position...");
  goHome(); 
}

// =====================================================
// 5. LOOP
// =====================================================
void loop() {
  roboEyes.update(); 
  runRealTimeAutoMode(); 
}

// =====================================================
// 6. LOGIC FUNCTIONS
// =====================================================

void runRealTimeAutoMode() {
  readColorFast(); 

  char currentColor = 'N';

  if (redVal > 350 && greenVal > 350 && blueVal > 350) {
      currentColor = 'N'; 
  }
  else if (isRed()) currentColor = 'R';
  else if (isGreen()) currentColor = 'G';
  else if (isBlue()) currentColor = 'B';

  if (currentColor == 'R') { setLedColor(HIGH, LOW, LOW); roboEyes.setMood(HAPPY); }
  else if (currentColor == 'G') { setLedColor(LOW, HIGH, LOW); roboEyes.setMood(HAPPY); }
  else if (currentColor == 'B') { setLedColor(LOW, LOW, HIGH); roboEyes.setMood(TIRED); }
  else { setLedColor(LOW, LOW, LOW); roboEyes.setMood(DEFAULT); }

  if (currentColor != 'N' && currentColor == detectedColorChar) {
      if (millis() - colorDetectionStartTime > 300) { 
          Serial.print("Confirmed Color: "); Serial.println(currentColor);
          performPickAndPlace(currentColor);
          detectedColorChar = 'N'; 
          colorDetectionStartTime = 0;
      }
  } else {
      detectedColorChar = currentColor;
      colorDetectionStartTime = millis();
  }
}

// =====================================================
// 7. MOVEMENT SEQUENCES
// =====================================================

void performPickAndPlace(char color) {
  
  // 1. Preparing to Pick (Waist = 150)
  Serial.println("1. Prep...");
  moveArm(PREP_W, PREP_S, PREP_E, PREP_G); 
  delay(400); 

  // 2. Grabbing
  Serial.println("2. Grab...");
  moveArm(GRAB_W, GRAB_S, GRAB_E, PREP_G); 
  delay(300);
  servoGRIPPER.write(GRAB_G); 
  curG = GRAB_G;
  delay(500); 

  // 3. Lifting
  Serial.println("3. Lift...");
  moveArm(curW, 90, 180, curG);
  delay(300);

  // 4. Moving to Drop Zone
  Serial.println("4. Move Drop...");
  int dropW, dropS, dropE;

  if (color == 'R') {      
      dropW = 90; dropS = 160; dropE = 120;
  } 
  else if (color == 'G') { 
      dropW = 50; dropS = 150; dropE = 90;
  } 
  else {                   
      dropW = 10; dropS = 160; dropE = 130;
  }

  moveArm(dropW, dropS, dropE, curG);
  delay(500);

  // 5. Dropping
  Serial.println("5. Drop...");
  servoGRIPPER.write(180); 
  curG = 180;
  delay(300);

  // 6. Return Home
  Serial.println("6. Home...");
  moveArm(dropW, 90, 130, curG);
  delay(200);
  
  goHome();
  setLedColor(LOW, LOW, LOW); 
}

// -----------------------------------------------------
// Helpers
// -----------------------------------------------------

void goHome() {
  moveArm(HOME_W, HOME_S, HOME_E, HOME_G);
}

void moveArm(int w, int s, int e, int g) {
  moveSmooth(servoWAIST, curW, w);
  moveSmooth(servoSHOULDER, curS, s);
  moveSmooth(servoELBOW, curE, e);
  servoGRIPPER.write(g);
  curG = g;
  delay(10); 
}

void moveSmooth(Servo &servo, int &currentPos, int targetPos) {
  int step = (targetPos > currentPos) ? 2 : -2; 
  if (abs(targetPos - currentPos) <= 2) {
      currentPos = targetPos;
      servo.write(currentPos);
      return;
  }
  for (int pos = currentPos; pos != targetPos; pos += step) {
    servo.write(pos);
    delay(6); 
    if (abs(targetPos - pos) <= 2) break; 
  }
  currentPos = targetPos;
  servo.write(currentPos);
}

// -----------------------------------------------------
// Sensors
// -----------------------------------------------------
void readColorFast() {
  digitalWrite(S2, LOW); digitalWrite(S3, LOW);
  redVal = pulseIn(S_OUT, LOW, 8000UL); 
  digitalWrite(S2, HIGH); digitalWrite(S3, HIGH);
  greenVal = pulseIn(S_OUT, LOW, 8000UL);
  digitalWrite(S2, LOW); digitalWrite(S3, HIGH);
  blueVal = pulseIn(S_OUT, LOW, 8000UL);
}

bool isRed() { return (redVal < 320 && redVal < greenVal && redVal < blueVal); }
bool isGreen() { return (greenVal < 320 && greenVal < redVal && greenVal < blueVal); }
bool isBlue() { return (blueVal < 320 && blueVal < redVal && blueVal < greenVal); }

void setLedColor(bool red, bool green, bool blue) {
  digitalWrite(PIN_LED_RED, red); digitalWrite(PIN_LED_GREEN, green); digitalWrite(PIN_LED_BLUE, blue);

}
