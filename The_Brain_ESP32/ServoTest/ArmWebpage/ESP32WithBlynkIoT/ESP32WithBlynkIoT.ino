#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>
#include <ESP32Servo.h> 

const char* ssid = "186F";      // <--- EDIT THIS
const char* password = "30004000"; // <--- EDIT THIS

WebServer server(80); 

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

const int HOME_W = 50;
const int HOME_S = 90;
const int HOME_E = 130;
const int HOME_G = 0; 

const int PREP_W = 150;
const int PREP_S = 90;
const int PREP_E = 180;
const int PREP_G = 180; 

const int GRAB_W = 150; 
const int GRAB_S = 150; 
const int GRAB_E = 160; 
const int GRAB_G = 0;   

const int GRIPPER_OPEN = 180;
const int GRIPPER_CLOSED = 0;

int redVal = 0, greenVal = 0, blueVal = 0;
char detectedColorChar = 'N'; 
unsigned long colorDetectionStartTime = 0;

unsigned long lastManualCommandTime = 0;
const int MANUAL_TIMEOUT = 1000; 

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
RoboEyes<Adafruit_SSD1306> roboEyes(display);

Servo servoWAIST, servoSHOULDER, servoELBOW, servoGRIPPER;

int curW = HOME_W, curS = HOME_S, curE = HOME_E, curG = HOME_G;

// --- PRO HTML PAGE DESIGN ---
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0">
  <title>Smart Arm Controller</title>
  <style>
    :root { --bg: #121212; --card: #1e1e1e; --primary: #00ff9d; --accent: #00bcd4; --danger: #ff4d4d; --text: #e0e0e0; }
    body { font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif; background-color: var(--bg); color: var(--text); display: flex; flex-direction: column; align-items: center; justify-content: center; min-height: 100vh; margin: 0; padding: 20px; user-select: none; }
    h1 { color: var(--primary); margin-bottom: 5px; text-transform: uppercase; letter-spacing: 2px; }
    p { color: #888; font-size: 14px; margin-bottom: 30px; }
    
    .container { background-color: var(--card); padding: 30px; border-radius: 20px; box-shadow: 0 10px 30px rgba(0,0,0,0.5); width: 100%; max-width: 400px; text-align: center; border: 1px solid #333; }
    
    .btn-group { display: flex; justify-content: center; gap: 15px; margin-bottom: 15px; }
    
    .btn { 
      background: #333; color: white; border: none; padding: 15px; 
      border-radius: 12px; font-size: 24px; cursor: pointer; width: 80px; height: 80px;
      display: flex; align-items: center; justify-content: center;
      transition: all 0.1s ease; box-shadow: 0 4px 0 #222;
    }
    .btn:active { transform: translateY(4px); box-shadow: 0 0 0 #222; }
    
    .btn-blue { background: var(--accent); color: #000; box-shadow: 0 4px 0 #008ba3; }
    .btn-blue:active { box-shadow: 0 0 0 #008ba3; background: #00acc1; }
    
    .btn-red { background: var(--danger); color: white; box-shadow: 0 4px 0 #c62828; width: 100%; font-size: 18px; font-weight: bold; }
    .btn-red:active { box-shadow: 0 0 0 #c62828; }
    
    .btn-green { background: var(--primary); color: black; box-shadow: 0 4px 0 #00c853; width: 100%; font-size: 18px; font-weight: bold; }
    .btn-green:active { box-shadow: 0 0 0 #00c853; }

    .btn-yellow { background: #ffc107; color: black; width: 100%; font-weight: bold; margin-top: 20px; font-size: 20px; box-shadow: 0 4px 0 #ff8f00; }
    .btn-yellow:active { box-shadow: 0 0 0 #ff8f00; }

    .control-pad { display: grid; grid-template-columns: 1fr 1fr 1fr; gap: 10px; margin-bottom: 30px; justify-items: center; }
    .empty { width: 80px; height: 80px; }
    
    .status { margin-top: 20px; font-size: 12px; color: #555; }
  </style>
</head>
<body>

  <div class="container">
    <h1>Smart Arm</h1>
    <p>Hybrid Control Panel</p>

    <!-- Directional Pad -->
    <div class="control-pad">
      <div class="empty"></div>
      <button class="btn btn-blue" onmousedown="send('u')">▲</button>
      <div class="empty"></div>
      
      <button class="btn btn-blue" onmousedown="send('l')">◀</button>
      <button class="btn" style="background:#222; cursor:default; box-shadow:none;">🦾</button>
      <button class="btn btn-blue" onmousedown="send('r')">▶</button>
      
      <div class="empty"></div>
      <button class="btn btn-blue" onmousedown="send('d')">▼</button>
      <div class="empty"></div>
    </div>

    <!-- Actions -->
    <div class="btn-group">
      <button class="btn btn-green" onmousedown="send('O')">OPEN ✋</button>
      <button class="btn btn-red" onmousedown="send('C')">CLOSE ✊</button>
    </div>

    <button class="btn btn-yellow" onmousedown="send('H')">WAVE HELLO 👋</button>

    <div class="status" id="status">Status: Ready</div>
  </div>

  <script>
    function send(cmd) {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/cmd?val=" + cmd, true);
      xhr.send();
      
      // Update UI Status
      let text = "";
      if(cmd == 'u') text = "Moving Up";
      if(cmd == 'd') text = "Moving Down";
      if(cmd == 'l') text = "Moving Left";
      if(cmd == 'r') text = "Moving Right";
      if(cmd == 'O') text = "Opening Gripper";
      if(cmd == 'C') text = "Closing Gripper";
      if(cmd == 'H') text = "Saying Hi!";
      document.getElementById('status').innerText = "Status: " + text;
    }
  </script>
</body>
</html>
)rawliteral";

void setup() {
  Serial.begin(115200);
  
  Serial.print("Connecting to WiFi");
  WiFi.begin(ssid, password);
  int retry = 0;
  while (WiFi.status() != WL_CONNECTED && retry < 20) {
    delay(500);
    Serial.print(".");
    retry++;
  }
  
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi Connected!");
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP()); 
  } else {
    Serial.println("\nWiFi Failed! Running in Offline Mode.");
  }

  server.on("/", handleRoot);
  server.on("/cmd", handleCommand);
  server.begin();

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

  Serial.println("System Ready.");
  goHome(); 
}

void loop() {
  server.handleClient(); 
  roboEyes.update();     

  if (checkSerialInput()) {
  } 
  else if (millis() - lastManualCommandTime > MANUAL_TIMEOUT) {
      runRealTimeAutoMode(); 
  }
}

void handleRoot() {
  server.send(200, "text/html", htmlPage);
}

void handleCommand() {
  if (server.hasArg("val")) {
    char cmd = server.arg("val").charAt(0);
    lastManualCommandTime = millis(); 
    executeManualCommand(cmd);
    server.send(200, "text/plain", "OK");
  }
}

bool checkSerialInput() {
  if (Serial.available()) {
    char cmd = Serial.read();
    if (cmd != 0 && cmd != '\n' && cmd != '\r') {
      lastManualCommandTime = millis();
      executeManualCommand(cmd);
      return true;
    }
  }
  return false;
}

void executeManualCommand(char cmd) {
    int step = 5; 
    
    switch (cmd) {
        case 'l': 
            curW = constrain(curW + step, 0, 180); 
            servoWAIST.write(curW);
            break;
        case 'r': 
            curW = constrain(curW - step, 0, 180); 
            servoWAIST.write(curW);
            break;
        case 'u': 
            curS = constrain(curS + step, 90, 160); 
            curE = constrain(curE + step/2, 90, 180); 
            servoSHOULDER.write(curS);
            servoELBOW.write(curE);
            break;
        case 'd': 
            curS = constrain(curS - step, 90, 160);
            curE = constrain(curE - step/2, 90, 180);
            servoSHOULDER.write(curS);
            servoELBOW.write(curE);
            break;
        case 'O': 
            curG = GRIPPER_OPEN;
            servoGRIPPER.write(curG);
            roboEyes.setMood(HAPPY);
            break;
        case 'C': 
            curG = GRIPPER_CLOSED;
            servoGRIPPER.write(curG);
            roboEyes.setMood(ANGRY);
            break;
        case 'H':
            waveHello();
            break;
    }
}

void waveHello() {
    roboEyes.setMood(HAPPY);
    int originalW = curW;
    for(int i=0; i<2; i++) {
        servoWAIST.write(constrain(originalW + 20, 0, 180)); delay(150); roboEyes.update();
        servoWAIST.write(constrain(originalW - 20, 0, 180)); delay(150); roboEyes.update();
    }
    curW = originalW; servoWAIST.write(curW);
}

void runRealTimeAutoMode() {
  readColorFast(); 
  char currentColor = 'N';

  if (redVal > 350 && greenVal > 350 && blueVal > 350) currentColor = 'N'; 
  else if (isRed()) currentColor = 'R';
  else if (isGreen()) currentColor = 'G';
  else if (isBlue()) currentColor = 'B';

  if (currentColor == 'R') { setLedColor(HIGH, LOW, LOW); roboEyes.setMood(HAPPY); }
  else if (currentColor == 'G') { setLedColor(LOW, HIGH, LOW); roboEyes.setMood(HAPPY); }
  else if (currentColor == 'B') { setLedColor(LOW, LOW, HIGH); roboEyes.setMood(TIRED); }
  else { setLedColor(LOW, LOW, LOW); roboEyes.setMood(DEFAULT); }

  if (currentColor != 'N' && currentColor == detectedColorChar) {
      if (millis() - colorDetectionStartTime > 300) { 
          performPickAndPlace(currentColor);
          detectedColorChar = 'N'; 
          colorDetectionStartTime = 0;
          lastManualCommandTime = millis(); 
      }
  } else {
      detectedColorChar = currentColor;
      colorDetectionStartTime = millis();
  }
}

void performPickAndPlace(char color) {
  moveArm(PREP_W, PREP_S, PREP_E, PREP_G); delay(400); 
  moveArm(GRAB_W, GRAB_S, GRAB_E, PREP_G); delay(300);
  servoGRIPPER.write(GRAB_G); curG = GRAB_G; delay(500); 
  moveArm(curW, 90, 180, curG); delay(300);

  int dropW, dropS, dropE;
  if (color == 'R') { dropW = 90; dropS = 160; dropE = 120; } 
  else if (color == 'G') { dropW = 50; dropS = 150; dropE = 90; } 
  else { dropW = 10; dropS = 160; dropE = 130; } 

  moveArm(dropW, dropS, dropE, curG); delay(500);
  servoGRIPPER.write(GRIPPER_OPEN); curG = GRIPPER_OPEN; delay(300);
  moveArm(dropW, 90, 130, curG); delay(200);
  goHome();
  setLedColor(LOW, LOW, LOW); 
}

void goHome() { moveArm(HOME_W, HOME_S, HOME_E, HOME_G); }

void moveArm(int w, int s, int e, int g) {
  moveSmooth(servoWAIST, curW, w);
  moveSmooth(servoSHOULDER, curS, s);
  moveSmooth(servoELBOW, curE, e);
  servoGRIPPER.write(g); curG = g; delay(10); 
}

void moveSmooth(Servo &servo, int &currentPos, int targetPos) {
  int step = (targetPos > currentPos) ? 2 : -2; 
  if (abs(targetPos - currentPos) <= 2) {
      currentPos = targetPos; servo.write(currentPos); return;
  }
  for (int pos = currentPos; pos != targetPos; pos += step) {
    servo.write(pos); delay(6); 
    if (abs(targetPos - pos) <= 2) break; 
  }
  currentPos = targetPos; servo.write(currentPos);
}

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