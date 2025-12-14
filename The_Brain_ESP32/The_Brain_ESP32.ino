#include <WiFi.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>
#include <ESP32Servo.h>

// ===================== WiFi ==========================
const char* ssid     = "186F";
const char* password = "30004000";

WebServer server(80);

// ===================== PINs ==========================
#define PIN_WAIST    13
#define PIN_SHOULDER 12
#define PIN_ELBOW    14
#define PIN_GRIPPER  27

#define PIN_LED_RED   4
#define PIN_LED_GREEN 5
#define PIN_LED_BLUE  18

#define S0    32
#define S1    33
#define S2    25
#define S3    26
#define S_OUT 35

// ===================== CONST =========================
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

const int GRIPPER_OPEN   = 180;
const int GRIPPER_CLOSED = 0;

int redVal = 0, greenVal = 0, blueVal = 0;
char detectedColorChar = 'N';
unsigned long colorDetectionStartTime = 0;

unsigned long lastManualCommandTime = 0;
const int MANUAL_TIMEOUT  = 1000;
int COLOR_THRESHOLD = 900;

// ===================== SERVO SMOOTHING ===============
int curW = HOME_W, curS = HOME_S, curE = HOME_E, curG = HOME_G;
int targetW = HOME_W, targetS = HOME_S, targetE = HOME_E, targetG = HOME_G;

const int STEP_SLOW = 2;
const int STEP_FAST = 5;
const int UPDATE_INTERVAL = 15;
unsigned long lastServoUpdate = 0;

// ===================== OLED & Eyes ===================
#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
RoboEyes<Adafruit_SSD1306> roboEyes(display);

// ===================== Servos ========================
Servo servoWAIST, servoSHOULDER, servoELBOW, servoGRIPPER;

// ===================== HTML (Dashboard UI) ===========
const char* htmlPage = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>Smart Arm Dashboard</title>
<style>
  :root {
    --bg-deep: #020617;
    --bg-soft: #030712;
    --panel: #020617;
    --panel-soft: #050816;
    --border: rgba(148,163,184,0.45);
    --border-strong: rgba(94,234,212,0.8);
    --text: #e5e7eb;
    --muted: #9ca3af;
    --accent-cyan: #22d3ee;
    --accent-blue: #3b82f6;
    --accent-purple: #a855f7;
    --accent-pink: #ec4899;
    --accent-amber: #fbbf24;
    --accent-lime: #a3e635;
    --danger: #f97373;
    --success: #22c55e;
  }

  * { box-sizing: border-box; }

  body {
    margin: 0;
    min-height: 100vh;
    display: flex;
    align-items: center;
    justify-content: center;
    padding: 12px;
    font-family: system-ui, -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif;
    color: var(--text);
    background:
      radial-gradient(circle at 0% 0%, #22d3ee33 0, transparent 55%),
      radial-gradient(circle at 100% 0%, #a855f733 0, transparent 55%),
      radial-gradient(circle at 0% 100%, #f9731633 0, transparent 55%),
      linear-gradient(135deg, #020617, #020617);
  }

  .shell {
    width: 100%;
    max-width: 720px;
    padding: 2px;
    border-radius: 26px;
    background:
      conic-gradient(from 200deg,
        #22d3ee,
        #a855f7,
        #f97316,
        #22c55e,
        #22d3ee);
    box-shadow:
      0 0 40px rgba(15,23,42,0.9),
      0 0 90px rgba(15,23,42,1);
  }

  .dashboard {
    position: relative;
    border-radius: 24px;
    padding: 14px 14px 10px;
    background: radial-gradient(circle at 20% 0%, #0b1120 0, #020617 55%);
    overflow: hidden;
  }

  .dashboard::before {
    content: "";
    position: absolute;
    inset: -40%;
    background:
      radial-gradient(circle at 0% 0%, #22d3ee22 0, transparent 60%),
      radial-gradient(circle at 100% 100%, #f9731622 0, transparent 60%);
    mix-blend-mode: screen;
    opacity: 0.9;
    pointer-events: none;
  }

  .content {
    position: relative;
    z-index: 1;
  }

  .header {
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    margin-bottom: 10px;
  }

  .title-block {
    display: flex;
    flex-direction: column;
    gap: 2px;
  }

  .title {
    font-size: 16px;
    font-weight: 700;
    letter-spacing: 0.18em;
    text-transform: uppercase;
  }

  .subtitle {
    font-size: 11px;
    color: var(--muted);
  }

  .status-dot {
    width: 8px;
    height: 8px;
    border-radius: 999px;
    background: #22c55e;
    box-shadow: 0 0 8px #22c55e;
    display: inline-block;
    margin-right: 4px;
  }

  .chip {
    font-size: 10px;
    padding: 2px 8px;
    border-radius: 999px;
    border: 1px solid rgba(148,163,184,0.6);
    background: linear-gradient(135deg, #020617, #020617);
    color: var(--muted);
  }

  .layout {
    display: grid;
    grid-template-columns: minmax(0, 1.4fr) minmax(0, 1.1fr);
    gap: 10px;
  }

  @media (max-width: 720px) {
    .layout { grid-template-columns: minmax(0, 1fr); }
  }

  .panel {
    border-radius: 18px;
    padding: 10px;
    background: radial-gradient(circle at 0% 0%, #020617 0, #020617 60%);
    border: 1px solid rgba(148,163,184,0.45);
    box-shadow:
      0 14px 26px rgba(15,23,42,0.9),
      inset 0 0 0 1px rgba(15,23,42,0.9);
  }

  .panel-header {
    display: flex;
    justify-content: space-between;
    align-items: baseline;
    margin-bottom: 6px;
  }

  .panel-title {
    font-size: 11px;
    text-transform: uppercase;
    letter-spacing: 0.12em;
    color: var(--muted);
  }

  .pill {
    font-size: 9px;
    padding: 1px 7px;
    border-radius: 999px;
    border: 1px solid rgba(148,163,184,0.5);
    color: var(--muted);
  }

  .slider-group {
    display: flex;
    flex-direction: column;
    gap: 8px;
    margin-top: 4px;
  }

  .slider-row {
    display: grid;
    grid-template-columns: 70px minmax(0, 1fr) 42px;
    align-items: center;
    gap: 8px;
  }

  .slider-label {
    font-size: 11px;
    text-transform: uppercase;
    letter-spacing: 0.08em;
    color: var(--muted);
  }

  .slider-value {
    font-size: 11px;
    text-align: right;
    color: var(--accent-cyan);
    font-variant-numeric: tabular-nums;
  }

  .slider-wrap {
    position: relative;
    height: 20px;
    display: flex;
    align-items: center;
  }

  input[type=range] {
    -webkit-appearance: none;
    appearance: none;
    width: 100%;
    height: 6px;
    background: radial-gradient(circle at 0% 50%, #0f172a 0, #020617 65%);
    border-radius: 999px;
    outline: none;
    border: 1px solid rgba(15,23,42,1);
    box-shadow:
      inset 0 1px 3px rgba(15,23,42,0.9),
      0 0 0 1px rgba(15,23,42,0.8);
  }

  input[type=range]::-webkit-slider-thumb {
    -webkit-appearance: none;
    appearance: none;
    width: 18px;
    height: 18px;
    border-radius: 999px;
    background:
      radial-gradient(circle at 30% 0%, #e5e7eb, #93c5fd);
    border: 2px solid #22d3ee;
    box-shadow:
      0 0 8px rgba(34,211,238,0.8),
      0 6px 12px rgba(15,23,42,1);
    cursor: pointer;
    margin-top: -6px;
  }

  input[type=range]::-moz-range-thumb {
    width: 18px;
    height: 18px;
    border-radius: 999px;
    background:
      radial-gradient(circle at 30% 0%, #e5e7eb, #93c5fd);
    border: 2px solid #22d3ee;
    box-shadow:
      0 0 8px rgba(34,211,238,0.8),
      0 6px 12px rgba(15,23,42,1);
    cursor: pointer;
  }

  .grid-buttons {
    display: grid;
    grid-template-columns: repeat(3, minmax(0, 1fr));
    gap: 6px;
    margin-top: 6px;
  }

  .grid-buttons-row {
    display: grid;
    grid-template-columns: repeat(2, minmax(0, 1fr));
    gap: 6px;
    margin-top: 6px;
  }

  button { border: none; cursor: pointer; }

  .btn {
    position: relative;
    height: 32px;
    border-radius: 999px;
    font-size: 11px;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.08em;
    color: #e5e7eb;
    background: radial-gradient(circle at 0% 0%, #111827, #020617);
    border: 1px solid rgba(148,163,184,0.6);
    box-shadow:
      0 10px 18px rgba(15,23,42,0.95),
      inset 0 0 4px rgba(0,0,0,1);
    -webkit-tap-highlight-color: transparent;
    transition:
      transform 0.08s ease,
      box-shadow 0.08s ease,
      border-color 0.12s ease,
      background 0.12s ease,
      color 0.12s ease;
  }

  .btn:active {
    transform: translateY(2px);
    box-shadow:
      0 4px 10px rgba(0,0,0,1),
      inset 0 0 6px rgba(0,0,0,1);
  }

  .btn-primary {
    background: linear-gradient(135deg, #22d3ee, #3b82f6);
    border-color: rgba(56,189,248,0.9);
    color: #020617;
  }

  .btn-secondary { border-color: rgba(148,163,184,0.8); }

  .btn-accent {
    background: linear-gradient(135deg, #a855f7, #ec4899);
    border-color: rgba(236,72,153,0.9);
  }

  .btn-safe {
    background: linear-gradient(135deg, #22c55e, #a3e635);
    border-color: rgba(34,197,94,0.9);
    color: #020617;
  }

  .btn-danger {
    background: linear-gradient(135deg, #fb7185, #b91c1c);
    border-color: rgba(248,113,113,0.9);
  }

  .btn-outline {
    background: radial-gradient(circle at 0% 0%, #020617, #020617);
    border-style: dashed;
  }

  .meta-row {
    display: flex;
    justify-content: space-between;
    align-items: center;
    margin-top: 8px;
    font-size: 10px;
    color: var(--muted);
    gap: 8px;
    flex-wrap: wrap;
  }

  .tag {
    padding: 2px 8px;
    border-radius: 999px;
    border: 1px solid rgba(148,163,184,0.45);
    background: linear-gradient(120deg, #020617, #020617);
  }

  .tag strong {
    color: var(--accent-cyan);
  }
</style>
</head>

<body>
<div class="shell">
  <div class="dashboard">
    <div class="content">
      <div class="header">
        <div class="title-block">
          <div class="title">SMART ARM</div>
          <div class="subtitle">
            <span class="status-dot"></span>
            Wi‑Fi · ESP32 · Live Control
          </div>
        </div>
        <div class="chip">Camera · Gamepad</div>
      </div>

      <div class="layout">
        <div class="panel">
          <div class="panel-header">
            <div class="panel-title">Joint Angles</div>
            <div class="pill">0 – 180 deg</div>
          </div>

          <div class="slider-group">
            <div class="slider-row">
              <div class="slider-label">Waist</div>
              <div class="slider-wrap">
                <input id="waist" type="range" min="0" max="180" value="50" oninput="updateLabel('waist')">
              </div>
              <div class="slider-value" id="waistVal">50°</div>
            </div>

            <div class="slider-row">
              <div class="slider-label">Shoulder</div>
              <div class="slider-wrap">
                <input id="shoulder" type="range" min="0" max="180" value="90" oninput="updateLabel('shoulder')">
              </div>
              <div class="slider-value" id="shoulderVal">90°</div>
            </div>

            <div class="slider-row">
              <div class="slider-label">Elbow</div>
              <div class="slider-wrap">
                <input id="elbow" type="range" min="0" max="180" value="130" oninput="updateLabel('elbow')">
              </div>
              <div class="slider-value" id="elbowVal">130°</div>
            </div>

            <div class="slider-row">
              <div class="slider-label">Gripper</div>
              <div class="slider-wrap">
                <input id="gripper" type="range" min="0" max="180" value="0" oninput="updateLabel('gripper')">
              </div>
              <div class="slider-value" id="gripperVal">0°</div>
            </div>
          </div>

          <div class="grid-buttons-row">
            <button class="btn btn-secondary" onclick="resetSliders()">Reset sliders</button>
            <button class="btn btn-primary" onclick="applySliders()">Apply angles</button>
          </div>
        </div>

        <div class="panel">
          <div class="panel-header">
            <div class="panel-title">Actions</div>
            <div class="pill">Presets</div>
          </div>

          <div class="grid-buttons">
            <button class="btn btn-safe" onclick="sendCmd('H')">Home</button>
            <button class="btn btn-accent" onclick="sendCmd('w')">Wave</button>
            <button class="btn btn-secondary" onclick="sendCmd('P')">Prep</button>

            <button class="btn btn-secondary" onclick="sendCmd('G')">Grab pose</button>
            <button class="btn btn-secondary" onclick="sendCmd('O')">Open</button>
            <button class="btn btn-danger" onclick="sendCmd('C')">Close</button>
          </div>

          <div class="grid-buttons-row">
            <button class="btn btn-outline" onclick="sendCmd('L')">Base left</button>
            <button class="btn btn-outline" onclick="sendCmd('R')">Base right</button>
          </div>

          <div class="grid-buttons-row">
            <button class="btn btn-outline" onclick="sendCmd('u')">Arm up</button>
            <button class="btn btn-outline" onclick="sendCmd('d')">Arm down</button>
          </div>

          <div class="grid-buttons-row">
            <button class="btn btn-outline" onclick="sendCmd('f')">Elbow forward</button>
            <button class="btn btn-outline" onclick="sendCmd('b')">Elbow back</button>
          </div>
        </div>
      </div>

      <div class="meta-row">
        <div class="tag">
          Local web control + auto color sorting.
        </div>
        <div class="tag">
          Sliders control waist, shoulder, elbow, gripper in real time.
        </div>
      </div>
    </div>
  </div>
</div>

<script>
  function updateLabel(id) {
    var s = document.getElementById(id);
    var v = document.getElementById(id + "Val");
    if (s && v) {
      v.textContent = s.value + "°";
    }
  }

  function sendCmd(cmd) {
    var xhr = new XMLHttpRequest();
    xhr.open("GET", "/cmd?val=" + cmd + "&t=" + Date.now(), true);
    xhr.send();
  }

  function pad(v) {
    v = parseInt(v, 10);
    if (v < 0) v = 0;
    if (v > 180) v = 180;
    return ("00" + v).slice(-3);
  }

  function applySliders() {
    var w = document.getElementById("waist").value;
    var s = document.getElementById("shoulder").value;
    var e = document.getElementById("elbow").value;
    var g = document.getElementById("gripper").value;

    var cmds = ["W" + pad(w), "S" + pad(s), "E" + pad(e), "G" + pad(g)];
    cmds.forEach(function(c) {
      var xhr = new XMLHttpRequest();
      xhr.open("GET", "/cmd?val=" + c + "&t=" + Date.now(), true);
      xhr.send();
    });
  }

  function resetSliders() {
    var defaults = { waist: 50, shoulder: 90, elbow: 130, gripper: 0 };
    Object.keys(defaults).forEach(function(k) {
      var el = document.getElementById(k);
      if (el) {
        el.value = defaults[k];
        updateLabel(k);
      }
    });
  }

  ["waist","shoulder","elbow","gripper"].forEach(updateLabel);
</script>
</body>
</html>
)rawliteral";

// ============= Prototypes ============================
void handleRoot();
void handleCommand();
void handleSerialInput();
void parsePositionCommand(String line);
void executeManualCommand(char cmd);
void updateServosSmooth();
void waveHello();
void runRealTimeAutoMode();
void performPickAndPlace(char color);
void goHome();
void setTargetPose(int w, int s, int e, int g);
void readColorFast();
bool isRed();
bool isGreen();
bool isBlue();
void setLedColor(bool red, bool green, bool blue);

// ===================== GLOBAL ========================
String serialBuffer = "";

// ===================== SETUP =========================
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
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("\nWiFi Failed! Running Offline.");
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

  pinMode(PIN_LED_RED,   OUTPUT);
  pinMode(PIN_LED_GREEN, OUTPUT);
  pinMode(PIN_LED_BLUE,  OUTPUT);
  setLedColor(LOW, LOW, LOW);

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(S3, OUTPUT);
  pinMode(S_OUT, INPUT);
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);

  Wire.begin(21, 22);
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    for(;;);
  }

  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 100);
  roboEyes.setAutoblinker(ON, 3, 2);
  roboEyes.setIdleMode(ON, 2, 2);
  roboEyes.setMood(ROBO_DEFAULT);

  goHome();
  servoWAIST.write(curW);
  servoSHOULDER.write(curS);
  servoELBOW.write(curE);
  servoGRIPPER.write(curG);
}

// ===================== LOOP ==========================
void loop() {
  server.handleClient();
  roboEyes.update();

  handleSerialInput();
  updateServosSmooth();

  if (millis() - lastManualCommandTime > MANUAL_TIMEOUT) {
    runRealTimeAutoMode();
  }
}

// ===================== WEB HANDLERS ==================
void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.send(200, "text/html", htmlPage);
}

void handleCommand() {
  if (server.hasArg("val")) {
    String v = server.arg("val");
    lastManualCommandTime = millis();

    if (v.length() == 4 &&
        (v.charAt(0) == 'W' || v.charAt(0) == 'S' ||
         v.charAt(0) == 'E' || v.charAt(0) == 'G')) {
      parsePositionCommand(v);
    }
    else if (v.length() == 1) {
      char cmd = v.charAt(0);
      if (cmd == 'w') {
        waveHello();
      } else {
        executeManualCommand(cmd);
      }
    }

    server.send(200, "text/plain", "OK");
  } else {
    server.send(400, "text/plain", "No cmd");
  }
}

// ===================== SERIAL INPUT ==================
void handleSerialInput() {
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r') continue;

    if (ch == '\n') {
      if (serialBuffer.length() > 0) {
        if (serialBuffer.length() >= 2 &&
            (serialBuffer.charAt(0) == 'W' ||
             serialBuffer.charAt(0) == 'S' ||
             serialBuffer.charAt(0) == 'E' ||
             serialBuffer.charAt(0) == 'G')) {
          parsePositionCommand(serialBuffer);
        } else if (serialBuffer.length() == 1) {
          executeManualCommand(serialBuffer.charAt(0));
        }
        serialBuffer = "";
      }
    } else {
      serialBuffer += ch;
      if (serialBuffer.length() > 10) serialBuffer = "";
    }
  }
}

void parsePositionCommand(String line) {
  char id = line.charAt(0);
  int val = line.substring(1).toInt();
  val = constrain(val, 0, 180);

  lastManualCommandTime = millis();

  switch (id) {
    case 'W': targetW = val; break;
    case 'S': targetS = val; break;
    case 'E': targetE = val; break;
    case 'G': targetG = val; break;
  }
}

// ===================== SMOOTH SERVO UPDATE ===========
void updateServosSmooth() {
  unsigned long now = millis();
  if (now - lastServoUpdate < UPDATE_INTERVAL) return;
  lastServoUpdate = now;

  if (curW != targetW) {
    int diff = abs(targetW - curW);
    int step = (diff > 20) ? STEP_FAST : STEP_SLOW;
    if (targetW > curW) curW = min(curW + step, targetW);
    else                curW = max(curW - step, targetW);
    servoWAIST.write(curW);
  }

  if (curS != targetS) {
    int diff = abs(targetS - curS);
    int step = (diff > 20) ? STEP_FAST : STEP_SLOW;
    if (targetS > curS) curS = min(curS + step, targetS);
    else                curS = max(curS - step, targetS);
    servoSHOULDER.write(curS);
  }

  if (curE != targetE) {
    int diff = abs(targetE - curE);
    int step = (diff > 20) ? STEP_FAST : STEP_SLOW;
    if (targetE > curE) curE = min(curE + step, targetE);
    else                curE = max(curE - step, targetE);
    servoELBOW.write(curE);
  }

  if (curG != targetG) {
    int diff = abs(targetG - curG);
    int step = (diff > 20) ? STEP_FAST : STEP_SLOW;
    if (targetG > curG) curG = min(curG + step, targetG);
    else                curG = max(curG - step, targetG);
    servoGRIPPER.write(curG);
  }
}

// ===================== MANUAL CONTROL ================
void executeManualCommand(char cmd) {
  int stepSmall = 5;
  int stepBig   = 15;

  switch (cmd) {
    case 'l': targetW = constrain(targetW + stepSmall, 0, 180); break;
    case 'r': targetW = constrain(targetW - stepSmall, 0, 180); break;
    case 'L': targetW = constrain(targetW + stepBig, 0, 180); break;
    case 'R': targetW = constrain(targetW - stepBig, 0, 180); break;

    case 'u':
      targetS = constrain(targetS + stepSmall, 90, 160);
      targetE = constrain(targetE + stepSmall / 2, 90, 180);
      break;
    case 'd':
      targetS = constrain(targetS - stepSmall, 90, 160);
      targetE = constrain(targetE - stepSmall / 2, 90, 180);
      break;
    case 'f':
      targetE = constrain(targetE + stepSmall, 90, 180);
      break;
    case 'b':
      targetE = constrain(targetE - stepSmall, 90, 180);
      break;

    case 'O':
      targetG = GRIPPER_OPEN;
      roboEyes.setMood(HAPPY);
      break;
    case 'C':
      targetG = GRIPPER_CLOSED;
      roboEyes.setMood(ANGRY);
      break;

    case 'H':
      goHome();
      break;
    case 'P':
      setTargetPose(PREP_W, PREP_S, PREP_E, PREP_G);
      break;
    case 'G':
      setTargetPose(GRAB_W, GRAB_S, GRAB_E, GRAB_G);
      break;
  }
}

// ===================== HELLO MOTION ==================
void waveHello() {
  roboEyes.setMood(HAPPY);

  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(30, 20);
  display.print("Hi");
  display.display();

  int originalW = curW;

  for (int i = 0; i < 3; i++) {
    servoWAIST.write(constrain(originalW + 20, 0, 180));
    delay(120);
    roboEyes.update();
    servoWAIST.write(constrain(originalW - 20, 0, 180));
    delay(120);
    roboEyes.update();
  }

  curW = originalW;
  servoWAIST.write(curW);

  delay(250);

  display.clearDisplay();
  display.display();
  roboEyes.setMood(ROBO_DEFAULT);
}

// ===================== AUTO MODE =====================
void runRealTimeAutoMode() {
  readColorFast();

  Serial.print("R="); Serial.print(redVal);
  Serial.print(" G="); Serial.print(greenVal);
  Serial.print(" B="); Serial.println(blueVal);

  char currentColor = 'N';

  if (redVal > COLOR_THRESHOLD && greenVal > COLOR_THRESHOLD && blueVal > COLOR_THRESHOLD) {
    currentColor = 'N';
  } else if (isRed()) {
    currentColor = 'R';
  } else if (isGreen()) {
    currentColor = 'G';
  } else if (isBlue()) {
    currentColor = 'B';
  }

  if (currentColor == 'R') {
    setLedColor(HIGH, LOW, LOW);
    roboEyes.setMood(HAPPY);
  } else if (currentColor == 'G') {
    setLedColor(LOW, HIGH, LOW);
    roboEyes.setMood(HAPPY);
  } else if (currentColor == 'B') {
    setLedColor(LOW, LOW, HIGH);
    roboEyes.setMood(TIRED);
  } else {
    setLedColor(LOW, LOW, LOW);
    roboEyes.setMood(ROBO_DEFAULT);
  }

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

// ===================== PICK & PLACE ==================
void performPickAndPlace(char color) {
  setTargetPose(PREP_W, PREP_S, PREP_E, PREP_G);
  delay(350);

  setTargetPose(GRAB_W, GRAB_S, GRAB_E, PREP_G);
  delay(300);

  targetG = GRAB_G;
  delay(400);

  targetS = 90;
  targetE = 180;
  delay(300);

  int dropW, dropS, dropE;
  if (color == 'R') {
    dropW = 90; dropS = 160; dropE = 120;
  } else if (color == 'G') {
    dropW = 50; dropS = 150; dropE = 90;
  } else {
    dropW = 10; dropS = 160; dropE = 130;
  }

  setTargetPose(dropW, dropS, dropE, curG);
  delay(500);

  targetG = GRIPPER_OPEN;
  delay(300);

  targetS = 90;
  targetE = 130;
  delay(250);

  goHome();
  setLedColor(LOW, LOW, LOW);
}

// ===================== HELPERS =======================
void goHome() {
  setTargetPose(HOME_W, HOME_S, HOME_E, HOME_G);
}

void setTargetPose(int w, int s, int e, int g) {
  targetW = constrain(w, 0, 180);
  targetS = constrain(s, 0, 180);
  targetE = constrain(e, 0, 180);
  targetG = constrain(g, 0, 180);
}

void readColorFast() {
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  redVal = pulseIn(S_OUT, LOW, 8000UL);

  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  greenVal = pulseIn(S_OUT, LOW, 8000UL);

  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blueVal = pulseIn(S_OUT, LOW, 8000UL);
}

// ===================== COLOR LOGIC ===================
bool isRed()   { return (redVal   < COLOR_THRESHOLD && redVal   < greenVal && redVal   < blueVal); }
bool isGreen() { return (greenVal < COLOR_THRESHOLD && greenVal < redVal   && greenVal < blueVal); }
bool isBlue()  { return (blueVal  < COLOR_THRESHOLD && blueVal  < redVal   && blueVal  < greenVal); }

void setLedColor(bool red, bool green, bool blue) {
  digitalWrite(PIN_LED_RED,   red);
  digitalWrite(PIN_LED_GREEN, green);
  digitalWrite(PIN_LED_BLUE,  blue);
}
