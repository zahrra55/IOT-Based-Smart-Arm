#include <Servo.h>

// =====================================================
// === SERVO SETUP (4 DOF ARM) =========================
// =====================================================
Servo servoWAIST, servoSHOULDER, servoELBOW, servoGRIPPER;

// **UPDATED:** Initial positions matching the HOME state
int posWAIST = 40, posSHOULDER = 10, posELBOW = 130, posGRIPPER = 0;

// =====================================================
// === COLOR SENSOR (TCS3200) SETUP ====================
// =====================================================
const int s0 = 12, s1 = 13, s2 = 10, s3 = 8, outPin = 11;
int redVal = 0, greenVal = 0, blueVal = 0;

// =====================================================
// === GRIPPER ANGLES (BASED ON YOUR CALIBRATION) ======
// =====================================================
const int GRIPPER_OPEN = 180;   // Your value for OPEN
const int GRIPPER_CLOSED = 0;   // Your value for CLOSED

// =====================================================
// === INITIAL SETUP ===================================
// =====================================================
void setup() {
  Serial.begin(9600);

  // Attach all servos
  servoWAIST.attach(3);
  servoSHOULDER.attach(5);
  servoELBOW.attach(6);
  servoGRIPPER.attach(9);

  // Setup the color sensor
  pinMode(s0, OUTPUT); pinMode(s1, OUTPUT);
  digitalWrite(s0, HIGH); digitalWrite(s1, LOW); // 20% frequency scaling
  pinMode(s2, OUTPUT); pinMode(s3, OUTPUT);
  pinMode(outPin, INPUT);

  // **UPDATED:** Moving to your custom HOME position
  Serial.println("Moving to HOME position...");
  servoWAIST.write(posWAIST);
  servoSHOULDER.write(posSHOULDER);
  servoELBOW.write(posELBOW);
  servoGRIPPER.write(posGRIPPER);
  delay(2000);

  Serial.println("Robot Arm Ready ✅");
}

// =====================================================
// === MAIN LOOP =======================================
// =====================================================
void loop() {
  readColorAverage(5);  // read averaged color values

  Serial.print("R: "); Serial.print(redVal);
  Serial.print(" | G: "); Serial.print(greenVal);
  Serial.print(" | B: "); Serial.println(blueVal);

  if (isRed()) {
    Serial.println("Detected RED box");
    colorTask(90); // Call task with RED drop-off angle
  } 
  else if (isGreen()) {
    Serial.println("Detected GREEN box");
    colorTask(50); // Call task with GREEN drop-off angle
  } 
  else if (isBlue()) {
    Serial.println("Detected BLUE box");
    colorTask(10); // Call task with BLUE drop-off angle
  } 
  else {
    Serial.println("No valid color detected.");
  }

  delay(1000); // Wait before next scan
}

// =====================================================
// === COLOR SENSOR READING FUNCTIONS ==================
// =====================================================
void readColorAverage(int samples) {
  long rSum = 0, gSum = 0, bSum = 0;
  for (int i = 0; i < samples; ++i) {
    rSum += readColorValue(LOW, LOW);    // red
    gSum += readColorValue(HIGH, HIGH);  // green
    bSum += readColorValue(LOW, HIGH);   // blue
    delay(30);
  }
  redVal = rSum / samples;
  greenVal = gSum / samples;
  blueVal = bSum / samples;
}

int readColorValue(bool s2val, bool s3val) {
  digitalWrite(s2, s2val);
  digitalWrite(s3, s3val);
  delay(40);
  unsigned long val = pulseIn(outPin, LOW, 30000UL);
  if (val == 0) return 30000;
  return (int)val;
}

// =====================================================
// === COLOR DETECTION (RATIO BASED) ===================
// =====================================================
// (No change here - these functions seem logically correct)
bool isRed() {
  if (redVal > 2000) return false;
  return ((float)redVal < (float)greenVal * 0.75 && (float)redVal < (float)blueVal * 0.75);
}
bool isGreen() {
  if (greenVal > 2000) return false;
  return ((float)greenVal < (float)redVal * 0.75 && (float)greenVal < (float)blueVal * 0.75);
}
bool isBlue() {
  if (blueVal > 2000) return false;
  return ((float)blueVal < (float)redVal * 0.75 && (float)blueVal < (float)greenVal * 0.75);
}

// =====================================================
// === SERVO MOTION HELPERS ============================
// =====================================================
void moveSmooth(Servo &servo, int &currentPos, int targetPos) {
  int step = (targetPos > currentPos) ? 1 : -1;
  for (int pos = currentPos; pos != targetPos; pos += step) {
    servo.write(pos);
    delay(8); // Motion speed
  }
  currentPos = targetPos;
  servo.write(currentPos);
}

void moveArm(int waist, int shoulder, int elbow, int gripperAngle) {
  moveSmooth(servoWAIST, posWAIST, waist);
  moveSmooth(servoSHOULDER, posSHOULDER, shoulder);
  moveSmooth(servoELBOW, posELBOW, elbow);
  // Gripper movement doesn't need smoothing
  posGRIPPER = constrain(gripperAngle, 0, 180);
  servoGRIPPER.write(posGRIPPER);
  delay(10);
}

// =====================================================
// === MAIN PICK & PLACE SEQUENCE (YOUR LOGIC) =========
// =====================================================
// **This function was completely rewritten**
void colorTask(int dropWaistAngle) {
  
  // --- 1. Pick-up Sequence ---
  Serial.println("Moving to pick position...");
  // Move to pick position with gripper open
  // W=135, S=60, E=160, G=160 (Open)
  moveArm(130, 60, 175, GRIPPER_OPEN);
  delay(1000); // Wait to arrive

  // --- 2. Close Gripper ---
  Serial.println("Gripping object...");
  servoGRIPPER.write(GRIPPER_CLOSED);
  posGRIPPER = GRIPPER_CLOSED;

  delay(800); // Wait to grip

  // --- 3. Transport and Drop Sequence ---
  Serial.println("Lifting object...");
  // Lift first (E=190)
  moveArm(130, 55, 199, GRIPPER_CLOSED);
  delay(700);

  Serial.println("Rotating to drop zone...");
  // Rotate to drop zone (while maintaining height)
  moveArm(dropWaistAngle, 60, 190, GRIPPER_CLOSED);
  delay(1000); // Wait for rotation to finish

  Serial.println("Lowering object...");
  // Lower to place object (E=160)
  moveArm(dropWaistAngle, 60, 160, GRIPPER_CLOSED);
  delay(700);

  // --- 4. Release Object ---
  Serial.println("Releasing object...");
  servoGRIPPER.write(GRIPPER_OPEN);
  posGRIPPER = GRIPPER_OPEN;
  delay(600);

  // --- 5. Return to HOME ---
  Serial.println("Returning to HOME...");
  // (Optional: Lift arm slightly before returning)
  moveArm(dropWaistAngle, 60, 180, GRIPPER_OPEN);
  delay(500);
  
  // Return to original HOME position
  // W=40, S=10, E=130, G=0 (Closed)
  moveArm(40, 10, 130, GRIPPER_CLOSED);
  delay(1000);
}