#include <ESP32Servo.h>

// =====================================================
// 1. تعريف المنافذ (نفس ربطك الحالي)
// =====================================================
#define PIN_WAIST    13
#define PIN_SHOULDER 12  // تأكدي من فصل السلك أثناء الرفع إذا علق
#define PIN_ELBOW    14  // تأكدي من فصل السلك أثناء الرفع إذا علق
#define PIN_GRIPPER  27

// كائنات السيرفو
Servo servoWaist;
Servo servoShoulder;
Servo servoElbow;
Servo servoGripper;

// متغيرات لحفظ المواقع الحالية
int valWaist = 90;
int valShoulder = 90;
int valElbow = 90;
int valGripper = 90;

void setup() {
  Serial.begin(115200);
  while(!Serial); // انتظار فتح السيريال

  Serial.println("\n==========================================");
  Serial.println("   🛠️ ROBOT ARM CALIBRATION TOOL 🛠️");
  Serial.println("==========================================");
  Serial.println("Commands format:");
  Serial.println("  w90  -> Move Waist to 90");
  Serial.println("  s45  -> Move Shoulder to 45");
  Serial.println("  e120 -> Move Elbow to 120");
  Serial.println("  g180 -> Open Gripper to 180");
  Serial.println("------------------------------------------");

  // إعداد التردد 50Hz (مهم جداً للاستقرار)
  servoWaist.setPeriodHertz(50);
  servoShoulder.setPeriodHertz(50);
  servoElbow.setPeriodHertz(50);
  servoGripper.setPeriodHertz(50);

  // ربط السيرفوات
  servoWaist.attach(PIN_WAIST, 500, 2400);
  servoShoulder.attach(PIN_SHOULDER, 500, 2400);
  servoElbow.attach(PIN_ELBOW, 500, 2400);
  servoGripper.attach(PIN_GRIPPER, 500, 2400);

  // البدء في وضع المنتصف الآمن (شكل حرف L تقريباً)
  moveServo(servoWaist, 90);
  moveServo(servoShoulder, 90);
  moveServo(servoElbow, 90);
  moveServo(servoGripper, 90);
  
  printStatus();
}

void loop() {
  if (Serial.available() > 0) {
    char motor = Serial.read(); // قراءة الحرف (w, s, e, g)
    int angle = Serial.parseInt(); // قراءة الرقم

    // تجاهل الأوامر الفارغة
    if (motor == '\n' || motor == '\r') return;

    // حماية الحدود (0 - 180)
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    // تنفيذ الحركة
    switch(motor) {
      case 'w': case 'W':
        valWaist = angle;
        moveServo(servoWaist, valWaist);
        Serial.print("✅ Waist moved to: "); Serial.println(valWaist);
        break;
        
      case 's': case 'S':
        valShoulder = angle;
        moveServo(servoShoulder, valShoulder);
        Serial.print("✅ Shoulder moved to: "); Serial.println(valShoulder);
        break;
        
      case 'e': case 'E':
        valElbow = angle;
        moveServo(servoElbow, valElbow);
        Serial.print("✅ Elbow moved to: "); Serial.println(valElbow);
        break;
        
      case 'g': case 'G':
        valGripper = angle;
        moveServo(servoGripper, valGripper);
        Serial.print("✅ Gripper moved to: "); Serial.println(valGripper);
        break;
        
      default:
        // لا تطبع خطأ إذا كانت مجرد مسافات
        break;
    }
  }
}

// دالة تحريك ناعمة
void moveServo(Servo &sv, int target) {
  sv.write(target);
  delay(15);
}

// طباعة الحالة الحالية
void printStatus() {
  Serial.print("Current: W="); Serial.print(valWaist);
  Serial.print(" | S="); Serial.print(valShoulder);
  Serial.print(" | E="); Serial.print(valElbow);
  Serial.print(" | G="); Serial.println(valGripper);
}