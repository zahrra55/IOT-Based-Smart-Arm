#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <FluxGarage_RoboEyes.h>
#include <ESP32Servo.h> 

// =====================================================
// تعريف المنافذ (نفس ربطك الصحيح)
// =====================================================
#define PIN_WAIST    13
#define PIN_SHOULDER 12
#define PIN_ELBOW    14
#define PIN_GRIPPER  27

// الشاشة والعيون
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
RoboEyes<Adafruit_SSD1306> roboEyes(display);

Servo servoWAIST, servoSHOULDER, servoELBOW, servoGRIPPER;

// مواضع البداية
int posWAIST = 90;     
int posSHOULDER = 90;  
int posELBOW = 130;    
int posGRIPPER = 0;    

void setup() {
  Serial.begin(115200); 

  // التردد 50Hz
  servoWAIST.setPeriodHertz(50);
  servoSHOULDER.setPeriodHertz(50);
  servoELBOW.setPeriodHertz(50);
  servoGRIPPER.setPeriodHertz(50);

  servoWAIST.attach(PIN_WAIST, 500, 2400);
  servoSHOULDER.attach(PIN_SHOULDER, 500, 2400);
  servoELBOW.attach(PIN_ELBOW, 500, 2400);
  servoGRIPPER.attach(PIN_GRIPPER, 500, 2400);

  // تشغيل الشاشة
  Wire.begin(21, 22);
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) for(;;);
  
  roboEyes.begin(SCREEN_WIDTH, SCREEN_HEIGHT, 100);
  roboEyes.setAutoblinker(ON, 3, 2);
  roboEyes.setIdleMode(ON, 2, 2);
  roboEyes.setMood(DEFAULT); // الوضع الطبيعي

  moveArm();
  Serial.println("Ready for Hi! 👋");
}

void loop() {
  roboEyes.update(); // تحديث حركة العيون المستمر

  if (Serial.available()) {
    char cmd = Serial.read();
    executeCommand(cmd);
  }
}

void executeCommand(char cmd) {
  int step = 4; // سرعة التحكم اليدوي

  switch (cmd) {
    case 'l': 
      posWAIST = constrain(posWAIST + step, 0, 180); 
      // roboEyes.setMood(DEFAULT); 
      break;
    case 'r': 
      posWAIST = constrain(posWAIST - step, 0, 180); 
      break;
    case 'u': 
      posSHOULDER = constrain(posSHOULDER + step, 45, 160); 
      posELBOW = constrain(posELBOW + step/2, 90, 180); 
      break;
    case 'd': 
      posSHOULDER = constrain(posSHOULDER - step, 45, 160);
      posELBOW = constrain(posELBOW - step/2, 90, 180);
      break;
    case 'O': 
      posGRIPPER = 180; 
      roboEyes.setMood(DEFAULT);
      break;
    case 'C': 
      posGRIPPER = 0; 
      roboEyes.setMood(ANGRY); // يركز عند الإمساك
      break;
      
    // --- الأمر الجديد (الترحيب) ---
    case 'H': 
      waveHello(); 
      break;
  }
  
  if (cmd != 'H') moveArm(); // لا تتحرك حركة عادية إذا كنت تلوح
}

// --- دالة التلويح السعيد ---
void waveHello() {
  roboEyes.setMood(HAPPY); // عيون سعيدة
  
  // حفظ الموقع الحالي
  int originalWaist = posWAIST;
  
  // التلويح يمين ويسار مرتين بسرعة
  for(int i=0; i<2; i++) {
    servoWAIST.write(constrain(originalWaist + 20, 0, 180)); 
    delay(150);
    roboEyes.update(); // تحديث العيون أثناء الحركة
    
    servoWAIST.write(constrain(originalWaist - 20, 0, 180)); 
    delay(150);
    roboEyes.update();
  }
  
  // العودة للمركز
  servoWAIST.write(originalWaist);
  // roboEyes.setMood(DEFAULT); // اختياري: هل تريده يعود طبيعياً فوراً؟
}

void moveArm() {
  servoWAIST.write(posWAIST);
  servoSHOULDER.write(posSHOULDER);
  servoELBOW.write(posELBOW);
  servoGRIPPER.write(posGRIPPER);
}