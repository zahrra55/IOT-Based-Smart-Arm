#include <Wire.h>
#include "BluetoothSerial.h"
#include <ESP32Servo.h> 

// تعريف المنافذ (نفس ربطك الحالي)
#define S0 32
#define S1 33
#define S2 25
#define S3 26
#define S_OUT 35

// متغيرات
int red = 0;
int green = 0;
int blue = 0;

void setup() {
  Serial.begin(115200);
  
  // إعداد الحساس
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT); pinMode(S3, OUTPUT);
  pinMode(S_OUT, INPUT);
  
  // تردد 20%
  digitalWrite(S0, HIGH);
  digitalWrite(S1, LOW);
  
  Serial.println("--- Color Calibration Tool Started ---");
  Serial.println("Put RED object -> Note the numbers");
  Serial.println("Put GREEN object -> Note the numbers");
}

void loop() {
  // قراءة الأحمر
  digitalWrite(S2, LOW);
  digitalWrite(S3, LOW);
  red = pulseIn(S_OUT, LOW);
  delay(20);
  
  // قراءة الأخضر
  digitalWrite(S2, HIGH);
  digitalWrite(S3, HIGH);
  green = pulseIn(S_OUT, LOW);
  delay(20);
  
  // قراءة الأزرق
  digitalWrite(S2, LOW);
  digitalWrite(S3, HIGH);
  blue = pulseIn(S_OUT, LOW);
  delay(20);

  // طباعة القيم بشكل واضح
  Serial.print("RED Value: "); Serial.print(red);
  Serial.print("\t GREEN Value: "); Serial.print(green);
  Serial.print("\t BLUE Value: "); Serial.println(blue);
  
  // تحليل بسيط للمساعدة
  if(red < green && red < blue && red < 500) Serial.println("-> Detected: RED?");
  else if(green < red && green < blue && green < 500) Serial.println("-> Detected: GREEN?");
  else if(blue < red && blue < green && blue < 500) Serial.println("-> Detected: BLUE?");
  else Serial.println("-> No Color / Unknown");

  Serial.println("-------------------------------------");
  delay(1000); // تحديث كل ثانية
}