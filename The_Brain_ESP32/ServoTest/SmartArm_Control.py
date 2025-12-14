import cv2
import mediapipe as mp
import serial
import time
import math

# =====================================================
# إعدادات الاتصال
# =====================================================
COM_PORT = 'COM12'  # <--- منفذك الصحيح
BAUD_RATE = 115200

try:
    esp = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)
    time.sleep(2) 
    print(f"✅ Connected to {COM_PORT} Successfully!")
except:
    print(f"❌ Error: Could not connect to {COM_PORT}.")
    exit()

# =====================================================
# إعدادات الذكاء الاصطناعي (زدنا الحساسية)
# =====================================================
mp_hands = mp.solutions.hands
# قللنا الثقة المطلوبة إلى 0.3 ليعمل حتى في الإضاءة الضعيفة
hands = mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.3, min_tracking_confidence=0.3)
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(0) # تأكد أن الرقم 0 هو كاميرا هاتفك
cap.set(3, 640)
cap.set(4, 480)

last_sent_time = 0
delay_between_commands = 0.1 

print("System Ready... Waiting for Hand...")

while True:
    success, img = cap.read()
    if not success: break
    
    img = cv2.flip(img, 1)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)
    
    command = None
    status_text = "Waiting for Hand..."
    
    h, w, c = img.shape
    
    # اذا اكتشف اليد
    if results.multi_hand_landmarks:
        status_text = "Hand Detected!" # رسالة تأكيد
        
        for hand_lms in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(img, hand_lms, mp_hands.HAND_CONNECTIONS)
            
            index_x = hand_lms.landmark[8].x
            index_y = hand_lms.landmark[8].y
            thumb_x = hand_lms.landmark[4].x
            thumb_y = hand_lms.landmark[4].y
            
            distance = math.hypot(index_x - thumb_x, index_y - thumb_y)
            
            if distance < 0.05:
                command = 'C'
                status_text = "Action: CLOSE Gripper"
            elif distance > 0.15:
                command = 'O'
                status_text = "Action: OPEN Gripper"
            else:
                if index_x < 0.3:
                    command = 'l'
                    status_text = "Action: LEFT"
                elif index_x > 0.7:
                    command = 'r'
                    status_text = "Action: RIGHT"
                elif index_y < 0.3:
                    command = 'u'
                    status_text = "Action: UP"
                elif index_y > 0.7:
                    command = 'd'
                    status_text = "Action: DOWN"
                else:
                    status_text = "Hand Center (No Move)"

    # طباعة الحالة على الفيديو
    cv2.putText(img, status_text, (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

    # إرسال الأمر وطباعته في التيرمنال
    if command and (time.time() - last_sent_time > delay_between_commands):
        try:
            esp.write(command.encode())
            print(f"📤 Sent Command: {command}") # <--- هذا السطر سيخبرنا هل تم الإرسال
            last_sent_time = time.time()
        except Exception as e:
            print(f"Error: {e}")

    cv2.imshow("Smart Arm Controller", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
try:
    esp.close()
except:
    pass