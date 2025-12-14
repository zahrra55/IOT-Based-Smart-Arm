import cv2
import mediapipe as mp
import serial
import time
import math

COM_PORT = 'COM12'   # عدّليها حسب منفذك
BAUD_RATE = 115200

esp = None
try:
    esp = serial.Serial(COM_PORT, BAUD_RATE, timeout=0.1)
    time.sleep(2)
    print(f"✅ Connected to {COM_PORT}")
except Exception as e:
    print(f"❌ Error connecting to {COM_PORT}: {e}")

mp_hands = mp.solutions.hands
hands = mp_hands.Hands(
    max_num_hands=1,
    min_detection_confidence=0.7,
    min_tracking_confidence=0.7
)
mp_draw = mp.solutions.drawing_utils

cap = cv2.VideoCapture(1)  # جرّبي 0 إن لزم
cap.set(3, 640)
cap.set(4, 480)

smooth_x = None
smooth_y = None
ALPHA = 0.4

last_send_time = 0
SEND_INTERVAL = 0.08  # ثانية تقريباً (حوالي 12 أمر/ث)

last_wave_time = 0
prev_open = False

def map_range(x, a1, a2, b1, b2):
    x = max(min(x, a2), a1)
    return b1 + (b2 - b1) * (x - a1) / (a2 - a1)

def send_pose(W=None, S=None, E=None, G=None):
    global last_send_time
    now = time.time()
    if now - last_send_time < SEND_INTERVAL:
        return
    last_send_time = now

    if esp is None or not esp.is_open:
        return

    try:
        if W is not None:
            esp.write(f"W{int(W):03d}\n".encode())
        if S is not None:
            esp.write(f"S{int(S):03d}\n".encode())
        if E is not None:
            esp.write(f"E{int(E):03d}\n".encode())
        if G is not None:
            esp.write(f"G{int(G):03d}\n".encode())
    except Exception as e:
        print(f"Serial error: {e}")

print("📷 Camera joystick started. Press 'q' to quit.")

while True:
    ok, img = cap.read()
    if not ok:
        print("❌ Camera error")
        break

    img = cv2.flip(img, 1)
    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    results = hands.process(img_rgb)

    status = "Idle"
    color = (255, 255, 255)

    if results.multi_hand_landmarks:
        for hand_lms in results.multi_hand_landmarks:
            mp_draw.draw_landmarks(img, hand_lms, mp_hands.HAND_CONNECTIONS)

            rx = hand_lms.landmark[8].x
            ry = hand_lms.landmark[8].y
            tx = hand_lms.landmark[4].x
            ty = hand_lms.landmark[4].y

            if smooth_x is None:
                smooth_x, smooth_y = rx, ry
            else:
                smooth_x = ALPHA * rx + (1 - ALPHA) * smooth_x
                smooth_y = ALPHA * ry + (1 - ALPHA) * smooth_y

            index_x = smooth_x
            index_y = smooth_y

            distance = math.hypot(index_x - tx, index_y - ty)

            # نحول موضع اليد إلى زوايا:
            # أفقياً -> خصر (0-180)
            waist_angle = map_range(index_x, 0.1, 0.9, 160, 20)
            # رأسيًا -> كتف (90-160 تقريباً)
            shoulder_angle = map_range(index_y, 0.1, 0.9, 120, 150)
            # الكوع نحركه عكس الكتف قليلاً
            elbow_angle = map_range(index_y, 0.1, 0.9, 90, 160)

            OPEN_DIST  = 0.10
            CLOSE_DIST = 0.07

            if distance > OPEN_DIST:
                gripper_angle = 180   # مفتوحة
                is_open = True
            elif distance < CLOSE_DIST:
                gripper_angle = 0     # مغلقة
                is_open = False
            else:
                gripper_angle = None
                is_open = prev_open

            # تلويح: يد مفتوحة تتحرك في الوسط => نرسل أمر "Hi" عبر زر WEB (نكتفي بحركة اليد + واجهة الويب)
            if is_open and 0.3 < index_x < 0.7 and 0.3 < index_y < 0.7:
                if time.time() - last_wave_time > 2.5:
                    # نبعث وضع HOME + HI عبر أمر ويب بديل إذا أردت فيما بعد
                    last_wave_time = time.time()
                status = "Waving / Open"
                color = (0, 255, 255)
            else:
                status = "Camera control"
                color = (0, 200, 255)

            send_pose(W=waist_angle, S=shoulder_angle, E=elbow_angle,
                      G=gripper_angle)

            prev_open = is_open

            cv2.putText(img,
                        f"W:{waist_angle:3.0f} S:{shoulder_angle:3.0f} E:{elbow_angle:3.0f}",
                        (20, 420), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (200, 200, 200), 1)

    cv2.rectangle(img, (192, 144), (448, 336), (0, 255, 0), 1)
    cv2.putText(img, "Center Zone", (200, 140),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

    cv2.putText(img, f"Status: {status}", (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    cv2.imshow("Smart Arm Joystick", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if esp is not None and esp.is_open:
    esp.close()
print("Closed.")
