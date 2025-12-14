import cv2

print("جاري فحص الكاميرات...")

# نجرب الأرقام من 0 إلى 3
for i in range(4):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        ret, frame = cap.read()
        if ret:
            print(f"✅ وجدنا كاميرا رقم: {i}")
            cv2.imshow(f"Camera {i}", frame)
            cv2.waitKey(1000) # تعرض الصورة لثانية واحدة
            cv2.destroyWindow(f"Camera {i}")
        else:
            print(f"❌ الكاميرا رقم {i} موجودة لكن لا تعطي صورة (قد تكون مشغولة).")
    else:
        print(f"❌ لا توجد كاميرا رقم {i}")
    cap.release()

print("انتهى الفحص.")