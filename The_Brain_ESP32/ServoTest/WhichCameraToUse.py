import cv2

# جربي تغيير هذا الرقم (0، 1، 2) حتى تظهر الصورة
camera_index = 0 

cap = cv2.VideoCapture(camera_index)

while True:
    success, img = cap.read()
    if success:
        cv2.imshow("Test Camera", img)
    else:
        print("Camera not found or blocked!")
        break
        
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()