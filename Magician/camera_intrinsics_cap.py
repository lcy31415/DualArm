import cv2
cap = cv2.VideoCapture(1) # 0是你的摄像头编号
i = 0
while True:
    ret, frame = cap.read()
    cv2.imshow('Calibration', frame)
    key = cv2.waitKey(1)
    if key == ord('s'): # 按 's' 保存
        cv2.imwrite(f'image_{i}.png', frame)
        print(f'Saved image_{i}')
        i += 1
    elif key == 27: # 按 ESC 退出
        break
cap.release()
cv2.destroyAllWindows()