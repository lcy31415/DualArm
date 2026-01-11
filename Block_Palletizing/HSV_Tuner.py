import os
import cv2
import numpy as np

BASE_DIR = os.path.dirname(__file__)

def open_camera():
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(1, cv2.CAP_MSMF)
    return cap if cap.isOpened() else None

def create_trackbars(win):
    cv2.createTrackbar("H1_L", win, 0, 180, lambda x: None)
    cv2.createTrackbar("S1_L", win, 100, 255, lambda x: None)
    cv2.createTrackbar("V1_L", win, 70, 255, lambda x: None)
    cv2.createTrackbar("H1_H", win, 10, 180, lambda x: None)
    cv2.createTrackbar("S1_H", win, 255, 255, lambda x: None)
    cv2.createTrackbar("V1_H", win, 255, 255, lambda x: None)
    cv2.createTrackbar("H2_L", win, 170, 180, lambda x: None)
    cv2.createTrackbar("S2_L", win, 100, 255, lambda x: None)
    cv2.createTrackbar("V2_L", win, 70, 255, lambda x: None)
    cv2.createTrackbar("H2_H", win, 180, 180, lambda x: None)
    cv2.createTrackbar("S2_H", win, 255, 255, lambda x: None)
    cv2.createTrackbar("V2_H", win, 255, 255, lambda x: None)

def get_thresholds(win):
    h1l = cv2.getTrackbarPos("H1_L", win)
    s1l = cv2.getTrackbarPos("S1_L", win)
    v1l = cv2.getTrackbarPos("V1_L", win)
    h1h = cv2.getTrackbarPos("H1_H", win)
    s1h = cv2.getTrackbarPos("S1_H", win)
    v1h = cv2.getTrackbarPos("V1_H", win)
    h2l = cv2.getTrackbarPos("H2_L", win)
    s2l = cv2.getTrackbarPos("S2_L", win)
    v2l = cv2.getTrackbarPos("V2_L", win)
    h2h = cv2.getTrackbarPos("H2_H", win)
    s2h = cv2.getTrackbarPos("S2_H", win)
    v2h = cv2.getTrackbarPos("V2_H", win)
    low1 = [h1l, s1l, v1l]
    high1 = [h1h, s1h, v1h]
    low2 = [h2l, s2l, v2l]
    high2 = [h2h, s2h, v2h]
    return low1, high1, low2, high2

def apply_mask(hsv, low1, high1, low2, high2):
    m1 = cv2.inRange(hsv, np.array(low1, np.uint8), np.array(high1, np.uint8))
    m2 = cv2.inRange(hsv, np.array(low2, np.uint8), np.array(high2, np.uint8))
    mask = cv2.bitwise_or(m1, m2)
    k = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask

DEFAULTS = {
    "red": {
        "low1": [0, 100, 70], "high1": [10, 255, 255],
        "low2": [170, 100, 70], "high2": [180, 255, 255]
    },
    "green": {
        "low1": [35, 100, 70], "high1": [85, 255, 255],
        "low2": [0, 0, 0], "high2": [0, 0, 0]
    },
    "blue": {
        "low1": [100, 100, 70], "high1": [140, 255, 255],
        "low2": [0, 0, 0], "high2": [0, 0, 0]
    }
}

def set_trackbars(win, params):
    cv2.setTrackbarPos("H1_L", win, params["low1"][0])
    cv2.setTrackbarPos("S1_L", win, params["low1"][1])
    cv2.setTrackbarPos("V1_L", win, params["low1"][2])
    cv2.setTrackbarPos("H1_H", win, params["high1"][0])
    cv2.setTrackbarPos("S1_H", win, params["high1"][1])
    cv2.setTrackbarPos("V1_H", win, params["high1"][2])
    cv2.setTrackbarPos("H2_L", win, params["low2"][0])
    cv2.setTrackbarPos("S2_L", win, params["low2"][1])
    cv2.setTrackbarPos("V2_L", win, params["low2"][2])
    cv2.setTrackbarPos("H2_H", win, params["high2"][0])
    cv2.setTrackbarPos("S2_H", win, params["high2"][1])
    cv2.setTrackbarPos("V2_H", win, params["high2"][2])

def main():
    cap = open_camera()
    if cap is None:
        print("摄像头打开失败")
        return
    print("摄像头预览启动")
    win = "HSV Tuner"
    cv2.namedWindow(win)
    create_trackbars(win)
    label = "red"
    set_trackbars(win, DEFAULTS[label])
    while True:
        ret, frame = cap.read()
        if not ret:
            cv2.waitKey(10)
            continue
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        low1, high1, low2, high2 = get_thresholds(win)
        mask = apply_mask(hsv, low1, high1, low2, high2)
        out = frame.copy()
        cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        for c in cnts:
            area = cv2.contourArea(c)
            if area < 300:
                continue
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            cv2.drawContours(out, [approx], -1, (0,255,255), 2)
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
            else:
                r = cv2.minAreaRect(c)
                cx, cy = int(r[0][0]), int(r[0][1])
            cv2.circle(out, (cx, cy), 6, (0,255,255), -1)
        cv2.putText(out, f"label:{label}  r/g/b: switch  s:print  q:quit", (10,30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.imshow("HSV Mask", mask)
        cv2.imshow(win, out)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'):
            break
        elif k == ord('r'):
            label = "red"
            set_trackbars(win, DEFAULTS[label])
        elif k == ord('g'):
            label = "green"
            set_trackbars(win, DEFAULTS[label])
        elif k == ord('b'):
            label = "blue"
            set_trackbars(win, DEFAULTS[label])
        elif k == ord('s'):
            low1, high1, low2, high2 = get_thresholds(win)
            print(f"{label} HSV:")
            print(f"  low1={low1} high1={high1}")
            print(f"  low2={low2} high2={high2}")
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
