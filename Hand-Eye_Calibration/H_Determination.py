import os
import cv2
import numpy as np

def load_params():
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    cam = np.load(os.path.join(root, "camera_params.npz"))
    he = np.load(os.path.join(root, "handeye_result.npz"))
    mtx = cam["mtx"].astype(np.float64)
    dist = cam["dist"].astype(np.float64)
    Rc2b = he["Rc2b"].astype(np.float64)
    tc2b = he["tc2b"].astype(np.float64)
    return mtx, dist, Rc2b, tc2b

H = 762.0

def pixel_to_base_xy(cx, cy, mtx, dist, Rc2b, tc2b, cz=0.0, flip_y=True):
    pt = np.array([[[float(cx), float(cy)]]], dtype=np.float64)
    und = cv2.undistortPoints(pt, mtx, dist).reshape(-1,2)
    xn, yn = float(und[0,0]), float(und[0,1])
    dir_c = np.array([xn, yn, 1.0], dtype=np.float64).reshape(3,1)
    dir_b = Rc2b @ dir_c
    org_b = tc2b.reshape(3,1)
    dz = float(dir_b[2,0])
    oz = float(org_b[2,0])
    if dz == 0.0:
        return None
    t = (H - oz)/dz
    pb = org_b + t*dir_b
    x, y = float(pb[0,0]), float(pb[1,0])
    if flip_y:
        y = -y
    return x, y, float(cz)

def main():
    global H
    mtx, dist, Rc2b, tc2b = load_params()
    print("标定数据加载成功。")
    print(f"当前几何计算平面高度设定为 H = {H:.1f} mm")
    z_in = input("请输入机械臂Z高度(mm)，回车默认0: ").strip()
    z_val = 0.0 if z_in == "" else float(z_in)
    pix = input("请输入像素坐标 'u v'，回车退出: ").strip()
    if not pix:
        return
    parts = pix.split()
    if len(parts) < 2:
        return
    try:
        cx = float(parts[0]); cy = float(parts[1])
    except:
        return
    win = "H Tuner"
    cv2.namedWindow(win)
    cv2.createTrackbar("H(mm)", win, int(H), 2000, lambda x: None)
    last_h = -1
    canvas = np.zeros((240, 800, 3), dtype=np.uint8)
    while True:
        h_now = cv2.getTrackbarPos("H(mm)", win)
        if h_now != last_h:
            H = float(h_now)
            res = pixel_to_base_xy(cx, cy, mtx, dist, Rc2b, tc2b, cz=z_val, flip_y=True)
            canvas[:] = 0
            if res is None:
                cv2.putText(canvas, "转换失败", (20,120), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,0,255), 2)
            else:
                x, y, zz = res
                text1 = f"H={H:.1f} mm  Z={zz:.1f} mm"
                text2 = f"Pixel({cx:.1f},{cy:.1f}) -> Base({x:.2f},{y:.2f},{zz:.2f})"
                cv2.putText(canvas, text1, (20,80), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)
                cv2.putText(canvas, text2, (20,140), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
                print(f"像素({cx:.1f}, {cy:.1f}) -> 机械臂({x:.2f}, {y:.2f}, {zz:.2f})")
            last_h = h_now
        cv2.imshow(win, canvas)
        if (cv2.waitKey(30) & 0xFF) == ord('q'):
            break
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
