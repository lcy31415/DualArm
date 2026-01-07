import cv2
import numpy as np
import glob

# ================= 配置参数 =================
# 4x4个方格，对应 3x3 个内角点
CHESSBOARD_SIZE = (3, 3) 
# 每个方格的边长（单位：mm），请量一下你的打印纸
SQUARE_SIZE = 25  

# 寻找角点的准则
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# 准备 3D 对象点 (0,0,0), (20,0,0), (40,0,0) ...
objp = np.zeros((CHESSBOARD_SIZE[0] * CHESSBOARD_SIZE[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHESSBOARD_SIZE[0], 0:CHESSBOARD_SIZE[1]].T.reshape(-1, 2)
objp = objp * SQUARE_SIZE

objpoints = [] # 3d points
imgpoints = [] # 2d points

# 加上 *.jpg 或 *.png 或 *.bmp，取决于你 AMCap 存的是什么格式
images = glob.glob(r'E:\DualArm\Magician\calibration\*.png')

if not images:
    print("没有找到图片，请确认路径是否正确！")
    exit()

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # 查找角点
    ret, corners = cv2.findChessboardCorners(gray, CHESSBOARD_SIZE, None)

    if ret:
        objpoints.append(objp)
        # 精细化角点坐标
        corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
        imgpoints.append(corners2)

        # 实时显示检测结果，方便你检查
        cv2.drawChessboardCorners(img, CHESSBOARD_SIZE, corners2, ret)
        cv2.imshow('Calibration Check', img)
        cv2.waitKey(200)

cv2.destroyAllWindows()

# ================= 执行标定 =================
if len(objpoints) > 0:
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)
    
    print("\n--- 标定成功！ ---")
    print(f"内参矩阵 (mtx):\n{mtx}")
    print(f"畸变系数 (dist):\n{dist}")
    print(f"平均重投影误差: {ret}") # 越接近0说明越准，一般应小于0.5
    
    # 保存内参
    np.savez("camera_params.npz", mtx=mtx, dist=dist)
else:
    print("标定失败：没有任何一张图片被成功识别角点。请检查 CHESSBOARD_SIZE 是否设置正确。")