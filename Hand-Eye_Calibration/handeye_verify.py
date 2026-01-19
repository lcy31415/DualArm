import os
import cv2
import numpy as np
import DobotDllType as dType

def wait_idx(api, last_index):
    while last_index > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)

def main():
    base_dir = os.path.dirname(__file__)
    matrix_path = os.path.join(base_dir, "affine_matrix_dual.npz")
    
    if not os.path.exists(matrix_path):
        print(f"找不到双臂矩阵文件: {matrix_path}")
        return
        
    data = np.load(matrix_path)
    
    # ----------------- 验证左臂 -----------------
    if 'affine_left' in data:
        M_left = data['affine_left']
        print("\n========== 验证 左臂 (Left Arm) ==========")
        # 请确保此处 PORT 是左臂的
        PORT_LEFT = input("请输入左臂串口 (例如 COM5): ").strip().upper()
        if PORT_LEFT:
             verify_arm(PORT_LEFT, M_left)
    
    # ----------------- 验证右臂 -----------------
    if 'affine_right' in data:
        M_right = data['affine_right']
        print("\n========== 验证 右臂 (Right Arm) ==========")
        PORT_RIGHT = input("请输入右臂串口 (例如 COM9): ").strip().upper()
        if PORT_RIGHT:
             verify_arm(PORT_RIGHT, M_right)

def verify_arm(port, M_affine):
    # 机械臂移动高度 (Z)
    APPROACH_Z = 10.0
    PATTERN_SIZE = (3, 3)
    
    api = dType.load()
    state = dType.ConnectDobot(api, port, 115200)[0]
    if state != dType.DobotConnect.DobotConnect_NoError:
        print(f"连接失败: {port}")
        return
    print(f"连接成功: {port}")
    
    dType.SetQueuedCmdClear(api)
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100, isQueued=1)
    dType.SetQueuedCmdStartExec(api)

    # 移动到安全高度
    pose = dType.GetPose(api)
    if pose[2] < APPROACH_Z:
        idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, pose[0], pose[1], APPROACH_Z, pose[3], isQueued=1)[0]
        wait_idx(api, idx)

    # 视觉识别
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("摄像头打开失败")
        dType.DisconnectDobot(api)
        return
    
    print(f"[{port}] 寻找棋盘格...")
    corners_pixel = None
    
    for _ in range(200):
        ret, frame = cap.read()
        if not ret: continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE, 
                                                   cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_pixel = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            cv2.drawChessboardCorners(frame, PATTERN_SIZE, corners_pixel, found)
            cv2.imshow("Verify", frame)
            if cv2.waitKey(500) != -1: pass
            break
        cv2.imshow("Verify", frame)
        cv2.waitKey(10)
    
    cap.release()
    cv2.destroyAllWindows()
    
    if corners_pixel is None:
        print("未检测到棋盘格")
        dType.DisconnectDobot(api)
        return

    # 执行验证移动
    pts_uv = corners_pixel.reshape(-1, 2)
    ones = np.ones((pts_uv.shape[0], 1))
    pts_uv_h = np.hstack([pts_uv, ones])
    pts_xy = (M_affine @ pts_uv_h.T).T
    
    for i in range(pts_xy.shape[0]):
        tx, ty = pts_xy[i]
        print(f"[{port}] 点{i} -> ({tx:.1f}, {ty:.1f})")
        
        if abs(tx) > 350 or abs(ty) > 350:
            print("警告：坐标越界，跳过")
            continue
            
        idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, tx, ty, APPROACH_Z, 0.0, isQueued=1)[0]
        wait_idx(api, idx)
        dType.dSleep(500)
        
    dType.DisconnectDobot(api)
    print(f"[{port}] 验证结束\n")

if __name__ == "__main__":
    main()
