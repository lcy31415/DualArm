import cv2
import numpy as np
import time
import os
import json

def get_image_points(pattern_size=(3,3)):
    """
    打开摄像头，检测棋盘格，返回角点像素坐标 (N, 2)
    """
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("无法打开摄像头")
        return None

    print(f"请将棋盘格放置在视野中，等待检测 {pattern_size} 角点...")
    corners_pixel = None
    
    # 尝试检测多次，直到稳定检测到
    for _ in range(500):
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue
            
        display_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        found, corners = cv2.findChessboardCorners(gray, pattern_size, 
                                                   cv2.CALIB_CB_ADAPTIVE_THRESH + 
                                                   cv2.CALIB_CB_NORMALIZE_IMAGE +
                                                   cv2.CALIB_CB_FAST_CHECK)
        
        if found:
            # 亚像素精确化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            corners_pixel = corners.reshape(-1, 2)
            
            # 绘制并显示
            cv2.drawChessboardCorners(display_frame, pattern_size, corners, found)
            cv2.putText(display_frame, "DETECTED! Press 's' to confirm and calculate.", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        else:
            cv2.putText(display_frame, "Searching...", (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
        cv2.imshow("Calibration Capture", display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('s') and found:
            print("已确认图像角点。")
            break
        elif key == ord('q'):
            print("用户取消。")
            corners_pixel = None
            break
            
    cap.release()
    cv2.destroyAllWindows()
    return corners_pixel

def load_robot_points(csv_path):
    """
    从 CSV 读取机械臂坐标
    假设 CSV 格式: index, x, y, z
    返回: (N, 2) 的 numpy 数组 (只取 x, y)
    """
    if not os.path.exists(csv_path):
        print(f"错误: 找不到机械臂坐标文件 {csv_path}")
        return None
        
    pts = []
    with open(csv_path, "r") as f:
        for line in f:
            parts = line.strip().split(',')
            if len(parts) >= 3:
                # 假设格式: index, x, y, z
                # 我们只需要 x, y 用于仿射变换 (假设 z 是平面的或我们只做 2D 映射)
                try:
                    x = float(parts[1])
                    y = float(parts[2])
                    pts.append([x, y])
                except ValueError:
                    continue
    return np.array(pts, dtype=np.float32)

def main():
    base_dir = os.path.dirname(__file__)
    
    # 1. 计算左臂矩阵
    print("\n========== 开始计算 左臂 (Left Arm) 标定 ==========")
    robot_csv_left = os.path.join(base_dir, "handeye_calibration", "points_3d_left.csv")
    if os.path.exists(robot_csv_left):
        print("检测左臂棋盘格...")
        img_pts = get_image_points(pattern_size=(3,3))
        if img_pts is not None:
            rob_pts = load_robot_points(robot_csv_left)
            if rob_pts is not None and len(img_pts) == len(rob_pts):
                M_left, _ = cv2.estimateAffine2D(img_pts, rob_pts)
                print(f"左臂矩阵:\n{M_left}")
            else:
                print("左臂点数不匹配或文件错误")
                M_left = None
        else:
            print("未检测到左臂棋盘格")
            M_left = None
    else:
        print(f"未找到左臂数据: {robot_csv_left}")
        M_left = None

    # 2. 计算右臂矩阵
    print("\n========== 开始计算 右臂 (Right Arm) 标定 ==========")
    print("请更换棋盘格到右臂，并按任意键继续（在OpenCV窗口按q取消）...")
    # 这里简单起见，直接调用检测函数，它会等待
    
    robot_csv_right = os.path.join(base_dir, "handeye_calibration", "points_3d_right.csv")
    if os.path.exists(robot_csv_right):
        print("检测右臂棋盘格...")
        img_pts = get_image_points(pattern_size=(3,3))
        if img_pts is not None:
            rob_pts = load_robot_points(robot_csv_right)
            if rob_pts is not None and len(img_pts) == len(rob_pts):
                M_right, _ = cv2.estimateAffine2D(img_pts, rob_pts)
                print(f"右臂矩阵:\n{M_right}")
            else:
                print("右臂点数不匹配或文件错误")
                M_right = None
        else:
            print("未检测到右臂棋盘格")
            M_right = None
    else:
        print(f"未找到右臂数据: {robot_csv_right}")
        M_right = None

    # 3. 保存双臂结果
    save_path = os.path.join(base_dir, "affine_matrix_dual.npz")
    # 构造保存字典，缺失的用 None 或 占位
    save_dict = {}
    if M_left is not None: save_dict['affine_left'] = M_left
    if M_right is not None: save_dict['affine_right'] = M_right
    
    if save_dict:
        np.savez(save_path, **save_dict)
        print(f"\n双臂矩阵已保存至: {save_path}")
    else:
        print("\n未生成任何有效矩阵，跳过保存。")

if __name__ == "__main__":
    main()
