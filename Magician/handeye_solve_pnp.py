import os
import cv2
import numpy as np

def Rz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta),  np.cos(theta), 0],
                     [0,              0,             1]], dtype=np.float64)

def yaw_from_R(R):
    return np.arctan2(R[1,0], R[0,0])

def main():
    base_dir = os.path.dirname(__file__)
    images_dir = os.path.join(base_dir, "calibration_data", "images")
    points3d_csv = os.path.join(base_dir, "calibration_data", "points_3d.csv")
    cam_npz = os.path.join(base_dir, "camera_params.npz")

    if not os.path.exists(points3d_csv):
        raise SystemExit("缺少3D点文件 calibration_data/points_3d.csv （每行: idx,x,y,z）")

    if not os.path.exists(cam_npz):
        raise SystemExit("缺少相机内参文件 camera_params.npz")
    cam = np.load(cam_npz)
    mtx = cam["mtx"].astype(np.float64)
    dist = cam["dist"].astype(np.float64)

    pattern_size = (3,3)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    obj_pts = []
    with open(points3d_csv, "r") as f:
        lines = [l.strip() for l in f if l.strip()]
        for l in lines:
            parts = l.split(",")
            if len(parts) < 4: continue
            obj_pts.append([float(parts[1]), float(parts[2]), float(parts[3])])
    obj_pts = np.array(obj_pts, dtype=np.float64)
    obj_pts[:,1] = -obj_pts[:,1]

    img = None
    for fn in sorted(os.listdir(images_dir)):
        if fn.lower().endswith((".jpg",".png",".jpeg")):
            p = os.path.join(images_dir, fn)
            img = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                break
    if img is None:
        raise SystemExit("未找到用于检测角点的图像")

    ok, corners = cv2.findChessboardCorners(img, pattern_size)
    if not ok:
        raise SystemExit("角点检测失败")
    corners = cv2.cornerSubPix(img, corners, (11,11), (-1,-1), criteria)
    img_pts = corners.reshape(-1,2).astype(np.float64)
    # 归一化去畸变像素坐标（相机坐标系单位化平面）
    undist = cv2.undistortPoints(corners, mtx, dist)
    img_pts_norm = undist.reshape(-1,2).astype(np.float64)
    if len(obj_pts) != len(img_pts):
        raise SystemExit("3D点与2D点数量不一致")

    flag = cv2.SOLVEPNP_IPPE_SQUARE if obj_pts.shape[0] == 4 else cv2.SOLVEPNP_IPPE
    # 优先使用平面 IPPE（归一化坐标 + 单位相机矩阵）
    I = np.eye(3, dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts_norm, I, None, flags=flag)
    if not ok:
        # 回退到通用迭代（像素坐标 + 畸变参数）
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, mtx, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise SystemExit("PnP求解失败")
    R,_ = cv2.Rodrigues(rvec)
    yaw = yaw_from_R(R)
    Rc2b = Rz(yaw)
    tc2b = tvec.reshape(3,1)

    print("Camera->Base R(仅Z轴旋转):")
    print(Rc2b)
    print("Camera->Base t(mm):", tc2b.reshape(3))

    # 使用原始内参与畸变在像素域评估重投影误差
    proj, _ = cv2.projectPoints(obj_pts, cv2.Rodrigues(Rc2b)[0], tc2b.reshape(3), mtx, dist)
    proj = proj.reshape(-1,2)
    rmse = np.sqrt(np.mean(np.sum((proj - img_pts)**2, axis=1)))
    print("重投影RMSE(像素):", float(rmse))

    out_path = os.path.join(base_dir, "handeye_result.npz")
    np.savez(out_path, Rc2b=Rc2b, tc2b=tc2b)
    print("已保存: ", out_path)

if __name__ == "__main__":
    main()
