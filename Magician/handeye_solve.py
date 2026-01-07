import os
import cv2
import numpy as np

images_dir = os.path.join(os.path.dirname(__file__), "calibration_data", "images")
poses_file = os.path.join(os.path.dirname(__file__), "calibration_data", "poses.txt")
# 优先加载相机内参文件，如不存在则使用默认值
cam_npz = os.path.join(os.path.dirname(__file__), "camera_params.npz")
if os.path.exists(cam_npz):
    cam = np.load(cam_npz)
    mtx = cam["mtx"].astype(np.float64)
    dist = cam["dist"].astype(np.float64)
else:
    mtx = np.array([[174.29661108, 0.0, 204.15192457],
                    [0.0, 178.81251935, 76.24016034],
                    [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.array([-0.01690069, 0.00018597, 0.00442455, 0.00747917, 0.00010005], dtype=np.float64)
pattern_size = (3,3)
square = 25.0
objp = np.zeros((pattern_size[0]*pattern_size[1],3), np.float32)
objp[:,:2] = np.mgrid[0:pattern_size[0],0:pattern_size[1]].T.reshape(-1,2)*square
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

name_to_pose = {}
with open(poses_file, "r") as f:
    lines = [l.strip() for l in f.readlines()[1:]]
    for l in lines:
        parts = l.split(",")
        name_to_pose[parts[0]] = np.array([float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4])], dtype=np.float64)

R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
used_images = []
for fn in sorted(os.listdir(images_dir)):
    if not fn.lower().endswith((".jpg",".png",".jpeg")): continue
    p = os.path.join(images_dir, fn)
    if fn not in name_to_pose: continue
    im = cv2.imread(p, cv2.IMREAD_GRAYSCALE)
    if im is None: continue
    ret, corners = cv2.findChessboardCorners(im, pattern_size)
    if not ret: continue
    corners = cv2.cornerSubPix(im, corners, (11,11), (-1,-1), criteria)
    ok, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok: continue
    R,_ = cv2.Rodrigues(rvec)
    R_t2c.append(R)
    t_t2c.append(tvec.reshape(3,1))
    x,y,z,r = name_to_pose[fn]
    rz = np.deg2rad(r)
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0],[np.sin(rz), np.cos(rz), 0],[0,0,1]], dtype=np.float64)
    R_g2b.append(Rz)
    t_g2b.append(np.array([[x],[y],[z]], dtype=np.float64))
    used_images.append(fn)

if len(R_t2c) < 3:
    print("数据不足")
    raise SystemExit(1)

methods = [
    cv2.CALIB_HAND_EYE_TSAI,
    cv2.CALIB_HAND_EYE_PARK,
    cv2.CALIB_HAND_EYE_HORAUD,
    cv2.CALIB_HAND_EYE_DANIILIDIS,
]

def to_T(R,t):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = t.reshape(3)
    return T

def yaw_from_R(R):
    return np.arctan2(R[1,0], R[0,0])

def mean_angle(angles):
    s = np.sum(np.sin(angles))
    c = np.sum(np.cos(angles))
    return np.arctan2(s, c)

def Rz(theta):
    return np.array([[np.cos(theta), -np.sin(theta), 0],
                     [np.sin(theta),  np.cos(theta), 0],
                     [0,              0,             1]], dtype=np.float64)

best = None
for m in methods:
    Rc2g, tc2g = cv2.calibrateHandEye(R_g2b, t_g2b, R_t2c, t_t2c, method=m)
    Tc2g = to_T(Rc2g, tc2g)
    Tc2b_list = []
    for i in range(len(R_g2b)):
        Tg2b = to_T(R_g2b[i], t_g2b[i])
        Tc2b_list.append(Tg2b@Tc2g)
    yaws = np.array([yaw_from_R(T[:3,:3]) for T in Tc2b_list], dtype=np.float64)
    yaw_mean = mean_angle(yaws)
    Rc2b_mean = Rz(yaw_mean)
    tc2b_mean = np.mean([T[:3,3] for T in Tc2b_list], axis=0).reshape(3,1)
    diffs = []
    for T in Tc2b_list:
        yaw_i = yaw_from_R(T[:3,:3])
        dyaw = np.rad2deg(np.arctan2(np.sin(yaw_i - yaw_mean), np.cos(yaw_i - yaw_mean)))
        angle = abs(dyaw)
        dtrans = np.linalg.norm(T[:3,3]-tc2b_mean.reshape(3))
        diffs.append((angle, dtrans))
    ang_err = float(np.mean([d[0] for d in diffs]))
    trans_err = float(np.mean([d[1] for d in diffs]))
    print("方法:", m)
    print("Camera->Base R:")
    print(Rc2b_mean)
    print("Camera->Base t(mm):", tc2b_mean.reshape(3))
    print("稳定性误差(仅Z轴旋转) 角度均值(度):", ang_err, " 平移均值(mm):", trans_err)
    score = ang_err + trans_err*0.1
    if best is None or score < best[0]:
        best = (score, Rc2b_mean, tc2b_mean, ang_err, trans_err, m)

out_path = os.path.join(os.path.dirname(__file__), "handeye_result.npz")
np.savez(out_path, Rc2b=best[1], tc2b=best[2])
print("已保存: ", out_path, " 方法: ", best[5], " 角度均值:", best[3], " 平移均值:", best[4])
