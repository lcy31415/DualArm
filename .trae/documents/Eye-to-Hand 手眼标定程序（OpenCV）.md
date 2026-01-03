## 目标
- 使用 Python + OpenCV 完成 Eye-to-Hand（相机固定在基座上方）手眼标定，解算 `T_cam_base`（相机坐标系相对于机械臂基座坐标系的变换）。
- 采用 OpenCV `cv2.calibrateHandEye`（Tsai/Park/Horaud 等方法）求解公式 `A X = Z B`，其中：
  - `A`: 每次抓拍时的 `T_gripper_base`
  - `B`: 每次抓拍时的 `T_target_cam`（棋盘格相对于相机）
  - `X`: 常量 `T_cam_base`（目标结果）
  - `Z`: 常量 `T_target_gripper`（不需显式求出）

## 输入/输出约定
- 输入：
  - `cameraMatrix`, `distCoeffs`（已标定，来源于 npz 或硬编码）
  - `images`（多姿态棋盘格图像）
  - `R_gripper2base[i]`, `t_gripper2base[i]`（机械臂末端相对于基座的位姿序列）
- 输出：
  - `R_cam2base`, `t_cam2base` 与 4x4 齐次矩阵 `T_cam_base`
  - 误差报告：配对运动闭环残差（旋转角度/平移量的 RMS）

## 主要步骤
### 1. 角点提取与亚像素精化
- 对每张图：
  - 用 `cv2.findChessboardCorners(image, (3, 3), flags)` 找内角点（3x3）
  - 用 `cv2.cornerSubPix` 做亚像素精化
- 注意：棋盘格规格为 4x4 方格 → 内角点为 3x3；按 OpenCV 默认坐标系建立角点顺序（行优先）。

### 2. SolvePnP 姿态估计（棋盘格相对于相机）
- 构造 `objp`：尺寸为 `(3*3, 3)` 的 3D 点阵，单位与臂位姿一致（建议统一用米，若输入为毫米则换算）。
- `cv2.solvePnP(objp, corners, cameraMatrix, distCoeffs, flags=cv2.SOLVEPNP_ITERATIVE)`
- `cv2.Rodrigues(rvec)` 得 `R_target2cam`，平移为 `t_target2cam`

### 3. 手眼标定（calibrateHandEye）
- 组织序列：`R_gripper2base`, `t_gripper2base`, `R_target2cam`, `t_target2cam`
- 尝试多种方法：`Tsai`, `Park`, `Horaud`，对比稳定性
- `R_cam2base, t_cam2base = cv2.calibrateHandEye(...)`
- 组合为 4x4 齐次矩阵 `T_cam_base`

### 4. 精度验证（闭环配对运动残差）
- 对任意样本对 `(i, j)`：
  - 计算相对运动：`A_ij = T_gripper_base[j] * inv(T_gripper_base[i])`
  - 计算相对运动：`B_ij = T_target_cam[j] * inv(T_target_cam[i])`
  - 理论闭环：`A_ij * X ≈ X * B_ij`
  - 评估旋转残差角度（轴角范数）与平移残差（范数），统计 RMS
- 额外：报告 `solvePnP` 重投影 RMS（确保每张图的角点/内参一致性）

## 代码骨架（Python）
```python
import cv2
import numpy as np
from pathlib import Path

# ===== 输入：相机内参（示例用你的标定结果） =====
cameraMatrix = np.array([[174.29661108, 0., 204.15192457],
                         [0., 178.81251935, 76.24016034],
                         [0., 0., 1.]], dtype=np.float64)
distCoeffs = np.array([-0.01690069, 0.00018597, 0.00442455, 0.00747917, 0.00010005], dtype=np.float64)

PATTERN_SIZE = (3, 3)
SQUARE_SIZE = 0.025  # 25mm → 0.025m

def make_object_points(pattern_size, square_size):
    nx, ny = pattern_size
    objp = np.zeros((nx*ny, 3), dtype=np.float32)
    grid = np.mgrid[0:nx, 0:ny].T.reshape(-1, 2)
    objp[:, :2] = grid * square_size
    return objp

def find_corners(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    ok, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE, flags)
    if not ok:
        return None
    term = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)
    corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), term)
    return corners

def estimate_target2cam(objp, corners, cameraMatrix, distCoeffs):
    ok, rvec, tvec = cv2.solvePnP(objp, corners, cameraMatrix, distCoeffs, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return None
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)
    return R, t

def to_homogeneous(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = t
    return T

def calibrate_handeye(R_gripper2base_list, t_gripper2base_list, R_target2cam_list, t_target2cam_list, method=cv2.CALIB_HAND_EYE_TSAI):
    R_cam2base, t_cam2base = cv2.calibrateHandEye(
        R_gripper2base_list, t_gripper2base_list,
        R_target2cam_list, t_target2cam_list,
        method=method
    )
    return R_cam2base, t_cam2base.reshape(3)

def pairwise_motion(T_list):
    motions = []
    for i in range(len(T_list)):
        for j in range(i+1, len(T_list)):
            Tij = T_list[j] @ np.linalg.inv(T_list[i])
            motions.append(Tij)
    return motions

def rotation_error_deg(R):
    angle = np.arccos(np.clip((np.trace(R) - 1)/2, -1, 1))
    return np.degrees(angle)

def validate_closed_loop(T_gripper_base_list, T_target_cam_list, T_cam_base):
    A_list = pairwise_motion(T_gripper_base_list)
    B_list = pairwise_motion(T_target_cam_list)
    rot_errs, trans_errs = [], []
    for A, B in zip(A_list, B_list):
        left = A @ T_cam_base
        right = T_cam_base @ B
        R_err = left[:3, :3] @ right[:3, :3].T
        t_err = left[:3, 3] - right[:3, 3]
        rot_errs.append(rotation_error_deg(R_err))
        trans_errs.append(np.linalg.norm(t_err))
    return np.mean(rot_errs), np.mean(trans_errs)

def main(images_dir, R_gripper2base, t_gripper2base):
    objp = make_object_points(PATTERN_SIZE, SQUARE_SIZE)
    R_target2cam_list, t_target2cam_list = [], []

    img_paths = sorted(Path(images_dir).glob('*.png'))
    for p in img_paths:
        img = cv2.imread(str(p))
        corners = find_corners(img)
        if corners is None:
            continue
        pose = estimate_target2cam(objp, corners, cameraMatrix, distCoeffs)
        if pose is None:
            continue
        R, t = pose
        R_target2cam_list.append(R)
        t_target2cam_list.append(t)

    T_gripper_base_list = [to_homogeneous(R, t) for R, t in zip(R_gripper2base, t_gripper2base)]
    R_cam2base, t_cam2base = calibrate_handeye(R_gripper2base, t_gripper2base, R_target2cam_list, t_target2cam_list)
    T_cam_base = to_homogeneous(R_cam2base, t_cam2base)

    rot_rms_deg, trans_rms_m = validate_closed_loop(T_gripper_base_list,
                                                    [to_homogeneous(R, t) for R, t in zip(R_target2cam_list, t_target2cam_list)],
                                                    T_cam_base)
    print('R_cam2base:\n', R_cam2base)
    print('t_cam2base:', t_cam2base)
    print('T_cam_base:\n', T_cam_base)
    print('Closed-loop RMS — rot(deg):', rot_rms_deg, 'trans(m):', trans_rms_m)

if __name__ == '__main__':
    # 占位：填入你的机器人数据与图像目录
    R_gripper2base = []  # [np.eye(3), ...]
    t_gripper2base = []  # [np.zeros(3), ...]
    images_dir = 'data/images'  # 修改为你的路径
    main(images_dir, R_gripper2base, t_gripper2base)
```

## 注意事项
- 单位统一：`t_gripper2base` 与棋盘 `SQUARE_SIZE` 要一致（推荐米）。
- 数据量：建议至少 10–15 个不同位姿，覆盖足够旋转/平移。
- 角点顺序：与 `objp` 构造保持一致；如图像有镜像/旋转需确保正确排序。
- 方法选择：若 `Tsai` 不稳定，尝试 `Park`、`Horaud` 并对比闭环残差。
- 验证：闭环残差越小越好（旋转 < 0.5°、平移 < 几毫米为佳，视场景而定）。

## 交付物
- 完整 Python 脚本（含四大模块函数）
- 示例调用与输出（`R_cam2base`, `t_cam2base`, `T_cam_base`）
- 验证报告（闭环残差 + 每幅图的重投影 RMS 可选）

请确认以上方案；确认后我将生成可直接运行的脚本，并根据你的 npz/数据路径做适配。