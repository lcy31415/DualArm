# 手眼标定使用指南

## 概览
- 目标：建立相机坐标系到机械臂基座坐标系的几何映射（Camera→Base），使视觉检测到的目标能转换为机械臂可到达的基座坐标进行运动。
- 输出文件：
  - 相机内参与畸变：E:\DualArm\camera_params.npz（键 mtx、dist）
  - 手眼标定外参：E:\DualArm\handeye_result.npz（键 Rc2b、tc2b）
- 目录结构（重命名后）：
  - 图片来源：E:\DualArm\Hand-Eye_Calibration\camera_calibration\*.png
  - 标定数据：E:\DualArm\Hand-Eye_Calibration\handeye_calibration\

## 前置条件
- 棋盘参数：
  - CHESSBOARD_SIZE = (3,3)：棋盘内角点维度（3x3 个内角点，源自 4x4 方格）
  - SQUARE_SIZE = 25 mm：单格边长，确保与实际打印尺寸一致
- 机械臂和相机状态：
  - 相机已连接且可读取
  - 机械臂已连接且可执行队列运动

## 步骤一：相机内参标定
1. 将棋盘图片放到 E:\DualArm\Hand-Eye_Calibration\camera_calibration\ 下
2. 运行 [camera_intrinsics.py](file:///e:/DualArm/Hand-Eye_Calibration/camera_intrinsics.py)
3. 成功后生成 E:\DualArm\camera_params.npz

关键参数说明：
- mtx：3x3 内参矩阵（焦距 fx, fy 与主点 cx, cy）
- dist：畸变系数（k1, k2, p1, p2, k3...）
- ret：平均重投影误差（越小越好，建议 < 0.5）

## 步骤二：采集棋盘角点对应的机械臂位姿
1. 运行 [collect_points_3d.py](file:///e:/DualArm/Hand-Eye_Calibration/collect_points_3d.py)
2. 在预览界面按键：
   - r：记录当前角点的机械臂位姿（mm）
   - n/b：切换到下一个/上一个角点索引
   - s：保存数据 points_3d.csv 并退出
   - q：退出
3. 数据保存位置：E:\DualArm\Hand-Eye_Calibration\handeye_calibration\points_3d.csv

文件内容格式：
- 每行：idx,x,y,z
- 单位：mm，坐标系为机械臂基座系

## 步骤三：基于 PnP 的手眼标定
1. 将一张棋盘图像保存为 E:\DualArm\Hand-Eye_Calibration\handeye_calibration\images\image_0.png
2. 运行 [handeye_solve_pnp.py](file:///e:/DualArm/Hand-Eye_Calibration/handeye_solve_pnp.py)
3. 输出 E:\DualArm\handeye_result.npz

流程要点：
- 读取 points_3d.csv 后对 Y 坐标取反：修正左右颠倒（y→-y）
- 优先使用平面 IPPE：对去畸变归一化后的像素坐标与单位相机矩阵 I 求解
- 失败回退至 ITERATIVE：使用原始像素坐标与内参畸变参数
- 仅取 Z 轴旋转：Rc2b = Rz(yaw)，tc2b 为平移向量

输出含义：
- Rc2b：Camera→Base 的旋转矩阵（仅 Z 轴旋转假设）
- tc2b：Camera→Base 的平移向量（mm）
- RMSE：基于像素域的重投影误差，建议 < 1–2 像素

## 步骤四：运动验证
1. 运行 [handeye_verify.py](file:///e:/DualArm/Hand-Eye_Calibration/handeye_verify.py)
2. 程序检测棋盘，逐点计算角点的基座坐标并移动到 z=10
3. 终端输出每点的基座坐标与队列索引，并显示预览

实用选项：
- flip_y：默认 True，用于运行阶段修正左右颠倒（将 y 取反再下发）
- approach_z：默认 10 mm，验证时的目标高度

## 关键参数与变量解释
- CHESSBOARD_SIZE / pattern_size：棋盘内角点网格维度，例如 (3,3)
- SQUARE_SIZE / square：棋盘单格边长，直接影响空间尺度
- criteria：角点亚像素优化的停止准则（最多迭代 30 次 / EPS）
- mtx, dist：相机内参与畸变系数，存储在 camera_params.npz
- Rc2b, tc2b：Camera→Base 外参（旋转与平移），存储在 handeye_result.npz
- flip_y：左右颠倒修正开关，将 y 取反以匹配机械臂基座系方向
- approach_z：验证运动的 Z 高度

## 常见问题与排查
- 棋盘检测失败：
  - 增加光照、降低反光，确保棋盘完整可见
  - 检查 CHESSBOARD_SIZE 是否匹配实际打印棋盘
- 运动方向左右颠倒：
  - 确认在 PnP 阶段已有 y 取反
  - 验证阶段可开启 flip_y 进一步修正
- 重投影误差过大：
  - 重新拍摄更清晰的棋盘图像
  - 增加采集的有效角点数据并保证机械臂位姿准确记录

## 参考脚本
- 相机标定：[camera_intrinsics.py](file:///e:/DualArm/Hand-Eye_Calibration/camera_intrinsics.py)
- 采集 3D 点：[collect_points_3d.py](file:///e:/DualArm/Hand-Eye_Calibration/collect_points_3d.py)
- PnP 标定：[handeye_solve_pnp.py](file:///e:/DualArm/Hand-Eye_Calibration/handeye_solve_pnp.py)
- 验证运动：[handeye_verify.py](file:///e:/DualArm/Hand-Eye_Calibration/handeye_verify.py)

