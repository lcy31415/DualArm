import os
import json
import cv2
import numpy as np
import DobotDllType as dType
from config import CAMERA_PARAMS_PATH, HANDEYE_RESULT_PATH, Z_WORKPLANE, APPROACH_Z, PICK_Z, PLACE_Z, FLIP_Y, STOP_POSITION, SAFE_POSITION, PLACE_POSITIONS, COORDINATES_FILE, USE_GRIPPER, USE_SUCTION

def wait_idx(api, idx):
    while idx > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)

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

def image_to_base(cx, cy, mtx, dist, Rc2b, tc2b):
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
    t = (Z_WORKPLANE - oz)/dz
    pb = org_b + t*dir_b
    x, y = float(pb[0,0]), float(pb[1,0])
    if FLIP_Y:
        y = -y
    return (x, y, Z_WORKPLANE)
def open_camera(api):
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("摄像头打开失败")
        dType.DisconnectDobot(api)
        return None
    print("摄像头预览启动")  
    return cap if cap.isOpened() else None

def detect_red_center(frame):
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    m1 = cv2.inRange(hsv, np.array([0,120,70], np.uint8), np.array([10,255,255], np.uint8))
    m2 = cv2.inRange(hsv, np.array([170,120,70], np.uint8), np.array([180,255,255], np.uint8))
    mask = cv2.bitwise_or(m1, m2)
    kernel = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    cnts = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
    if not cnts:
        return None
    c = max(cnts, key=cv2.contourArea)
    M = cv2.moments(c)
    if M["m00"] <= 0:
        r = cv2.minAreaRect(c)
        return (int(r[0][0]), int(r[0][1]))
    return (int(M["m10"]/M["m00"]), int(M["m01"]/M["m00"]))

def find_board_pose(frame, mtx, dist):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    pattern_size = (3,3)
    square = 25.0
    objp = np.zeros((pattern_size[0]*pattern_size[1],3), np.float32)
    grid = np.mgrid[0:pattern_size[0],0:pattern_size[1]].T.reshape(-1,2)
    objp[:,:2] = grid * square
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    ok, corners = cv2.findChessboardCorners(gray, pattern_size)
    if not ok:
        return None
    corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
    I = np.eye(3, dtype=np.float64)
    und = cv2.undistortPoints(corners, mtx, dist)
    flag = cv2.SOLVEPNP_IPPE if objp.shape[0] >= 4 else cv2.SOLVEPNP_ITERATIVE
    ok2, rvec, tvec = cv2.solvePnP(objp, und.reshape(-1,2), I, None, flags=flag)
    if not ok2:
        ok2, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok2:
        return None
    Rb2c,_ = cv2.Rodrigues(rvec)
    tb2c = tvec.reshape(3,1)
    return (Rb2c, tb2c)

def main():
    api = dType.load()
    state = dType.ConnectDobot(api, "", 115200)[0]
    if state != dType.DobotConnect.DobotConnect_NoError:
        return
    print("机械臂连接成功")
    dType.SetQueuedCmdClear(api)
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100, isQueued=1)
    dType.SetQueuedCmdStartExec(api)
    idx = dType.SetHOMECmd(api, 0, isQueued=1)[0]
    wait_idx(api, idx)
    print("回零完成")
    idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, STOP_POSITION["x"], STOP_POSITION["y"], STOP_POSITION["z"], STOP_POSITION["r"], isQueued=1)[0]
    wait_idx(api, idx)
    print(f"已移动到停止位: ({STOP_POSITION['x']:.2f},{STOP_POSITION['y']:.2f},{STOP_POSITION['z']:.2f},{STOP_POSITION['r']:.2f})")
    cam = np.load(CAMERA_PARAMS_PATH)
    mtx = cam["mtx"].astype(np.float64)
    dist = cam["dist"].astype(np.float64)
    he = np.load(HANDEYE_RESULT_PATH)
    Rc2b = he["Rc2b"].astype(np.float64)
    tc2b = he["tc2b"].astype(np.float64)
    print("已加载相机内参与手眼标定")
    coord_path = COORDINATES_FILE
    if not os.path.exists(coord_path):
        print(f"坐标文件不存在: {coord_path}")
        dType.SetQueuedCmdStopExec(api)
        dType.DisconnectDobot(api)
        return
    with open(coord_path, "r", encoding="utf-8") as f:
        data = json.load(f)
    blocks = data.get("blocks", [])
    print(f"检测到 {len(blocks)} 个小物块，开始分类码垛")
    order = {"red": 0, "green": 1, "blue": 2}
    blocks.sort(key=lambda b: order.get(b.get("color",""), 99))
    for b in blocks:
        cx = b.get("x", 0.0)
        cy = b.get("y", 0.0)
        color = b.get("color", "")
        print(f"处理 {color} 物块，像素中心=({cx:.1f},{cy:.1f})")
        base = pixel_to_base_xy(cx, cy, mtx, dist, Rc2b, tc2b, cz=APPROACH_Z, flip_y=FLIP_Y)
        if base is None:
            print("坐标转换失败，跳过该物块")
            continue
        x, y, z = base
        print(f"转换为基座坐标=({x:.2f},{y:.2f},{z:.2f})，靠近高度={APPROACH_Z:.2f}")
        idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, x, y, APPROACH_Z, 0.0, isQueued=1)[0]
        wait_idx(api, idx)
        print("到达上方，下降抓取")
        idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, x, y, PICK_Z, 0.0, isQueued=1)[0]
        wait_idx(api, idx)
        dType.dSleep(1000)
        print("执行抓取")
        if USE_GRIPPER:
            idx = dType.SetEndEffectorGripper(api, 1, 1, isQueued=1)[0]
        elif USE_SUCTION:
            idx = dType.SetEndEffectorSuctionCup(api, 1, 1, isQueued=1)[0]
        else:
            idx = dType.SetEndEffectorGripper(api, 1, 1, isQueued=1)[0]
        wait_idx(api, idx)
        dType.dSleep(1000)
        print("抬起")
        idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, x, y, APPROACH_Z, 0.0, isQueued=1)[0]
        wait_idx(api, idx)
        if color in PLACE_POSITIONS:
            px = PLACE_POSITIONS[color]["x"]
            py = PLACE_POSITIONS[color]["y"]
            pr = PLACE_POSITIONS[color].get("r", 0.0)
            print(f"前往投放区 {color}: ({px:.2f},{py:.2f}), r={pr:.2f}")
            idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, px, py, APPROACH_Z, pr, isQueued=1)[0]
            wait_idx(api, idx)
            print("下降投放")
            idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, px, py, PLACE_Z, pr, isQueued=1)[0]
            wait_idx(api, idx)
            dType.dSleep(1000)
            print("释放物块")
            if USE_GRIPPER:
                idx = dType.SetEndEffectorGripper(api, 1, 0, isQueued=1)[0]
            elif USE_SUCTION:
                idx = dType.SetEndEffectorSuctionCup(api, 1, 0, isQueued=1)[0]
            else:
                idx = dType.SetEndEffectorGripper(api, 1, 0, isQueued=1)[0]
            wait_idx(api, idx)
            dType.dSleep(1000)
            print("抬起离开")
            idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, px, py, APPROACH_Z, pr, isQueued=1)[0]
            wait_idx(api, idx)
        else:
            print(f"未配置 {color} 的投放位置，跳过投放")
    idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, SAFE_POSITION["x"], SAFE_POSITION["y"], SAFE_POSITION["z"], SAFE_POSITION["r"], isQueued=1)[0]
    wait_idx(api, idx)
    print(f"已回到安全位置: ({SAFE_POSITION['x']:.2f},{SAFE_POSITION['y']:.2f},{SAFE_POSITION['z']:.2f},{SAFE_POSITION['r']:.2f})")
    dType.SetQueuedCmdStopExec(api)
    dType.DisconnectDobot(api)
    print("流程结束，已断开连接")

if __name__ == "__main__":
    main()
