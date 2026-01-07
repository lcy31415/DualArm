import os
import cv2
import numpy as np
import DobotDllType as dType

def to_T(R,t):
    T = np.eye(4)
    T[:3,:3] = R
    T[:3,3] = t.reshape(3)
    return T

def wait_idx(api, last_index):
    while last_index > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)

def acquire_Tb2c(cap, pattern_size, objp, mtx, dist, criteria):
    for _ in range(100):
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(gray, pattern_size)
        if ok:
            corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            I = np.eye(3, dtype=np.float64)
            und = cv2.undistortPoints(corners, mtx, dist)
            flag = cv2.SOLVEPNP_IPPE if objp.shape[0] >= 4 else cv2.SOLVEPNP_ITERATIVE
            ok2, rvec, tvec = cv2.solvePnP(objp, und.reshape(-1,2), I, None, flags=flag)
            if not ok2:
                ok2, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist, flags=cv2.SOLVEPNP_ITERATIVE)
            if ok2:
                Rb2c,_ = cv2.Rodrigues(rvec)
                tb2c = tvec.reshape(3,1)
                cv2.drawChessboardCorners(frame, pattern_size, corners, ok)
                cv2.imshow("Verify Preview", frame)
                cv2.waitKey(10)
                return to_T(Rb2c, tb2c)
        cv2.imshow("Verify Preview", frame)
        cv2.waitKey(10)
    return None

def main():
    api = dType.load()
    state = dType.ConnectDobot(api, "", 115200)[0]
    if state != dType.DobotConnect.DobotConnect_NoError:
        print("连接机械臂失败")
        return
    print("机械臂连接成功")
    dType.SetQueuedCmdClear(api)
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100, isQueued=1)
    dType.SetQueuedCmdStartExec(api)

    base_x, base_y, base_z, base_r = 258.5760, 122.2609, 64.2540, 0.0
    idx = dType.SetHOMECmd(api, 0, isQueued=1)[0]
    wait_idx(api, idx)
    idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, base_x, base_y, base_z, base_r, isQueued=1)[0]
    wait_idx(api, idx)
    print(f"已移动到基准位姿: ({base_x:.2f}, {base_y:.2f}, {base_z:.2f}, {base_r:.2f})")

    calib_path = os.path.join(os.path.dirname(__file__), "handeye_result.npz")
    cam_params_path = os.path.join(os.path.dirname(__file__), "camera_params.npz")
    if not (os.path.exists(calib_path) and os.path.exists(cam_params_path)):
        print("缺少 handeye_result.npz 或 camera_params.npz")
        dType.DisconnectDobot(api)
        return
    data = np.load(calib_path)
    Rc2b = data["Rc2b"].astype(np.float64)
    tc2b = data["tc2b"].astype(np.float64)
    cam = np.load(cam_params_path)
    mtx = cam["mtx"].astype(np.float64)
    dist = cam["dist"].astype(np.float64)

    pattern_size = (3,3)
    square = 25.0
    objp = np.zeros((pattern_size[0]*pattern_size[1],3), np.float32)
    grid = np.mgrid[0:pattern_size[0],0:pattern_size[1]].T.reshape(-1,2)
    objp[:,:2] = grid * square
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("摄像头打开失败")
        dType.DisconnectDobot(api)
        return
    print("摄像头预览启动")

    Tb2c = acquire_Tb2c(cap, pattern_size, objp, mtx, dist, criteria)
    if Tb2c is None:
        print("棋盘检测失败，无法获取位姿")
        cap.release()
        cv2.destroyAllWindows()
        dType.SetQueuedCmdStopExec(api)
        dType.DisconnectDobot(api)
        return

    Rb2c = Tb2c[:3,:3]
    tb2c = Tb2c[:3,3].reshape(3,1)
    approach_z = 10.0
    targets = objp.reshape(-1,3)
    for k, c in enumerate(targets):
        pc = Rb2c @ c.reshape(3,1) + tb2c
        pb = Rc2b @ pc + tc2b.reshape(3,1)
        x, y = float(pb[0]), float(pb[1])
        print(f"目标{k}: 棋盘点({c[0]:.1f},{c[1]:.1f}) → 基座({x:.2f},{y:.2f},{approach_z:.2f})")
        idx1 = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, x, y, approach_z, 0.0, isQueued=1)[0]
        print(f"下发移动指令，队列索引: {idx1}")
        wait_idx(api, idx1)
        print("移动完成")

    cap.release()
    cv2.destroyAllWindows()
    dType.SetQueuedCmdStopExec(api)
    dType.DisconnectDobot(api)

if __name__ == "__main__":
    main()

