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

def main():
    api = dType.load()
    state = dType.ConnectDobot(api, "", 115200)[0]
    if state != dType.DobotConnect.DobotConnect_NoError:
        return
    dType.SetQueuedCmdClear(api)
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100, isQueued=1)
    dType.SetQueuedCmdStartExec(api)

    base_x, base_y, base_z, base_r = 258.5760, 122.2609, 64.2540, 0.0
    idx = dType.SetHOMECmd(api, 0, isQueued=1)[0]
    wait_idx(api, idx)
    idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, base_x, base_y, base_z, base_r, isQueued=1)[0]
    wait_idx(api, idx)

    calib_path = os.path.join(os.path.dirname(__file__), "handeye_result.npz")
    cam_params_path = os.path.join(os.path.dirname(__file__), "camera_params.npz")
    if not (os.path.exists(calib_path) and os.path.exists(cam_params_path)):
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
        dType.DisconnectDobot(api)
        return

    def acquire_Tb2c():
        for _ in range(100):
            ret, frame = cap.read()
            if not ret:
                continue
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ok, corners = cv2.findChessboardCorners(gray, pattern_size)
            if ok:
                corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
                ok2, rvec, tvec = cv2.solvePnP(objp, corners, mtx, dist, flags=cv2.SOLVEPNP_ITERATIVE)
                if ok2:
                    Rb2c,_ = cv2.Rodrigues(rvec)
                    tb2c = tvec.reshape(3,1)
                    return to_T(Rb2c, tb2c)
            cv2.imshow("Validate Preview", frame)
            cv2.waitKey(10)
        return None

    approach_z = 10.0
    for j in range(4):
        cols = [i for i in range(4) if (i + j) % 2 == 0]
        for i in cols:
            Tb2c = acquire_Tb2c()
            if Tb2c is None:
                continue
            c = np.array([(i - 0.5) * square, (j - 0.5) * square, 0.0], dtype=np.float64).reshape(3,1)
            pc = Tb2c[:3,:3] @ c + Tb2c[:3,3].reshape(3,1)
            pb = Rc2b @ pc + tc2b.reshape(3,1)
            x, y = float(pb[0]), float(pb[1])
            idx1 = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, x, y, approach_z, 0.0, isQueued=1)[0]
            wait_idx(api, idx1)
            idx2 = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, base_x, base_y, base_z, base_r, isQueued=1)[0]
            wait_idx(api, idx2)

    cap.release()
    cv2.destroyAllWindows()
    dType.SetQueuedCmdStopExec(api)
    dType.DisconnectDobot(api)

if __name__ == "__main__":
    main()
