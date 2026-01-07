import os
import cv2
import numpy as np
import DobotDllType as dType

pattern_size = (3,3)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def ensure_dir(p):
    if not os.path.exists(p):
        os.makedirs(p)

def main():
    base_dir = os.path.dirname(__file__)
    out_dir = os.path.join(base_dir, "calibration_data")
    ensure_dir(out_dir)
    out_csv = os.path.join(out_dir, "points_3d.csv")

    api = dType.load()
    state = dType.ConnectDobot(api, "", 115200)[0]
    if state != dType.DobotConnect.DobotConnect_NoError:
        return
    dType.SetQueuedCmdClear(api)
    dType.SetQueuedCmdStartExec(api)

    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        dType.DisconnectDobot(api)
        return

    points = [None]* (pattern_size[0]*pattern_size[1])
    idx = 0

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ok, corners = cv2.findChessboardCorners(gray, pattern_size)
        if ok:
            corners = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
            for k,c in enumerate(corners.reshape(-1,2)):
                color = (0,255,0) if k!=idx else (0,0,255)
                cv2.circle(frame, (int(c[0]), int(c[1])), 6, color, -1)
                cv2.putText(frame, str(k), (int(c[0])+5, int(c[1])-5), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.putText(frame, f"Corner {idx}/{len(points)-1}", (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255,255,255), 2)
        cv2.putText(frame, "r: record pose for current corner", (10,60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        cv2.putText(frame, "n/b: next/back corner | s: save | q: quit", (10,85), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
        for k,p in enumerate(points):
            if p is not None:
                cv2.putText(frame, f"{k}:{p[0]:.1f},{p[1]:.1f},{p[2]:.1f}", (10,120+20*k), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
        cv2.imshow("Collect 3D Points", frame)
        key = cv2.waitKey(10) & 0xFF
        if key == ord('q'):
            break
        if key == ord('n'):
            idx = min(idx+1, len(points)-1)
        if key == ord('b'):
            idx = max(idx-1, 0)
        if key == ord('r'):
            pose = dType.GetPose(api)
            points[idx] = (float(pose[0]), float(pose[1]), float(pose[2]))
            idx = min(idx+1, len(points)-1)
        if key == ord('s'):
            if any(p is None for p in points):
                pass
            else:
                with open(out_csv, "w") as f:
                    for i,p in enumerate(points):
                        f.write(f"{i},{p[0]},{p[1]},{p[2]}\n")
                break

    cap.release()
    cv2.destroyAllWindows()
    dType.SetQueuedCmdStopExec(api)
    dType.DisconnectDobot(api)

if __name__ == "__main__":
    main()
