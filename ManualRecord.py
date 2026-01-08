import threading
import DobotDllType as dType
import time
import cv2
import os
import math
import sys

# ---------------- 配置参数 ----------------
# 数据保存路径
DATA_DIR = "manual_data"
IMAGE_DIR = os.path.join(DATA_DIR, "images")
POSE_FILE = os.path.join(DATA_DIR, "poses.txt")

# 运动检测阈值 (mm) - 超过此距离认为在移动
MOVE_THRESHOLD = 2.0
# 静止判定时间 (秒) - 移动停止后保持静止多久才触发保存
STABLE_DURATION = 1.0

# ---------------- 全局变量 ----------------
api = None
current_pose = [0, 0, 0, 0]
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

def init_dobot():
    global api
    api = dType.load()
    state = dType.ConnectDobot(api, "", 115200)[0]
    print("连接状态:", CON_STR[state])
    if state == dType.DobotConnect.DobotConnect_NoError:
        dType.SetQueuedCmdClear(api)
        return True
    return False

def get_pose_loop():
    """后台线程持续更新位姿"""
    global current_pose
    while True:
        try:
            pose = dType.GetPose(api)
            # pose: [x, y, z, r, j1, j2, j3, j4]
            current_pose = [pose[0], pose[1], pose[2], pose[3]]
            time.sleep(0.05)
        except:
            break

def calculate_distance(p1, p2):
    return math.sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2 + (p1[2]-p2[2])**2)

def save_record(frame, pose, count):
    print(f"\n[坐标 #{count}] X:{pose[0]:.4f} Y:{pose[1]:.4f} Z:{pose[2]:.4f} R:{pose[3]:.4f}")

def main():
    if not init_dobot():
        return

    # 启动位姿更新线程
    t = threading.Thread(target=get_pose_loop)
    t.daemon = True
    t.start()

    # 初始化摄像头
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 状态变量
    last_pose = current_pose[:]
    last_move_time = time.time()
    is_moving = False
    record_count = 0
    
    # 检查文件是否存在，获取当前计数
    if os.path.exists(POSE_FILE):
        with open(POSE_FILE, "r") as f:
            lines = f.readlines()
            record_count = len(lines) - 1 if len(lines) > 0 else 0

    print("\n---------------------------------------------------")
    print("  Dobot 手动示教记录程序")
    print("  操作说明：")
    print("  1. 按住机械臂 Unlock 键，拖动到目标位置")
    print("  2. 松开 Unlock 键，保持静止约 1 秒")
    print("  3. 程序将自动显示当前坐标（不保存）")
    print("  按 'q' 退出程序")
    print("---------------------------------------------------\n")

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break

            # ---------------- 运动检测逻辑 ----------------
            curr_pose_snapshot = current_pose[:] # 快照
            dist = calculate_distance(curr_pose_snapshot, last_pose)
            
            now = time.time()
            
            if dist > MOVE_THRESHOLD:
                # 正在移动
                if not is_moving:
                    print("\r[状态] 正在拖动...", end="")
                is_moving = True
                last_move_time = now
                last_pose = curr_pose_snapshot # 更新“上一次”位置
            else:
                # 当前看似静止
                if is_moving:
                    # 之前在动，现在停了，检查停了多久
                    stable_time = now - last_move_time
                    print(f"\r[状态] 正在稳定... {stable_time:.1f}s", end="")
                    
                    if stable_time > STABLE_DURATION:
                        # 稳定时间达标，触发保存
                        record_count += 1
                        save_record(frame, curr_pose_snapshot, record_count)
                        is_moving = False # 重置状态
                        # 更新 last_pose，避免微小抖动重复触发
                        last_pose = curr_pose_snapshot
                else:
                    # 一直静止，更新 last_pose 以适应缓慢漂移
                    last_pose = curr_pose_snapshot

            # ---------------- 界面显示 ----------------
            # 在画面上显示坐标
            info_text = f"X:{curr_pose_snapshot[0]:.1f} Y:{curr_pose_snapshot[1]:.1f} Z:{curr_pose_snapshot[2]:.1f}"
            status_text = "MOVING" if is_moving else "IDLE"
            color = (0, 0, 255) if is_moving else (0, 255, 0)
            
            cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            cv2.putText(frame, status_text, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
            cv2.putText(frame, f"Records: {record_count}", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            cv2.imshow("Dobot Manual Recorder", frame)
            
            key = cv2.waitKey(10) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                record_count += 1
                save_record(frame, curr_pose_snapshot, record_count)

    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        cv2.destroyAllWindows()
        dType.DisconnectDobot(api)
        print("\n程序退出。")

if __name__ == "__main__":
    main()
