import cv2
import math
import os
import json
import time
import numpy as np
from ultralytics import YOLO
from config import COORDINATES_FILE, COMMAND_FILE

# 稳定检测参数
STABLE_TOL = 5.0  # 位置波动容忍度
STABLE_FRAMES = 10 # 连续稳定帧数

def check_command():
    if not os.path.exists(COMMAND_FILE):
        return "idle"
    try:
        with open(COMMAND_FILE, "r", encoding="utf-8") as f:
            data = json.load(f)
            return data.get("command", "idle")
    except Exception:
        return "idle"

def set_command(cmd):
    try:
        with open(COMMAND_FILE, "w", encoding="utf-8") as f:
            json.dump({"command": cmd}, f)
    except Exception:
        pass

def main():
    # 1. 加载模型
    model = YOLO(r'E:\DualArm\Camera_Module\runs\obb\train\weights\best.pt')
    
    # 2. 打开摄像头
    cap = cv2.VideoCapture(1, cv2.CAP_MSMF)
    if not cap.isOpened():
        cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    print("YOLO检测程序启动，等待机械臂指令...")
    set_command("idle")
    
    stable_counter = 0
    blocks_prev = []
    
    while True:
        success, frame = cap.read()
        if not success:
            time.sleep(0.1)
            continue

        # 3. 执行推理
        results = model.predict(frame, conf=0.5, show=False)
        display_frame = frame.copy()
        
        current_blocks = []
        
        for r in results:
            display_frame = r.plot() 
            if r.obb is not None:
                obb_data = r.obb.xywhr.cpu().numpy()
                classes = r.obb.cls.cpu().numpy()

                for i, box in enumerate(obb_data):
                    x, y, w, h, angle_rad = box
                    angle_deg = math.degrees(angle_rad) - 90
                    class_id = int(classes[i])
                    class_name = model.names[class_id]
                    
                    # 收集当前帧的检测结果
                    current_blocks.append({
                        "color": class_name, # 复用 color 字段存储类别
                        "x": float(x),
                        "y": float(y),
                        "angle": -float(angle_deg)
                    })

                    # 绘制额外信息
                    cv2.circle(display_frame, (int(x), int(y)), 5, (0, 0, 255), -1)
                    cv2.putText(display_frame, f"{angle_deg:.1f}", (int(x), int(y)), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # --- 握手通信逻辑 ---
        current_cmd = check_command()
        
        if current_cmd == "detect":
            # 稳定性检查
            stable = True
            
            # 简单对比：数量一致且位置接近
            if len(current_blocks) != len(blocks_prev):
                stable = False
            else:
                # 这里的匹配逻辑比较简单，假设顺序基本一致或通过最近邻匹配
                # 为简单起见，这里假设只有一个或少数几个，且 YOLO 顺序相对稳定
                # 更严谨的做法是做匈牙利匹配，但这里先用简单逻辑：
                # 只要每个新块都能在旧块中找到一个距离很近的，就认为稳定
                for b in current_blocks:
                    matched = False
                    for pb in blocks_prev:
                        dist = abs(b["x"] - pb["x"]) + abs(b["y"] - pb["y"])
                        if dist < STABLE_TOL and b["color"] == pb["color"]:
                            matched = True
                            break
                    if not matched:
                        stable = False
                        break
            
            blocks_prev = current_blocks
            
            if stable and len(current_blocks) > 0:
                stable_counter += 1
            else:
                stable_counter = 0
                
            cv2.putText(display_frame, f"Scanning... {stable_counter}/{STABLE_FRAMES}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            
            if stable_counter >= STABLE_FRAMES:
                try:
                    os.makedirs(os.path.dirname(COORDINATES_FILE), exist_ok=True)
                    data = {"timestamp": time.time(), "blocks": current_blocks}
                    with open(COORDINATES_FILE, "w", encoding="utf-8") as f:
                        json.dump(data, f, ensure_ascii=False, indent=2)
                    
                    print(f"检测稳定，写入 {len(current_blocks)} 个目标")
                    set_command("done")
                    stable_counter = 0
                except Exception as e:
                    print(f"写入失败: {e}")
        else:
            stable_counter = 0
            cv2.putText(display_frame, f"Status: {current_cmd.upper()}", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2)

        cv2.imshow("YOLOv11 OBB Real-time", display_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()