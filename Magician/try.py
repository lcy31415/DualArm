import DobotDllType as dType
import time
import cv2
import os
import random

# ---------------- 配置参数 ----------------
# 初始位置 (X, Y, Z, R)
INIT_POSE = [181.038, 29.961, 6.106, 9.397]
# 采集数量
CAPTURE_COUNT = 15
# 每个位置停留时间 (秒)
WAIT_TIME = 5
# 位姿变化范围 (mm 和 度)
OFFSET_RANGE = {
    'x': (-15, 15),
    'y': (-15, 15),
    'z': (0, 10),   # Z轴只向上微调，防止撞击
    'r': (-10, 10)
}
# 数据保存路径
DATA_DIR = "calibration_data"
IMAGE_DIR = os.path.join(DATA_DIR, "images")
POSE_FILE = os.path.join(DATA_DIR, "poses.txt")

# ---------------- 全局变量 ----------------
api = None
CON_STR = {
    dType.DobotConnect.DobotConnect_NoError:  "DobotConnect_NoError",
    dType.DobotConnect.DobotConnect_NotFound: "DobotConnect_NotFound",
    dType.DobotConnect.DobotConnect_Occupied: "DobotConnect_Occupied"
}

def init_dobot():
    """初始化并连接机械臂"""
    global api
    print("正在加载 Dobot DLL...")
    api = dType.load()
    
    print("正在连接机械臂...")
    state = dType.ConnectDobot(api, "", 115200)[0]
    print("连接状态:", CON_STR[state])
    
    if state == dType.DobotConnect.DobotConnect_NoError:
        # 清空队列
        dType.SetQueuedCmdClear(api)
        # 启动队列执行（关键步骤）
        dType.SetQueuedCmdStartExec(api)
        # 设置运动参数（降低速度以实现缓慢移动）
        dType.SetPTPCommonParams(api, 20, 20, isQueued=1)
        dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100, isQueued=1)
        return True
    else:
        print("无法连接机械臂，请检查连接！")
        return False

def wait_for_cmd(last_index):
    """等待指令队列执行完成"""
    while last_index > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)

def move_to(x, y, z, r):
    """移动到指定坐标"""
    print(f"移动至: X={x:.3f}, Y={y:.3f}, Z={z:.3f}, R={r:.3f}")
    last_index = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, x, y, z, r, isQueued=1)[0]
    wait_for_cmd(last_index)
    # 到位校验
    pose = dType.GetPose(api)
    dx, dy, dz = abs(pose[0]-x), abs(pose[1]-y), abs(pose[2]-z)
    dr = abs(pose[3]-r)
    print(f"到位偏差: dX={dx:.3f} dY={dy:.3f} dZ={dz:.3f} dR={dr:.3f}")

def do_homing():
    """执行回零"""
    print("正在执行回零，请注意安全...")
    last_index = dType.SetHOMECmd(api, temp=0, isQueued=1)[0]
    wait_for_cmd(last_index)
    print("回零完成！")

def main():
    """主函数"""
    # 创建保存目录
    os.makedirs(IMAGE_DIR, exist_ok=True)
    
    # 初始化机械臂
    if not init_dobot():
        return
    
    try:
        # 执行回零
        do_homing()
        
        # 1. 移动到初始位置
        print("\n=== 步骤1: 移动到初始位置 ===")
        move_to(*INIT_POSE)
        time.sleep(0.5)
        
        # 2. 执行一次夹取动作（张开然后夹紧）
        print("\n=== 步骤2: 执行夹取动作 ===")
        print("张开夹爪...")
        last_index = dType.SetEndEffectorGripper(api, 1, 0, isQueued=1)[0]  # 0表示张开
        wait_for_cmd(last_index)
        time.sleep(2)
        print("夹紧夹爪...")
        last_index = dType.SetEndEffectorGripper(api, 1, 1, isQueued=1)[0]  # 1表示夹紧
        wait_for_cmd(last_index)
        print("夹取动作完成")
        
        # 3. 打开摄像头索引为1并显示
        print("\n=== 步骤3: 打开摄像头 ===")
        cap = cv2.VideoCapture(1, cv2.CAP_MSMF)
        if not cap.isOpened():
            print("错误：无法打开摄像头索引1！")
            return
        
        # 读取一帧并显示，确认摄像头已打开
        ret, frame = cap.read()
        if not ret:
            print("错误：无法获取摄像头画面！")
            cap.release()
            return
        
        cv2.imshow("Camera Preview", frame)
        cv2.waitKey(500)
        print("摄像头已打开并显示")
        
        # 4. 开始15个位姿的数据采集
        print("\n=== 步骤4: 开始位姿数据采集 ===")
        with open(POSE_FILE, "w", encoding='utf-8') as f:
            f.write("image_name,x,y,z,r\n")  # CSV Header
            
            for i in range(CAPTURE_COUNT):
                print(f"\n--- 采集进度: {i+1}/{CAPTURE_COUNT} ---")
                
                # 生成基于初始位置的随机目标点
                tx = INIT_POSE[0] + random.uniform(*OFFSET_RANGE['x'])
                ty = INIT_POSE[1] + random.uniform(*OFFSET_RANGE['y'])
                tz = INIT_POSE[2] + random.uniform(*OFFSET_RANGE['z'])
                tr = INIT_POSE[3] + random.uniform(*OFFSET_RANGE['r'])
                
                # 缓慢移动到目标位置
                move_to(tx, ty, tz, tr)
                
                # 停留5秒
                print(f"停留 {WAIT_TIME} 秒...")
                time.sleep(WAIT_TIME)
                
                # 读取摄像头画面
                ret, frame = cap.read()
                if not ret:
                    print("错误：无法获取图像帧！")
                    continue
                
                # 获取当前实际位姿
                pose = dType.GetPose(api)
                real_x, real_y, real_z, real_r = pose[0], pose[1], pose[2], pose[3]
                print(f"当前实际位姿: X={real_x:.3f}, Y={real_y:.3f}, Z={real_z:.3f}, R={real_r:.3f}")
                
                # 在画面上叠加信息并显示
                overlay = frame.copy()
                cv2.putText(overlay, f"Pose X:{real_x:.2f} Y:{real_y:.2f} Z:{real_z:.2f} R:{real_r:.2f}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                cv2.putText(overlay, f"Sample {i+1}/{CAPTURE_COUNT}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("Camera Preview", overlay)
                cv2.waitKey(1)
                
                # 保存图片
                img_filename = f"img_{i}.jpg"
                img_path = os.path.join(IMAGE_DIR, img_filename)
                cv2.imwrite(img_path, frame)
                print(f"已保存图片: {img_path}")
                
                # 保存位姿数据
                f.write(f"{img_filename},{real_x:.6f},{real_y:.6f},{real_z:.6f},{real_r:.6f}\n")
                f.flush()  # 确保写入磁盘
        
        # 释放摄像头资源
        cap.release()
        cv2.destroyAllWindows()
        print("\n=== 数据采集完成！所有数据已保存到", DATA_DIR, "===")
        
    except KeyboardInterrupt:
        print("\n用户中断程序")
    except Exception as e:
        print(f"\n发生错误: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # 断开连接
        print("\n断开机械臂连接...")
        if api:
            dType.SetQueuedCmdStopExec(api)
            dType.DisconnectDobot(api)

if __name__ == "__main__":
    main()

