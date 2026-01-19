import multiprocessing
import time
import sys
import os
import numpy as np

# 尝试导入 PyBullet
try:
    import pybullet as p
    import pybullet_data
except ImportError:
    print("请先安装 PyBullet: pip install pybullet")
    sys.exit(1)

# 定义机械臂的任务逻辑
def arm_process(port, label, target_pos, shared_state):
    """
    port: 串口号 (e.g., "COM3")
    label: 机械臂标签 (e.g., "Left_Arm")
    target_pos: 目标坐标字典 {"x":..., "y":..., "z":..., "r":...}
    shared_state: 共享字典，用于存储实时关节角度
    """
    # 动态添加路径以导入 DobotMagician 模块
    sys.path.append(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
    from DobotMagician import DobotDllType as dType

    # 加载 DLL
    api = dType.load()
    
    print(f"[{label}] 正在连接 {port}...")
    state = dType.ConnectDobot(api, port, 115200)[0]
    
    if state != dType.DobotConnect.DobotConnect_NoError:
        print(f"[{label}] 连接失败! 请检查串口号是否正确。")
        return

    print(f"[{label}] 连接成功!")

    dType.SetQueuedCmdClear(api)
    dType.SetPTPCommonParams(api, 100, 100, isQueued=1)
    dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100, isQueued=1)
    
    # 回零 (可选)
    idx = dType.SetHOMECmd(api, 0, isQueued=1)[0]
    while idx > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)
    
    print(f"[{label}] 移动到目标: {target_pos}")
    idx = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, 
                          target_pos["x"], target_pos["y"], target_pos["z"], target_pos["r"], 
                          isQueued=1)[0]
    
    # 循环等待动作完成，并实时更新关节角度
    while idx > dType.GetQueuedCmdCurrentIndex(api)[0]:
        pose = dType.GetPose(api)
        # 更新共享内存 [j1, j2, j3, j4]
        shared_state[label] = [pose[4], pose[5], pose[6], pose[7]]
        dType.dSleep(20) # 提高一点刷新率
        
    print(f"[{label}] 动作完成!")
    
    # 保持最后状态一会儿
    time.sleep(2)
    dType.DisconnectDobot(api)

if __name__ == "__main__":
    # ================= 配置区域 =================
    PORT_1 = "COM5" 
    PORT_2 = "COM6"
    
    POS_1 = {"x": 250, "y": 0, "z": 50, "r": 0}
    POS_2 = {"x": 250, "y": 0, "z": 50, "r": 0}
    
    # URDF 文件路径
    URDF_PATH = r"e:\DualArm\DualArm_Demo\magician-master\urdf\magician_pybullet.urdf"
    # ===========================================

    print("启动双臂控制 + PyBullet 仿真 Demo...")
    
    manager = multiprocessing.Manager()
    shared_state = manager.dict()
    shared_state["Arm_1"] = [0, 0, 0, 0]
    shared_state["Arm_2"] = [0, 0, 0, 0]
    
    # 启动 PyBullet GUI
    p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)
    p.loadURDF("plane.urdf")
    
    # 加载两个机械臂
    # 配置：两个机械臂沿 X 轴放置，相距 0.6米，面对面
    
    # Arm 1 (左侧): 位置 x=-0.3, y=0, z=0. 旋转: 0 (朝向+X)
    pos1 = [-0.3, 0, 0]
    ori1 = p.getQuaternionFromEuler([0, 0, 0])
    
    # Arm 2 (右侧): 位置 x=0.3, y=0, z=0. 旋转: pi (朝向-X)
    pos2 = [0.3, 0, 0]
    ori2 = p.getQuaternionFromEuler([0, 0, 3.14159265]) # 旋转 180 度
    
    print(f"加载 URDF: {URDF_PATH}")
    try:
        dobot1 = p.loadURDF(URDF_PATH, basePosition=pos1, baseOrientation=ori1, useFixedBase=True)
        dobot2 = p.loadURDF(URDF_PATH, basePosition=pos2, baseOrientation=ori2, useFixedBase=True)
    except Exception as e:
        print(f"加载 URDF 失败: {e}")
        sys.exit(1)

    # 映射 PyBullet 关节索引
    # 根据 urdf 文件：
    # joint_1 (Base) -> Index 0
    # joint_2 (Rear) -> Index 1
    # joint_5 (Fore) -> Index 2
    # joint_6 (End)  -> Index 3
    
    joints = {}
    for i in range(p.getNumJoints(dobot1)):
        info = p.getJointInfo(dobot1, i)
        joint_name = info[1].decode('utf-8')
        joints[joint_name] = i
        
    # 启动控制进程
    p1 = multiprocessing.Process(target=arm_process, args=(PORT_1, "Arm_1", POS_1, shared_state))
    p2 = multiprocessing.Process(target=arm_process, args=(PORT_2, "Arm_2", POS_2, shared_state))
    
    p1.start()
    p2.start()
    
    print("开始仿真循环...")
    try:
        while p1.is_alive() or p2.is_alive():
            # 更新 Arm 1
            j_1 = shared_state.get("Arm_1", [0,0,0,0])
            j1_rad = np.radians(j_1[0])
            j2_rad = np.radians(j_1[1])
            j3_rad = np.radians(j_1[2])
            j4_rad = np.radians(j_1[3])
            j3_urdf = j3_rad - j2_rad
            
            p.resetJointState(dobot1, joints.get("joint_1", 0), j1_rad)
            p.resetJointState(dobot1, joints.get("joint_2", 1), j2_rad)
            p.resetJointState(dobot1, joints.get("joint_5", 2), j3_urdf) 
            p.resetJointState(dobot1, joints.get("joint_6", 3), j4_rad)

            # 更新 Arm 2
            j_2 = shared_state.get("Arm_2", [0,0,0,0])
            j1_rad = np.radians(j_2[0])
            j2_rad = np.radians(j_2[1])
            j3_rad = np.radians(j_2[2])
            j4_rad = np.radians(j_2[3])
            j3_urdf = j3_rad - j2_rad
            
            p.resetJointState(dobot2, joints.get("joint_1", 0), j1_rad)
            p.resetJointState(dobot2, joints.get("joint_2", 1), j2_rad)
            p.resetJointState(dobot2, joints.get("joint_5", 2), j3_urdf)
            p.resetJointState(dobot2, joints.get("joint_6", 3), j4_rad)
            
            p.stepSimulation()
            time.sleep(0.02)
            
    except KeyboardInterrupt:
        pass
    
    print("等待子进程结束...")
    p1.join()
    p2.join()
    p.disconnect()
    print("程序结束")
