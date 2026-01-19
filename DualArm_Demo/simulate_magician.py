import pybullet as p
import pybullet_data
import time
import math
import os
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", default="gui", choices=["gui", "direct"], help="Simulation mode: gui or direct")
    args = parser.parse_args()

    # 1. 准备 URDF 文件
    # 假设脚本在项目根目录下运行
    original_urdf_path = os.path.join("magician-master", "urdf", "demo.urdf")
    new_urdf_path = os.path.join("magician-master", "urdf", "magician_pybullet.urdf")

    if not os.path.exists(original_urdf_path):
        print(f"Error: Could not find {original_urdf_path}")
        return

    # 读取原始 URDF
    with open(original_urdf_path, "r", encoding="utf-8") as f:
        urdf_content = f.read()

    # 替换路径
    # 将 package://magician/ 替换为 ../
    # 这样 ../meshes/... 就能正确找到 meshes 目录（相对于 urdf 目录）
    new_urdf_content = urdf_content.replace("package://magician/", "../")

    # 写入新的 URDF
    with open(new_urdf_path, "w", encoding="utf-8") as f:
        f.write(new_urdf_content)

    print(f"Generated new URDF at {new_urdf_path}")

    # 2. 连接 PyBullet
    if args.mode == "gui":
        try:
            physicsClient = p.connect(p.GUI)
        except Exception:
            print("Could not connect to GUI, falling back to DIRECT mode")
            physicsClient = p.connect(p.DIRECT)
    else:
        physicsClient = p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.8)

    # 3. 加载环境
    planeId = p.loadURDF("plane.urdf")
    
    # 加载桌子
    tableStartPos = [0, 0, 0]
    tableStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
    tableId = p.loadURDF("table/table.urdf", tableStartPos, tableStartOrientation, useFixedBase=True)

    # 4. 加载机器人
    # 桌子高度通常是 0.625 左右，我们需要把机器人放在桌子上
    # 稍微调整 Z 轴高度以避免碰撞
    startPos = [0, 0, 0.625]
    startOrientation = p.getQuaternionFromEuler([0, 0, 0])
    
    try:
        # useFixedBase=True 将机器人底座固定在空间中（即固定在桌面上）
        robotId = p.loadURDF(new_urdf_path, startPos, startOrientation, useFixedBase=True)
    except Exception as e:
        print(f"Failed to load robot: {e}")
        p.disconnect()
        return

    # 获取关节信息
    numJoints = p.getNumJoints(robotId)
    print(f"Number of joints: {numJoints}")

    joint_indices = []
    joint_debug_params = {} # 存储 slider ID

    for i in range(numJoints):
        info = p.getJointInfo(robotId, i)
        joint_name = info[1].decode('utf-8')
        joint_type = info[2]
        lower_limit = info[8]
        upper_limit = info[9]
        
        # 如果 limit 都是 0，说明 URDF 可能没定义 limit，或者确实是 0
        # 给一个默认范围，方便调试
        if lower_limit == 0 and upper_limit == 0:
            lower_limit = -3.14
            upper_limit = 3.14

        print(f"Joint {i}: {joint_name}, type: {joint_type}, limits: [{lower_limit}, {upper_limit}]")
        
        if joint_type != p.JOINT_FIXED:
            joint_indices.append(i)
            # 添加 slider
            param_id = p.addUserDebugParameter(joint_name, lower_limit, upper_limit, 0)
            joint_debug_params[i] = param_id

    # 5. 简单的控制循环
    print("Starting simulation...")
    t = 0
    try:
        while True:
            p.stepSimulation()
            if p.getConnectionInfo(physicsClient)['connectionMethod'] == p.GUI:
                time.sleep(1./240.)
            
            t += 0.01
            
            # 读取 slider 值并控制关节
            for joint_index, param_id in joint_debug_params.items():
                if p.getConnectionInfo(physicsClient)['connectionMethod'] == p.GUI:
                     targetPos = p.readUserDebugParameter(param_id)
                     p.setJointMotorControl2(robotId, joint_index, p.POSITION_CONTROL, targetPosition=targetPos)
                else:
                    # In direct mode, keep old behavior or do nothing
                    pass
            
            # 在 DIRECT 模式下打印一些状态以验证运行

            if p.getConnectionInfo(physicsClient)['connectionMethod'] == p.DIRECT and int(t*100) % 100 == 0:
                 states = p.getJointStates(robotId, joint_indices)
                 positions = [state[0] for state in states]
                 print(f"Time: {t:.2f}, Joint Positions: {[f'{pos:.2f}' for pos in positions]}")
                 if t > 5: # DIRECT 模式下运行 5 秒后退出
                     break
                     
    except KeyboardInterrupt:
        pass
    finally:
        p.disconnect()
        print("Simulation finished.")

if __name__ == "__main__":
    main()
