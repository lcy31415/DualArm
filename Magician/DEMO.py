import DobotDllType as dType
import time

api = dType.load()
state = dType.ConnectDobot(api, "", 115200)[0]

if state == dType.DobotConnect.DobotConnect_NoError:
    print("连接成功！")
    
    # 1. 清空队列
    dType.SetQueuedCmdClear(api)
    
    # 2. 强行使能电机 (确保电机是有电锁死状态)
    dType.SetQueuedCmdStartExec(api)

    # 3. [关键步骤] 执行回零动作
    # 机械臂会先升起大臂再旋转寻找原点，请确保周围没障碍物
    print("正在执行回零，请注意安全...")
    last_index = dType.SetHOMECmd(api, 0, isQueued=1)[0]
    
    # 等待回零完成
    while last_index > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(200)
    print("回零完成！")

    # 4. 获取当前位姿
    pose = dType.GetPose(api)
    print(f"当前位置: X={pose[0]:.2f}, Y={pose[1]:.2f}, Z={pose[2]:.2f}")

    # 5. 设置运动参数
    dType.SetPTPCommonParams(api, 50, 50, isQueued=1)
    dType.SetPTPJointParams(api, 100, 100, 100, 100, 100, 100, 100, 100, isQueued=1)

    # 6. 选择三个不同位置，到位后依次旋转到 0、90、-90、0 度
    positions = [
        (181.0382843017578, 29.961204528808594, 6.1058349609375),
    ]
    sequence = [0.0, 90.0, -90.0, 0.0]
    for x, y, z in positions:
        print(f"移动到目标点位: X={x:.3f} Y={y:.3f} Z={z:.3f} R=0.000")
        idx_move = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, x, y, z, 0.0, isQueued=1)[0]
        last_index = idx_move
        for r in sequence:
            last_index = dType.SetPTPCmd(api, dType.PTPMode.PTPMOVJXYZMode, x, y, z, r, isQueued=1)[0]
            dType.SetWAITCmd(api, 300, isQueued=1)

    # 等待执行
    while last_index > dType.GetQueuedCmdCurrentIndex(api)[0]:
        dType.dSleep(100)

    print("已在三个位置完成抓夹 0/90/-90/0 旋转。")

    # 7. 断开连接
    dType.SetQueuedCmdStopExec(api)
    dType.DisconnectDobot(api)
    print("连接已断开")

else:
    print("连接失败")
