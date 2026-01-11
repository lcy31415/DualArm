## 目标
- 在 E:\DualArm\Block_Palletizing 新建独立流程，实现红/绿/蓝小物块的分类码垛。
- main.py 自动并行启动 Detect.py 与 ArmControl.py；Detect 仅在机械臂就位后输出坐标（相机画面持续显示但停止写坐标）；ArmControl 在检测完成后读取坐标分拣搬运，结束后回到安全位并触发下一轮检测。

## 目录结构
- E:\DualArm\Block_Palletizing/
  - main.py：流程编排与状态机控制
  - Detect.py：颜色检测，按需输出坐标，预览窗口常开
  - ArmControl.py：读取 npz 标定文件与坐标数据，完成分拣搬运
  - config.json：用户配置停止位、投放位与高度参数
  - data/
    - status.json：状态机共享文件（当前轮次与阶段）
    - blocks_<cycle_id>.json：本轮检测坐标数据（像素坐标与标签）
  - logs/：运行日志（可选）

## 运行依赖
- 根目录标定文件：
  - E:\DualArm\camera_params.npz（mtx、dist）
  - E:\DualArm\handeye_result.npz（Rc2b、tc2b）
- 机械臂库：E:\DualArm\DobotMagician（已迁移）
- 环境：在 PowerShell 中执行 E:\DualArm\set_dobot_env.ps1 以注入 PATH/PYTHONPATH；建议从 DobotMagician 目录启动。

## 数据与协议
- status.json（由 main 初始化、两个子进程读写）
  - {"cycle": 1, "arm_state": "idle|ready|busy|done", "detect_state": "sleep|capturing|done", "request_next": false}
- blocks_<cycle>.json（Detect 产出，ArmControl 消费）
  - 数组元素：
    - color: "red"|"green"|"blue"
    - cx, cy: 像素坐标（float）
    - side: "left"|"right"（基于画面左右半区归类）
    - score: 置信度（可选）
    - area: 轮廓面积（可选）

## 检测逻辑（Detect.py）
- 读取 status.json；当 arm_state=="ready" 时进入捕获（detect_state="capturing"）；
- 使用 HSV 阈值与形态学提取 red/green/blue；取最大轮廓中心作为 (cx, cy)；按左右半区填入 side；
- 直到至少收集到左右两侧的坐标（或到达可配置超时），将数据写入 blocks_<cycle>.json，置 detect_state="done"；
- 停止继续写坐标，但保持预览窗口开启；当 request_next==true 时进入下一轮（detect_state="sleep"→"capturing"）。

## 控制逻辑（ArmControl.py）
- 读取 config.json：
  - stop_pose：机械臂停止位（x,y,z,r）
  - bins：三色投放位（red/green/blue 的 (x,y)）
  - z_workplane：台面高度；approach_z/pick_z/place_z；flip_y（是否按需取反 y）
- 启动后回零（HOME）并移动至 stop_pose；写入 status.arm_state="ready"；终端输出“机械臂已就位”。
- 轮询 status.detect_state；当为 "done" 时，读取 blocks_<cycle>.json；按 color 分类与 side 组织队列：
  - 像素→基座坐标转换：
    - undistortPoints 获取 (x_n,y_n)；
    - dir_c=[x_n,y_n,1]^T；dir_b=Rc2b@dir_c；org_b=tc2b；
    - t=-(org_b.z - z_workplane)/dir_b.z；pb=org_b + t*dir_b → (x,y,z_workplane)
    - 可选 flip_y：y=-y。
  - 动作序列：
    - 到 (x,y,approach_z)→降至 pick_z→夹取→升至 approach_z→到 bins[color] 的 (bx,by,approach_z)→降至 place_z→放置→升至 approach_z。
- 全部分拣完成后，回到安全位（stop_pose 或 home），写入 arm_state="done" 并将 request_next=true 以触发下一轮检测。

## 编排逻辑（main.py）
- 初始化 data/status.json：cycle=1，arm_state="idle"，detect_state="sleep"，request_next=false；
- 并行启动 Detect.py 与 ArmControl.py（子进程）并监控状态：
  - 等待 arm_state="ready"→设置 detect_state="capturing"；
  - 等待 detect_state="done"→允许 ArmControl 开始搬运；
  - 等待 arm_state="done"→递增 cycle，写 request_next=true；重置 detect_state="sleep"、arm_state="idle"；进入下一轮。
- 输出关键阶段日志与错误提示（超时、空检测、坐标转换失败等）。

## 关键参数与配置
- Block_Palletizing/config.json 示例：
  - stop_pose: {"x": 260, "y": -120, "z": 60, "r": 0}
  - bins: {"red": {"x": 200, "y": -50}, "green": {"x": 230, "y": -50}, "blue": {"x": 260, "y": -50}}
  - z_workplane: 0
  - approach_z: 30, pick_z: -5, place_z: 0
  - flip_y: true|false（按现场方向决定）
  - timeouts: {"detect": 8.0, "arm": 60.0}

## 健壮性与异常处理
- Detect 超时：若在设定时间内未收集到两侧坐标，仍将已有数据写入并置 done；ArmControl 按已有目标分拣。
- 射线与平面平行或转换失败：跳过该目标并记录日志。
- 可达性检查：在分拣前检查 (x,y,z) 是否在机械臂工作空间与安全高度范围内。
- 连接或相机异常：在 status.json 中写入错误字段，main.py 负责通知并停止流程。

## 运行方式
- 在 PowerShell 中：
  - 执行 E:\DualArm\set_dobot_env.ps1（确保库可导入）
  - python E:\DualArm\Block_Palletizing\main.py
- main.py 自动启动 Detect 与 ArmControl；按上述状态机循环处理。

## 交付内容
- 创建 Block_Palletizing 目录与以下文件（确认后实施）：
  - main.py、Detect.py、ArmControl.py、config.json、data/status.json（初始）、logs（可选）。
- 代码遵循现有库的调用方式，不改动已有模块；如需 DLL 相对路径加载，main 将设定工作目录或提示使用 DobotMagician 目录运行。