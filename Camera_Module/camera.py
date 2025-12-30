import cv2
import math
from ultralytics import YOLO

# 1. 加载模型
model = YOLO('runs/obb/train/weights/best.pt')

# 2. 打开摄像头
# 如果 0 不行，请尝试 1 或 2；CAP_DSHOW 在 Windows 上通常比 MSMF 更稳定
cap = cv2.VideoCapture(1, cv2.CAP_MSMF)

if not cap.isOpened():
    print("无法打开摄像头，请检查索引号或权限")
    exit()

print("程序正在运行... 按 'q' 键退出")

while True:
    success, frame = cap.read()
    if not success:
        print("无法获取画面")
        break

    # 3. 执行推理
    # 注意：在 while 循环手动读取帧时，通常不使用 stream=True，直接 predict 即可
    results = model.predict(frame, conf=0.5, show=False)

    # 先初始化要显示的画面：默认就是原始画面
    # 这样即使没有检测到目标，窗口也不会报错或卡住
    display_frame = frame.copy()

    for r in results:
        # 使用 YOLO 自带的 plot() 生成带有检测框的图像
        display_frame = r.plot() 

        # 获取 OBB 框的数据
        if r.obb is not None:
            obb_data = r.obb.xywhr.cpu().numpy()
            classes = r.obb.cls.cpu().numpy()

            for i, box in enumerate(obb_data):
                x, y, w, h, angle_rad = box
                
                # 将弧度转换为角度
                angle_deg = math.degrees(angle_rad)
                
                # 获取类别名称
                class_id = int(classes[i])
                class_name = model.names[class_id]

                # 打印结果到控制台
                print(f"目标: {class_name} | 中心: ({x:.1f}, {y:.1f}) | 角度: {angle_deg:.2f}°")

                # --- 在 display_frame 上进行自定义绘制 ---
                # 绘制中心点（红色实心圆）
                cv2.circle(display_frame, (int(x), int(y)), 5, (0, 0, 255), -1)
                
                # 在画面上标注具体的角度数值
                cv2.putText(display_frame, f"Ang: {angle_deg:.1f}deg", (int(x) + 10, int(y) - 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

    # 4. 显示结果窗口
    cv2.imshow("YOLOv11 OBB Real-time", display_frame)

    # 5. 按 'q' 键退出
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# 释放资源
cap.release()
cv2.destroyAllWindows()