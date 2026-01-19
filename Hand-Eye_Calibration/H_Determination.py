import os
import cv2
import numpy as np

def load_dual_affine_matrices():
    base_dir = os.path.dirname(__file__)
    matrix_path = os.path.join(base_dir, "affine_matrix_dual.npz")
    
    if not os.path.exists(matrix_path):
        print(f"找不到矩阵文件: {matrix_path}")
        return None
    
    data = np.load(matrix_path)
    matrices = {}
    if 'affine_left' in data:
        matrices['left'] = data['affine_left']
    if 'affine_right' in data:
        matrices['right'] = data['affine_right']
        
    return matrices

def pixel_to_base_affine(u, v, M):
    """
    使用仿射矩阵进行变换
    [x, y] = M * [u, v, 1]^T
    """
    vec = np.array([u, v, 1.0])
    res = M @ vec
    return float(res[0]), float(res[1])

def on_mouse(event, x, y, flags, param):
    """
    鼠标回调：点击画面获取坐标
    """
    if event == cv2.EVENT_LBUTTONDOWN:
        param['clicked_uv'] = (float(x), float(y))
        param['update'] = True

def main():
    matrices = load_dual_affine_matrices()
    if not matrices:
        print("未加载到任何有效矩阵，程序退出。")
        return

    print("已加载矩阵: ", list(matrices.keys()))
    
    # 打开摄像头
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 状态字典，用于鼠标回调通信
    state = {
        'clicked_uv': None,
        'update': False
    }

    win_name = "H Determination (Click to Measure)"
    cv2.namedWindow(win_name)
    cv2.setMouseCallback(win_name, on_mouse, state)
    
    print("\n操作说明:")
    print("1. 在画面上点击任意位置，查看该点在机械臂坐标系下的计算值")
    print("2. 仿射变换矩阵已经包含了几何关系，不需要额外调整 H 参数")
    print("3. 按 'q' 退出")

    while True:
        ret, frame = cap.read()
        if not ret:
            continue
            
        display_frame = frame.copy()
        
        # 如果有点击点，绘制出来
        if state['clicked_uv']:
            u, v = state['clicked_uv']
            cv2.circle(display_frame, (int(u), int(v)), 5, (0, 0, 255), -1)
            
            # 计算并显示坐标
            y_offset = 30
            for arm_name, M in matrices.items():
                if M is not None:
                    rx, ry = pixel_to_base_affine(u, v, M)
                    text = f"{arm_name.upper()}: ({rx:.1f}, {ry:.1f})"
                    cv2.putText(display_frame, text, (int(u)+10, int(v)+y_offset), 
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    y_offset += 25

        cv2.imshow(win_name, display_frame)
        
        key = cv2.waitKey(10) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
