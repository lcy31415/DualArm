import cv2
import numpy as np
import time

def main():
    # 棋盘格规格 (内角点行列数)
    # 请根据您的实际棋盘格修改，例如 (3, 3) 或 (9, 6)
    PATTERN_SIZE = (3, 3)
    
    # 初始化摄像头
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        print("无法打开摄像头")
        return

    # 设置分辨率 (可选)
    # cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    # cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    print("摄像头预览启动。按 'q' 退出，按 's' 截图并打印坐标。")
    print(f"寻找 {PATTERN_SIZE[0]}x{PATTERN_SIZE[1]} 的棋盘格...")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.01)
            continue
            
        display_frame = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # 寻找角点
        found, corners = cv2.findChessboardCorners(gray, PATTERN_SIZE, 
                                                   cv2.CALIB_CB_ADAPTIVE_THRESH + 
                                                   cv2.CALIB_CB_NORMALIZE_IMAGE +
                                                   cv2.CALIB_CB_FAST_CHECK)
        
        if found:
            # 亚像素精确化
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # 绘制角点
            cv2.drawChessboardCorners(display_frame, PATTERN_SIZE, corners, found)
            
            # 在画面上标注坐标
            # corners 的形状是 (N, 1, 2)
            for i, corner in enumerate(corners):
                x, y = corner.ravel()
                # 只标注第1个和最后一个，避免太乱，或者标注全部
                cv2.circle(display_frame, (int(x), int(y)), 4, (0, 0, 255), -1)
                cv2.putText(display_frame, f"{i}", (int(x)+5, int(y)-5), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # 显示
        cv2.imshow("Chessboard Detector", display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            if found:
                print("\n=== 检测到棋盘格角点 (像素坐标) ===")
                # 格式化打印
                print(f"{'Index':<5} | {'X':<10} | {'Y':<10}")
                print("-" * 30)
                for i, corner in enumerate(corners):
                    x, y = corner.ravel()
                    print(f"{i:<5} | {x:<10.2f} | {y:<10.2f}")
                print("===================================\n")
                
                # 保存图片
                filename = f"chessboard_corners_{int(time.time())}.jpg"
                cv2.imwrite(filename, display_frame)
                print(f"已保存截图: {filename}")
            else:
                print("当前画面未检测到棋盘格！")

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
